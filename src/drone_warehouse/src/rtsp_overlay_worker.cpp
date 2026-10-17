#include "drone_warehouse/rtsp_overlay_worker.hpp"

#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <chrono>
static GstBusSyncReply overlay_bus_sync_handler(GstBus *, GstMessage *message, gpointer user_data)
{
    if (!gst_is_video_overlay_prepare_window_handle_message(message)) return GST_BUS_PASS;
    auto *overlay = GST_VIDEO_OVERLAY(GST_MESSAGE_SRC(message));
    const auto handle = *static_cast<quintptr *>(user_data);
    gst_video_overlay_set_window_handle(overlay, handle);
    gst_video_overlay_set_render_rectangle(overlay, 0, 0, 640, 480);
    gst_message_unref(message);
    return GST_BUS_DROP;

}
RtspOverlayWorker::RtspOverlayWorker(QObject *parent) : QObject(parent) {}

RtspOverlayWorker::~RtspOverlayWorker() { stop(); }

void RtspOverlayWorker::start(const QString &url, quintptr window_handle)
{
    if (running_.exchange(true)) return;
    stop_requested_ = false;
    thread_ = std::thread(&RtspOverlayWorker::run, this, url, window_handle);
}

void RtspOverlayWorker::stop()
{
    stop_requested_ = true;
    if (thread_.joinable()) thread_.join();
    running_ = false;
}

void RtspOverlayWorker::run(QString url, quintptr window_handle)
{
    gst_init(nullptr, nullptr);
    emit statusChanged(QStringLiteral("视频启动中"));
    while (!stop_requested_) {
        GError *parse_error = nullptr;
        const QByteArray location = url.toUtf8();
        const QString description = QStringLiteral(
            "rtspsrc location=%1 protocols=udp latency=50 drop-on-latency=true "
            "! rtph264depay ! h264parse config-interval=-1 ! mppvideodec "
            "! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! ximagesink sync=false").arg(QString::fromUtf8(location));
        GstElement *pipeline = gst_parse_launch(description.toUtf8().constData(), &parse_error);
        if (!pipeline) {
            const QString message = parse_error ? QString::fromUtf8(parse_error->message)
                                                : QStringLiteral("无法创建视频管线");
            if (parse_error) g_error_free(parse_error);
            emit statusChanged(message);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "ximagesink0");
        if (sink && GST_IS_VIDEO_OVERLAY(sink)) {
            gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(sink), window_handle);
            gst_video_overlay_set_render_rectangle(GST_VIDEO_OVERLAY(sink), 0, 0, 640, 480);
        }
        if (sink) gst_object_unref(sink);
        GstBus *bus = gst_element_get_bus(pipeline);
        gst_bus_set_sync_handler(bus, overlay_bus_sync_handler, &window_handle, nullptr);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        emit statusChanged(QStringLiteral("视频播放中"));
        bool reconnect = false;
        while (!stop_requested_) {
            GstMessage *message = gst_bus_timed_pop_filtered(
                bus, 200 * GST_MSECOND,
                static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_STATE_CHANGED));
            if (message) {
                if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR ||
                    GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS) {
                    reconnect = true;
                    gst_message_unref(message);
                    break;
                }
                if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_STATE_CHANGED &&
                    GST_MESSAGE_SRC(message) == GST_OBJECT(pipeline)) {
                    GstState old_state, new_state, pending;
                    gst_message_parse_state_changed(message, &old_state, &new_state, &pending);
                    if (new_state == GST_STATE_PLAYING) emit statusChanged(QStringLiteral("视频播放中"));
                }
                gst_message_unref(message);
            }
        }
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(bus);
        gst_object_unref(pipeline);
        if (reconnect && !stop_requested_) {
            for (int i = 0; i < 20 && !stop_requested_; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    emit statusChanged(QStringLiteral("视频已停止"));
    running_ = false;
}
