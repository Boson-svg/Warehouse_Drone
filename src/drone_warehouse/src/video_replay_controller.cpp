#include "drone_warehouse/video_replay_controller.hpp"

#include <gst/gst.h>
#include <gst/video/videooverlay.h>

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFileInfoList>
#include <QFile>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>
#include <QDialog>
#include <QCloseEvent>
#include <QShowEvent>
#include <QWidget>

namespace
{
constexpr int kReplayWidth = 640;
constexpr int kReplayHeight = 480;

GstBusSyncReply replayOverlaySyncHandler(GstBus *, GstMessage *message,
                                         gpointer user_data)
{
    if (!gst_is_video_overlay_prepare_window_handle_message(message))
        return GST_BUS_PASS;
    auto *overlay = GST_VIDEO_OVERLAY(GST_MESSAGE_SRC(message));
    const auto handle = *static_cast<quintptr *>(user_data);
    gst_video_overlay_set_window_handle(overlay, handle);
    gst_video_overlay_set_render_rectangle(
        overlay, 0, 0, kReplayWidth, kReplayHeight);
    gst_message_unref(message);
    return GST_BUS_DROP;
}

class ReplayDialog final : public QDialog
{
public:
    ReplayDialog(const QStringList &files, QWidget *parent)
        : QDialog(parent), files_(files)
    {
        setWindowTitle(QStringLiteral("录像回放"));
        setModal(false);
        setFixedSize(1000, 590);

        auto *root = new QHBoxLayout(this);
        root->setContentsMargins(12, 12, 12, 12);
        root->setSpacing(12);

        auto *left = new QVBoxLayout();
        list_ = new QListWidget(this);
        list_->setMinimumWidth(300);
        list_->setStyleSheet(QStringLiteral(
            "QListWidget { font-size: 22px; }"
            "QListWidget::item { min-height: 48px; padding: 5px; }"));
        auto *recent_label = new QLabel(QStringLiteral("最近录像"), this);
        recent_label->setStyleSheet(QStringLiteral("font-size: 20px; font-weight: 600;"));
        left->addWidget(recent_label);
        left->addWidget(list_, 1);
        root->addLayout(left);

        auto *right = new QVBoxLayout();
        surface_ = new QWidget(this);
        surface_->setFixedSize(kReplayWidth, kReplayHeight);
        surface_->setAttribute(Qt::WA_NativeWindow);
        surface_->setAttribute(Qt::WA_DontCreateNativeAncestors);
        surface_->setAttribute(Qt::WA_OpaquePaintEvent);
        surface_->setStyleSheet(QStringLiteral("background: black"));
        right->addWidget(surface_);

        auto *controls = new QHBoxLayout();
        play_button_ = new QPushButton(QStringLiteral("播放"), this);
        pause_button_ = new QPushButton(QStringLiteral("暂停"), this);
        resume_button_ = new QPushButton(QStringLiteral("继续"), this);
        position_slider_ = new QSlider(Qt::Horizontal, this);
        position_slider_->setRange(0, 1000);
        controls->addWidget(play_button_);
        controls->addWidget(pause_button_);
        controls->addWidget(resume_button_);
        controls->addWidget(position_slider_, 1);
        right->addLayout(controls);
        status_ = new QLabel(QStringLiteral("请选择录像"), this);
        right->addWidget(status_);
        root->addLayout(right, 1);

        for (const auto &file : files_) {
            auto *item = new QListWidgetItem(QFileInfo(file).fileName(), list_);
            item->setToolTip(file);
        }
        if (!files_.isEmpty()) list_->setCurrentRow(0);
        connect(play_button_, &QPushButton::clicked, this, [this]() { playSelected(); });
        connect(pause_button_, &QPushButton::clicked, this, [this]() {
            if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_PAUSED);
        });
        connect(resume_button_, &QPushButton::clicked, this, [this]() {
            if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        });
        connect(list_, &QListWidget::itemDoubleClicked, this,
                [this](QListWidgetItem *) { playSelected(); });
        connect(position_slider_, &QSlider::sliderReleased, this, [this]() {
            if (!pipeline_) return;
            gint64 duration = GST_CLOCK_TIME_NONE;
            if (gst_element_query_duration(pipeline_, GST_FORMAT_TIME, &duration) &&
                duration > 0) {
                const gint64 position = duration * position_slider_->value() / 1000;
                gst_element_seek_simple(pipeline_, GST_FORMAT_TIME,
                                        static_cast<GstSeekFlags>(GST_SEEK_FLAG_FLUSH |
                                                                   GST_SEEK_FLAG_KEY_UNIT),
                                        position);
            }
        });
        poll_timer_ = new QTimer(this);
        connect(poll_timer_, &QTimer::timeout, this, [this]() { updatePosition(); });
    }

    ~ReplayDialog() override { stopPipeline(); }

protected:
    void showEvent(QShowEvent *event) override
    {
        QDialog::showEvent(event);
        if (!poll_timer_->isActive()) poll_timer_->start(500);
    }

    void closeEvent(QCloseEvent *event) override
    {
        stopPipeline();
        QDialog::closeEvent(event);
    }

private:
    void playSelected()
    {
        const int index = list_->currentRow();
        if (index < 0 || index >= files_.size()) {
            status_->setText(QStringLiteral("请先选择录像"));
            return;
        }
        stopPipeline();
        gst_init(nullptr, nullptr);
        GError *parse_error = nullptr;
        pipeline_ = gst_parse_launch(
            "filesrc name=replay_source ! matroskademux ! h264parse "
            "! openh264dec ! videoconvert ! videoscale "
            "! video/x-raw,width=640,height=480 "
            "! ximagesink name=replay_sink sync=true",
            &parse_error);
        if (!pipeline_) {
            status_->setText(parse_error ? QString::fromUtf8(parse_error->message)
                                         : QStringLiteral("回放组件不可用"));
            if (parse_error) g_error_free(parse_error);
            pipeline_ = nullptr;
            return;
        }
        if (parse_error) g_error_free(parse_error);

        GstElement *source = gst_bin_get_by_name(GST_BIN(pipeline_), "replay_source");
        GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline_), "replay_sink");
        if (!source || !sink) {
            status_->setText(QStringLiteral("回放组件不可用"));
            if (source) gst_object_unref(source);
            if (sink) gst_object_unref(sink);
            stopPipeline();
            return;
        }
        const QByteArray file_path = QFile::encodeName(files_.at(index));
        g_object_set(source, "location", file_path.constData(), nullptr);
        gst_object_unref(source);

        window_handle_ = static_cast<quintptr>(surface_->winId());
        gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(sink), window_handle_);
        gst_video_overlay_set_render_rectangle(
            GST_VIDEO_OVERLAY(sink), 0, 0, kReplayWidth, kReplayHeight);
        gst_object_unref(sink);
        bus_ = gst_element_get_bus(pipeline_);
        gst_bus_set_sync_handler(
            bus_, replayOverlaySyncHandler, &window_handle_, nullptr);
        position_slider_->setValue(0);
        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            status_->setText(QStringLiteral("无法启动回放"));
            stopPipeline();
            return;
        }
        status_->setText(QStringLiteral("正在播放：%1")
                             .arg(QFileInfo(files_.at(index)).fileName()));
    }

    void stopPipeline()
    {
        if (!pipeline_) return;
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        if (bus_) {
            gst_object_unref(bus_);
            bus_ = nullptr;
        }
    }

    void updatePosition()
    {
        if (bus_) {
            GstMessage *message = gst_bus_pop_filtered(
                bus_, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
            if (message) {
                if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
                    GError *error = nullptr;
                    gchar *debug = nullptr;
                    gst_message_parse_error(message, &error, &debug);
                    status_->setText(error ? QString::fromUtf8(error->message)
                                           : QStringLiteral("回放失败"));
                    if (error) g_error_free(error);
                    g_free(debug);
                } else {
                    status_->setText(QStringLiteral("回放结束"));
                }
                gst_message_unref(message);
            }
        }
        if (!pipeline_ || position_slider_->isSliderDown()) return;
        gint64 position = GST_CLOCK_TIME_NONE;
        gint64 duration = GST_CLOCK_TIME_NONE;
        if (gst_element_query_position(pipeline_, GST_FORMAT_TIME, &position) &&
            gst_element_query_duration(pipeline_, GST_FORMAT_TIME, &duration) &&
            duration > 0) {
            position_slider_->setValue(static_cast<int>(position * 1000 / duration));
        }
    }

    QStringList files_;
    QListWidget *list_{nullptr};
    QWidget *surface_{nullptr};
    QLabel *status_{nullptr};
    QPushButton *play_button_{nullptr};
    QPushButton *pause_button_{nullptr};
    QPushButton *resume_button_{nullptr};
    QSlider *position_slider_{nullptr};
    QTimer *poll_timer_{nullptr};
    GstElement *pipeline_{nullptr};
    GstBus *bus_{nullptr};
    quintptr window_handle_{0};
};
}  // namespace

VideoReplayController::VideoReplayController(const QString &rtsp_url,
                                             QWidget *dialog_parent,
                                             QObject *parent)
    : QObject(parent),
      rtsp_url_(rtsp_url),
      recordings_dir_(QStringLiteral("/home/orangepi/recordings")),
      dialog_parent_(dialog_parent)
{
    gst_init(nullptr, nullptr);
    if (rtsp_url_.isEmpty())
        rtsp_url_ = QStringLiteral("rtsp://192.168.3.114:8554/d435i");
    QDir().mkpath(recordings_dir_);
    refreshRecordings();
}

VideoReplayController::~VideoReplayController()
{
    stopRecording();
    if (replay_dialog_) delete replay_dialog_;
}

void VideoReplayController::setArmed(bool armed)
{
    if (armed) {
        disarm_stable_count_ = 0;
        if (armed_) return;
        armed_ = true;
        startRecording();
        return;
    }
    if (!armed_) return;
    ++disarm_stable_count_;
    if (disarm_stable_count_ < 15) return;
    disarm_stable_count_ = 0;
    armed_ = false;
    stopRecording();
}

void VideoReplayController::showReplayDialog()
{
    refreshRecordings();
    if (replay_dialog_) {
        auto *dialog = static_cast<ReplayDialog *>(replay_dialog_);
        dialog->raise();
        dialog->activateWindow();
        return;
    }
    replay_dialog_ = new ReplayDialog(recordings_, dialog_parent_);
    replay_dialog_->setAttribute(Qt::WA_DeleteOnClose, true);
    QObject::connect(replay_dialog_, &QObject::destroyed, this,
                     [this]() { replay_dialog_ = nullptr; });
    replay_dialog_->show();
}

void VideoReplayController::startRecording()
{
    if (recording_.exchange(true)) return;
    if (recording_thread_.joinable()) recording_thread_.join();
    cleanupRecordings(4);
    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
    active_file_path_ = QDir(recordings_dir_).filePath(stamp + QStringLiteral("_回放.mkv"));
    stop_requested_ = false;
    recording_thread_ = std::thread(&VideoReplayController::recordingLoop,
                                    this, active_file_path_);
    emit recordingStateChanged(QStringLiteral("录像已开始"));
}

void VideoReplayController::stopRecording()
{
    const bool was_recording = recording_.exchange(false);
    stop_requested_ = true;
    if (recording_thread_.joinable()) recording_thread_.join();
    if (!was_recording) return;
    active_file_path_.clear();
    refreshRecordings();
    emit recordingStateChanged(QStringLiteral("录像已保存"));
}

void VideoReplayController::recordingLoop(QString file_path)
{
    const QString pipeline_description = QStringLiteral(
        "rtspsrc location=%1 protocols=udp latency=100 drop-on-latency=true "
        "! rtph264depay ! h264parse config-interval=-1 ! matroskamux "
        "! filesink location=%2 sync=false")
        .arg(rtsp_url_, file_path);
    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(
        pipeline_description.toUtf8().constData(), &error);
    if (!pipeline) {
        emit recordingStateChanged(error ? QString::fromUtf8(error->message)
                                         : QStringLiteral("录像管线创建失败"));
        if (error) g_error_free(error);
        recording_ = false;
        return;
    }
    GstBus *bus = gst_element_get_bus(pipeline);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    while (!stop_requested_) {
        GstMessage *message = gst_bus_timed_pop_filtered(
            bus, 200 * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (!message) continue;
        if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR ||
            GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS) {
            gst_message_unref(message);
            break;
        }
        gst_message_unref(message);
    }
    if (stop_requested_) {
        gst_element_send_event(pipeline, gst_event_new_eos());
        GstMessage *final_message = gst_bus_timed_pop_filtered(
            bus, 5 * GST_SECOND,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (final_message) gst_message_unref(final_message);
    }
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);
}

void VideoReplayController::refreshRecordings()
{
    cleanupRecordings(5);
    QDir dir(recordings_dir_);
    QFileInfoList files = dir.entryInfoList(QStringList() << QStringLiteral("*_回放.mkv"),
                                            QDir::Files, QDir::Time);
    recordings_.clear();
    for (const QFileInfo &file : files) {
        if (file.absoluteFilePath() != active_file_path_)
            recordings_.append(file.absoluteFilePath());
    }
}

void VideoReplayController::cleanupRecordings(int keep_count)
{
    QDir dir(recordings_dir_);
    const QFileInfoList files = dir.entryInfoList(QStringList() << QStringLiteral("*_回放.mkv"),
                                                  QDir::Files, QDir::Time);
    for (int i = keep_count; i < files.size(); ++i) QFile::remove(files.at(i).absoluteFilePath());
}
