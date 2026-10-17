// orangepi_rtsp_mpp_probe.cpp
//
// Non-Qt RTSP -> Rockchip MPP H.264 hardware-decode probe for OrangePi 5 Ultra.
// Part of the RDK X5 -> OrangePi D435i video-link validation (see
// docs/rdk_x5_orangepi5ultra_send_receive_implementation_plan.md, T4-T7).
//
// Pipeline:
//   rtspsrc -> rtph264depay -> h264parse config-interval=-1 -> mppvideodec
//   -> capsfilter(video/x-raw,format=NV12|RGB|BGR) -> appsink
//
// The probe actively pulls samples with gst_app_sink_try_pull_sample
// (max-buffers=1, drop=true, sync=false), reads caps/stride/format through
// GstVideoInfo/GstVideoFrame (DMABUF-safe), keeps only the newest frame, and
// reports sample / timeout / drop / fps / reconnect / bus-error statistics.
// It never creates Qt controls and never depends on QImage.
//
// Exit codes: 0 normal, 1 usage, 2 pipeline, 3 connection, 4 negotiation,
//             5 decode, 6 no-data timeout.

#include <getopt.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/base/gstbasesink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

namespace
{

constexpr int kExitNormal = 0;
constexpr int kExitUsage = 1;
constexpr int kExitPipeline = 2;
constexpr int kExitConnection = 3;
constexpr int kExitNegotiation = 4;
constexpr int kExitDecode = 5;
constexpr int kExitNoData = 6;

volatile std::sig_atomic_t g_stop_requested = 0;

void handleSignal(int)
{
  g_stop_requested = 1;
}

struct Options
{
  std::string url{"rtsp://192.168.3.114:8554/d435i"};
  std::string protocol{"udp"};
  int latency_ms{50};
  std::string format{"nv12"};
  int duration_sec{30};
  std::string dump_dir;
  int stats_period_sec{5};
  int idle_timeout_ms{15000};
};

struct ProbeStats
{
  std::uint64_t samples{0};
  std::uint64_t timeouts{0};
  std::uint64_t reconnects{0};
  std::uint64_t bus_errors{0};
  std::uint64_t dropped_by_appsink{0};
  std::uint64_t dump_frames{0};
  int width{0};
  int height{0};
  double last_fps{0.0};
  bool header_printed{false};
};

struct Pipeline
{
  GstElement * pipeline{nullptr};
  GstElement * sink{nullptr};
};

void printUsage(const char * program)
{
  std::fprintf(
    stderr,
    "Usage: %s [options]\n"
    "  --url URL              RTSP URL (default: rtsp://192.168.3.114:8554/d435i)\n"
    "  --protocol udp|tcp     RTSP lower transport (default: udp)\n"
    "  --latency-ms MS        rtspsrc latency (default: 50)\n"
    "  --format nv12|rgb|bgr|auto  appsink output format (default: nv12)\n"
    "  --duration-sec SEC     0 = run until SIGINT (default: 30)\n"
    "  --dump-dir DIR         dump one raw frame per stats period (optional)\n"
    "  --stats-period-sec SEC statistics interval (default: 5)\n"
    "  --idle-timeout-ms MS   no-frame threshold before reconnect (default: 15000)\n"
    "  --help                 show this message\n"
    "\n"
    "Exit codes: 0 normal, 1 usage, 2 pipeline, 3 connection, 4 negotiation,\n"
    "            5 decode, 6 no-data timeout\n",
    program);
}

int parsePositive(const char * value, const char * name)
{
  char * end = nullptr;
  errno = 0;
  const long parsed = std::strtol(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || parsed < 0 || parsed > INT32_MAX) {
    throw std::invalid_argument(std::string("invalid ") + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  const option long_options[] = {
    {"url", required_argument, nullptr, 'u'},
    {"protocol", required_argument, nullptr, 'p'},
    {"latency-ms", required_argument, nullptr, 'l'},
    {"format", required_argument, nullptr, 'f'},
    {"duration-sec", required_argument, nullptr, 'd'},
    {"dump-dir", required_argument, nullptr, 'o'},
    {"stats-period-sec", required_argument, nullptr, 's'},
    {"idle-timeout-ms", required_argument, nullptr, 'i'},
    {"help", no_argument, nullptr, '?'},
    {nullptr, 0, nullptr, 0},
  };

  while (true) {
    const int key = getopt_long(argc, argv, "", long_options, nullptr);
    if (key == -1) {
      break;
    }
    switch (key) {
      case 'u': options.url = optarg; break;
      case 'p': options.protocol = optarg; break;
      case 'l': options.latency_ms = parsePositive(optarg, "latency-ms"); break;
      case 'f': options.format = optarg; break;
      case 'd': options.duration_sec = parsePositive(optarg, "duration-sec"); break;
      case 'o': options.dump_dir = optarg; break;
      case 's': options.stats_period_sec = parsePositive(optarg, "stats-period-sec"); break;
      case 'i': options.idle_timeout_ms = parsePositive(optarg, "idle-timeout-ms"); break;
      case '?':
      default:
        printUsage(argv[0]);
        std::exit(key == '?' ? kExitNormal : kExitUsage);
    }
  }

  if (options.protocol != "udp" && options.protocol != "tcp") {
    throw std::invalid_argument("--protocol must be udp or tcp");
  }
  if (options.format != "nv12" && options.format != "rgb" &&
    options.format != "bgr" && options.format != "auto")
  {
    throw std::invalid_argument("--format must be nv12, rgb, bgr or auto");
  }
  if (options.stats_period_sec <= 0) {
    throw std::invalid_argument("--stats-period-sec must be positive");
  }
  return options;
}

// GST_RTSP_LOWER_TRANS_UDP=1, GST_RTSP_LOWER_TRANS_TCP=4.
int protocolMask(const Options & options)
{
  return options.protocol == "tcp" ? 4 : 1;
}

// GStreamer format names are case-sensitive ("NV12", not "nv12").
std::string uppercase(std::string value)
{
  for (char & c : value) {
    if (c >= 'a' && c <= 'z') {
      c = static_cast<char>(c - 'a' + 'A');
    }
  }
  return value;
}

void onPadAdded(GstElement *, GstPad * pad, gpointer user_data)
{
  GstElement * depay = static_cast<GstElement *>(user_data);
  GstPad * sink_pad = gst_element_get_static_pad(depay, "sink");
  if (sink_pad == nullptr) {
    return;
  }
  if (!gst_pad_is_linked(sink_pad)) {
    const GstPadLinkReturn ret = gst_pad_link(pad, sink_pad);
    if (GST_PAD_LINK_FAILED(ret)) {
      std::fprintf(
        stderr, "[probe] pad link to rtph264depay failed: %d\n",
        static_cast<int>(ret));
    }
  }
  gst_object_unref(sink_pad);
}

Pipeline buildPipeline(const Options & options, std::string & error)
{
  Pipeline pipeline;
  pipeline.pipeline = gst_pipeline_new("rtsp-mpp-probe");
  if (pipeline.pipeline == nullptr) {
    error = "gst_pipeline_new failed";
    return pipeline;
  }

  const char * element_names[] = {
    "rtspsrc", "rtph264depay", "h264parse", "mppvideodec", "capsfilter", "appsink"};
  GstElement * elements[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
  for (int i = 0; i < 6; ++i) {
    elements[i] = gst_element_factory_make(element_names[i], element_names[i]);
    if (elements[i] == nullptr) {
      error = std::string("element unavailable: ") + element_names[i];
      for (int j = 0; j < i; ++j) {
        gst_object_unref(elements[j]);
      }
      gst_object_unref(pipeline.pipeline);
      pipeline.pipeline = nullptr;
      return pipeline;
    }
  }

  GstElement * rtspsrc = elements[0];
  GstElement * depay = elements[1];
  GstElement * parse = elements[2];
  GstElement * decoder = elements[3];
  GstElement * capsfilter = elements[4];
  GstElement * appsink = elements[5];

  g_object_set(
    rtspsrc,
    "location", options.url.c_str(),
    "protocols", protocolMask(options),
    "latency", static_cast<gint64>(options.latency_ms),
    "drop-on-latency", TRUE,
    nullptr);

  g_object_set(parse, "config-interval", -1, nullptr);

  if (options.format != "auto") {
    const std::string caps_format = uppercase(options.format);
    GstCaps * caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, caps_format.c_str(), nullptr);
    g_object_set(capsfilter, "caps", caps, nullptr);
    gst_caps_unref(caps);
  }

  g_object_set(
    appsink,
    "max-buffers", 1,
    "drop", TRUE,
    "sync", FALSE,
    "emit-signals", FALSE,
    nullptr);

  gst_bin_add_many(
    GST_BIN(pipeline.pipeline),
    rtspsrc, depay, parse, decoder, capsfilter, appsink, nullptr);

  if (!gst_element_link_many(depay, parse, decoder, capsfilter, appsink, nullptr)) {
    error = "failed to link depay->parse->decoder->capsfilter->appsink";
    gst_object_unref(pipeline.pipeline);
    pipeline.pipeline = nullptr;
    return pipeline;
  }

  // rtspsrc pads appear dynamically once the session is established.
  g_signal_connect(rtspsrc, "pad-added", G_CALLBACK(onPadAdded), depay);

  pipeline.sink = appsink;
  return pipeline;
}

void teardownPipeline(Pipeline & pipeline)
{
  if (pipeline.pipeline != nullptr) {
    gst_element_set_state(pipeline.pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline.pipeline);
    pipeline.pipeline = nullptr;
    pipeline.sink = nullptr;
  }
}

bool startPipeline(Pipeline & pipeline, GstBus ** bus, const Options & options)
{
  std::string error;
  pipeline = buildPipeline(options, error);
  if (pipeline.pipeline == nullptr) {
    std::fprintf(stderr, "[probe] pipeline build failed: %s\n", error.c_str());
    return false;
  }
  *bus = gst_element_get_bus(pipeline.pipeline);
  if (gst_element_set_state(pipeline.pipeline, GST_STATE_PLAYING) ==
    GST_STATE_CHANGE_FAILURE)
  {
    std::fprintf(stderr, "[probe] failed to set pipeline to PLAYING\n");
    teardownPipeline(pipeline);
    return false;
  }
  return true;
}

void dumpFrame(
  GstSample * sample,
  const GstVideoInfo & info,
  const Options & options,
  ProbeStats & stats)
{
  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &info, gst_sample_get_buffer(sample), GST_MAP_READ)) {
    std::fprintf(
      stderr, "[probe] frame %llu not CPU-mappable (DMABUF?); dump skipped\n",
      static_cast<unsigned long long>(stats.samples));
    return;
  }

  char path[512];
  std::snprintf(
    path, sizeof(path), "%s/frame_%010llu.raw",
    options.dump_dir.c_str(), static_cast<unsigned long long>(stats.samples));
  FILE * file = std::fopen(path, "wb");
  if (file == nullptr) {
    std::fprintf(stderr, "[probe] cannot open dump %s: %s\n", path, std::strerror(errno));
    gst_video_frame_unmap(&frame);
    return;
  }

  const std::size_t frame_size = GST_VIDEO_INFO_SIZE(&info);
  if (frame_size > 0 && frame.data[0] != nullptr) {
    std::fwrite(frame.data[0], 1, frame_size, file);
  }

  std::fclose(file);
  ++stats.dump_frames;
  gst_video_frame_unmap(&frame);
}

void printStats(const ProbeStats & stats)
{
  std::fprintf(
    stderr,
    "[probe] stats: samples=%llu fps=%.2f timeouts=%llu dropped=%llu "
    "reconnects=%llu bus_errors=%llu dumps=%llu %dx%d\n",
    static_cast<unsigned long long>(stats.samples),
    stats.last_fps,
    static_cast<unsigned long long>(stats.timeouts),
    static_cast<unsigned long long>(stats.dropped_by_appsink),
    static_cast<unsigned long long>(stats.reconnects),
    static_cast<unsigned long long>(stats.bus_errors),
    static_cast<unsigned long long>(stats.dump_frames),
    stats.width,
    stats.height);
}

void updateDroppedCount(GstElement * appsink, ProbeStats & stats)
{
  if (appsink == nullptr) {
    return;
  }
  GstStructure * sink_stats = gst_base_sink_get_stats(GST_BASE_SINK(appsink));
  if (sink_stats != nullptr) {
    std::uint64_t dropped = 0;
    if (gst_structure_get_uint64(sink_stats, "dropped", &dropped)) {
      stats.dropped_by_appsink = dropped;
    }
    gst_structure_free(sink_stats);
  }
}

int runProbe(const Options & options)
{
  Pipeline pipeline;
  GstBus * bus = nullptr;
  if (!startPipeline(pipeline, &bus, options)) {
    return kExitPipeline;
  }

  ProbeStats stats;
  const auto start = std::chrono::steady_clock::now();
  auto window_start = start;
  auto last_stats = start;
  auto last_sample = start;
  auto last_dump = start;
  std::uint64_t window_samples = 0;
  bool first_frame_after_reconnect = true;
  bool reconnect_pending = false;

  while (!g_stop_requested) {
    // 1. Drain bus messages.
    GstMessage * message = nullptr;
    while ((message = gst_bus_pop_filtered(
        bus, static_cast<GstMessageType>(
          GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS))) != nullptr)
    {
      const GstMessageType type = GST_MESSAGE_TYPE(message);

      if (type == GST_MESSAGE_EOS) {
        gst_message_unref(message);
        // For a live RTSP source, EOS usually means the server closed the
        // stream (T7 restart scenario). Treat it as a reconnect trigger so the
        // probe stays up instead of exiting; the run is bounded by duration.
        std::fprintf(stderr, "[probe] EOS received; will reconnect\n");
        reconnect_pending = true;
        continue;
      }

      if (type == GST_MESSAGE_ERROR) {
        GError * error = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        const char * source = GST_OBJECT_NAME(message->src);
        ++stats.bus_errors;
        std::fprintf(
          stderr, "[probe] BUS ERROR from %s: %s\n",
          source == nullptr ? "?" : source, error == nullptr ? "?" : error->message);
        if (debug != nullptr) {
          std::fprintf(stderr, "[probe]   debug: %s\n", debug);
        }

        const bool not_negotiated =
          error != nullptr &&
          std::strstr(error->message, "not-negotiated") != nullptr;
        const bool decode_element =
          source != nullptr &&
          (std::strcmp(source, "mppvideodec") == 0 ||
            std::strcmp(source, "rtph264depay") == 0 ||
            std::strcmp(source, "h264parse") == 0 ||
            std::strcmp(source, "capsfilter") == 0);
        const bool from_rtspsrc = source != nullptr && std::strcmp(source, "rtspsrc") == 0;

        if (error != nullptr) {
          g_error_free(error);
        }
        if (debug != nullptr) {
          g_free(debug);
        }
        gst_message_unref(message);

        if (not_negotiated) {
          teardownPipeline(pipeline);
          gst_object_unref(bus);
          std::fprintf(stderr, "[probe] negotiation failed; exiting\n");
          return kExitNegotiation;
        }
        if (decode_element) {
          teardownPipeline(pipeline);
          gst_object_unref(bus);
          std::fprintf(stderr, "[probe] decode error; exiting\n");
          return kExitDecode;
        }
        if (from_rtspsrc) {
          std::fprintf(stderr, "[probe] RTSP transport error; will reconnect\n");
          reconnect_pending = true;
        } else {
          teardownPipeline(pipeline);
          gst_object_unref(bus);
          std::fprintf(stderr, "[probe] unexpected connection error; exiting\n");
          return kExitConnection;
        }
        continue;
      }

      // WARNING
      gchar * debug = nullptr;
      gst_message_parse_warning(message, nullptr, &debug);
      std::fprintf(
        stderr, "[probe] WARNING: %s\n",
        debug == nullptr ? "(no debug text)" : debug);
      if (debug != nullptr) {
        g_free(debug);
      }
      gst_message_unref(message);
    }

    // 2. Reconnect on transport error or idle timeout.
    if (reconnect_pending) {
      reconnect_pending = false;
      ++stats.reconnects;
      std::fprintf(
        stderr, "[probe] reconnecting (%llu)...\n",
        static_cast<unsigned long long>(stats.reconnects));
      teardownPipeline(pipeline);
      gst_object_unref(bus);
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (!startPipeline(pipeline, &bus, options)) {
        return kExitPipeline;
      }
      last_sample = std::chrono::steady_clock::now();
      first_frame_after_reconnect = true;
      continue;
    }

    // 3. Pull the newest frame.
    GstSample * sample = gst_app_sink_try_pull_sample(
      GST_APP_SINK(pipeline.sink), 20 * GST_MSECOND);
    const auto now = std::chrono::steady_clock::now();

    if (sample != nullptr) {
      ++stats.samples;
      ++window_samples;
      last_sample = now;

      const GstCaps * caps = gst_sample_get_caps(sample);
      if (caps != nullptr) {
        GstVideoInfo info;
        if (gst_video_info_from_caps(&info, caps)) {
          stats.width = GST_VIDEO_INFO_WIDTH(&info);
          stats.height = GST_VIDEO_INFO_HEIGHT(&info);

          if (!stats.header_printed) {
            const GstCapsFeatures * features = gst_caps_get_features(caps, 0);
            const char * memory = "system";
            if (features != nullptr &&
              gst_caps_features_contains(features, "memory:DMABuf"))
            {
              memory = "DMA";
            }
            std::fprintf(
              stderr,
              "[probe] negotiated: %dx%d format=%s memory=%s framerate=%d/%d\n",
              stats.width,
              stats.height,
              gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&info)),
              memory,
              GST_VIDEO_INFO_FPS_N(&info),
              GST_VIDEO_INFO_FPS_D(&info));
            stats.header_printed = true;
          }

          if (!options.dump_dir.empty()) {
            const double since_dump =
              std::chrono::duration<double>(now - last_dump).count();
            if (since_dump >= options.stats_period_sec) {
              dumpFrame(sample, info, options, stats);
              last_dump = now;
            }
          }
        }
      }

      if (first_frame_after_reconnect) {
        std::fprintf(
          stderr, "[probe] got first frame after (re)connect, seq=%llu\n",
          static_cast<unsigned long long>(stats.samples));
        first_frame_after_reconnect = false;
      }

      gst_sample_unref(sample);
    } else {
      ++stats.timeouts;
    }

    // 4. Idle fallback: no frames for a long time, even without a bus error.
    if (options.idle_timeout_ms > 0) {
      const double idle_ms =
        std::chrono::duration<double, std::milli>(now - last_sample).count();
      if (idle_ms > options.idle_timeout_ms) {
        std::fprintf(
          stderr, "[probe] no frames for %.1fs; treating link as lost\n",
          idle_ms / 1000.0);
        reconnect_pending = true;
      }
    }

    // 5. Periodic statistics.
    const double since_stats =
      std::chrono::duration<double>(now - last_stats).count();
    if (since_stats >= options.stats_period_sec) {
      const double since_window =
        std::chrono::duration<double>(now - window_start).count();
      stats.last_fps =
        since_window > 0.0 ? static_cast<double>(window_samples) / since_window : 0.0;
      updateDroppedCount(pipeline.sink, stats);
      printStats(stats);
      window_start = now;
      window_samples = 0;
      last_stats = now;
    }

    // 6. Duration reached?
    if (options.duration_sec > 0) {
      const double elapsed =
        std::chrono::duration<double>(now - start).count();
      if (elapsed >= options.duration_sec) {
        break;
      }
    }
  }

  const double since_window =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - window_start).count();
  stats.last_fps =
    since_window > 0.0 ? static_cast<double>(window_samples) / since_window : 0.0;
  updateDroppedCount(pipeline.sink, stats);
  printStats(stats);

  teardownPipeline(pipeline);
  gst_object_unref(bus);
  return kExitNormal;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  try {
    options = parseOptions(argc, argv);
  } catch (const std::invalid_argument & error) {
    std::fprintf(stderr, "argument error: %s\n", error.what());
    printUsage(argv[0]);
    return kExitUsage;
  }

  gst_init(&argc, &argv);

  const char * required_elements[] = {
    "rtspsrc", "rtph264depay", "h264parse", "mppvideodec", "appsink"};
  for (const char * name : required_elements) {
    if (gst_element_factory_find(name) == nullptr) {
      std::fprintf(stderr, "[probe] required element not installed: %s\n", name);
      return kExitPipeline;
    }
  }

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  if (!options.dump_dir.empty()) {
    if (mkdir(options.dump_dir.c_str(), 0755) != 0 && errno != EEXIST) {
      std::fprintf(
        stderr, "[probe] cannot create dump dir %s: %s\n",
        options.dump_dir.c_str(), std::strerror(errno));
      return kExitUsage;
    }
  }

  std::fprintf(
    stderr,
    "[probe] url=%s protocol=%s latency=%dms format=%s duration=%ds "
    "dump_dir=%s stats_period=%ds\n",
    options.url.c_str(),
    options.protocol.c_str(),
    options.latency_ms,
    options.format.c_str(),
    options.duration_sec,
    options.dump_dir.empty() ? "(none)" : options.dump_dir.c_str(),
    options.stats_period_sec);

  return runProbe(options);
}
