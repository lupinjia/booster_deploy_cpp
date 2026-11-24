// Depth image recorder for Booster robots.
//
// Subscribes to the depth image DDS topic and saves frames at a fixed rate
// (default 16.6 Hz) as 8-bit grayscale PNG preview images plus a CSV index.
//
// Real robot (booster_camera service, verified on K1):
//   topic    rt/boostercamera/head/depth
//   encoding mono16 (uint16 millimeters, camera catalog depth_scale = 0.001)
//   size     544x448, ~20 Hz
//
// Simulation (booster_mujoco): topic rt/depthimage, encoding mono8
// (already normalized to [0,255]). Use --sim to select this automatically.
//
// Real-robot mode (default) initializes with
//   ChannelFactory::InitWithConfigPath(0, fastdds_profile.xml)
// Simulation mode initializes with
//   ChannelFactory::Init(1, "lo")
//
// Supported encodings:
//   mono8 / 8UC1   - one byte per pixel, written directly (no scaling)
//   16UC1 / mono16 - 16-bit depth values, scaled to meters by --depth-scale
//   32FC1          - 32-bit float depth in meters
//
// For metric encodings (16UC1 / mono16 / 32FC1) a preview is produced by
// clipping to [--min-depth, --max-depth] with a fixed range so consecutive
// frames are visually comparable. With --save-raw the untouched 16-bit depth
// samples are also stored as a 16-bit PNG.
//
// Usage:
//   ./depth_recorder                      # real robot (default)
//   ./depth_recorder --real               # real robot (explicit)
//   ./depth_recorder --sim                # booster_mujoco: rt/depthimage, domain 1, lo
//   ./depth_recorder -h                   # full option list

#include <atomic>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <zlib.h>

#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/sensor_msgs/Image.h>

namespace fs = std::filesystem;
using booster::robot::ChannelFactory;
using booster::robot::ChannelSubscriber;
using sensor_msgs::msg::Image;

namespace {

constexpr double kDefaultRateHz = 16.6;

// FastDDS profile used by the on-robot booster services (including
// booster_camera). Real-robot mode always initializes with this profile;
// simulation mode uses ChannelFactory::Init(1, "lo") instead.
constexpr const char *kRobotDdsProfile =
    "/opt/booster/config/dds_configs/single-board_configs/fastdds_profile.xml";

std::atomic<bool> g_stop{false};

void HandleSignal(int) { g_stop = true; }

// ---------------------------------------------------------------------------
// Minimal PNG writer (grayscale, 8 or 16 bit) using zlib.
// ---------------------------------------------------------------------------
void AppendU32Be(std::vector<uint8_t> &out, uint32_t v)
{
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xff));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xff));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
    out.push_back(static_cast<uint8_t>(v & 0xff));
}

void AppendChunk(std::vector<uint8_t> &out, const char type[4],
                 const std::vector<uint8_t> &payload)
{
    AppendU32Be(out, static_cast<uint32_t>(payload.size()));
    const size_t crc_start = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), payload.begin(), payload.end());
    uLong crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, out.data() + crc_start, static_cast<uInt>(out.size() - crc_start));
    AppendU32Be(out, static_cast<uint32_t>(crc));
}

// rows: raw samples with no per-row filter byte. For 8-bit each row is
// `width` bytes; for 16-bit each row is `width` big-endian samples.
bool WritePngGray(const std::string &path, uint32_t width, uint32_t height,
                  int bit_depth, const std::vector<uint8_t> &scanlines)
{
    const size_t bytes_per_sample = static_cast<size_t>(bit_depth / 8);
    const size_t row_bytes = static_cast<size_t>(width) * bytes_per_sample;

    // Add the mandatory filter byte (0 = None) to every scanline.
    std::vector<uint8_t> raw;
    raw.reserve((row_bytes + 1) * height);
    for (uint32_t y = 0; y < height; ++y) {
        raw.push_back(0);
        raw.insert(raw.end(), scanlines.begin() + static_cast<size_t>(y) * row_bytes,
                   scanlines.begin() + static_cast<size_t>(y + 1) * row_bytes);
    }

    uLongf compressed_size = compressBound(static_cast<uLong>(raw.size()));
    std::vector<uint8_t> compressed(compressed_size);
    if (compress2(compressed.data(), &compressed_size, raw.data(),
                  static_cast<uLong>(raw.size()), 6) != Z_OK) {
        return false;
    }
    compressed.resize(compressed_size);

    std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};

    std::vector<uint8_t> ihdr;
    AppendU32Be(ihdr, width);
    AppendU32Be(ihdr, height);
    ihdr.push_back(static_cast<uint8_t>(bit_depth)); // bit depth
    ihdr.push_back(0);                               // color type: grayscale
    ihdr.push_back(0);                               // compression
    ihdr.push_back(0);                               // filter
    ihdr.push_back(0);                               // interlace
    AppendChunk(png, "IHDR", ihdr);
    AppendChunk(png, "IDAT", compressed);
    AppendChunk(png, "IEND", {});

    std::ofstream file(path, std::ios::binary);
    if (!file) {
        return false;
    }
    file.write(reinterpret_cast<const char *>(png.data()),
               static_cast<std::streamsize>(png.size()));
    return file.good();
}

// ---------------------------------------------------------------------------
// Configuration and frame state
// ---------------------------------------------------------------------------
struct Config {
    // Real robot (booster_camera service) publishes at rt/boostercamera/head/depth
    // as mono16 (uint16 millimeters, depth_scale = 0.001).
    std::string topic = "rt/boostercamera/head/depth";
    std::string out_dir = "depth_frames";
    std::string profile = kRobotDdsProfile;
    double rate_hz = kDefaultRateHz;
    uint64_t max_frames = 0; // 0 = unlimited
    double depth_scale = 0.001; // 16UC1/mono16 units -> meters (millimeters)
    double min_depth = 0.1;  // meters
    double max_depth = 3.0;  // meters
    bool save_raw = false;
    bool reliable = false;
    bool topic_explicit = false;
    bool sim = false; // false = real robot (default)
};

struct Frame {
    bool valid = false;
    uint64_t seq = 0;
    std::string encoding;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t step = 0;
    bool bigendian = false;
    int32_t stamp_sec = 0;
    uint32_t stamp_nsec = 0;
    int64_t recv_unix_ns = 0;
    std::vector<uint8_t> data;
};

int64_t NowUnixNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// ---------------------------------------------------------------------------
// Conversion helpers
// ---------------------------------------------------------------------------
inline uint16_t ReadU16(const uint8_t *p, bool big_endian)
{
    if (big_endian) {
        return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(p[1]) << 8) | p[0]);
}

inline float ReadF32(const uint8_t *p, bool big_endian)
{
    uint32_t bits;
    if (big_endian) {
        bits = (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
               (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
    } else {
        bits = (static_cast<uint32_t>(p[3]) << 24) | (static_cast<uint32_t>(p[2]) << 16) |
               (static_cast<uint32_t>(p[1]) << 8) | static_cast<uint32_t>(p[0]);
    }
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline uint8_t NormalizeToU8(double meters, double min_depth, double max_depth)
{
    if (!std::isfinite(meters) || max_depth <= min_depth) {
        return 0;
    }
    const double normalized = (meters - min_depth) / (max_depth - min_depth);
    const double clamped = std::max(0.0, std::min(1.0, normalized));
    return static_cast<uint8_t>(clamped * 255.0 + 0.5);
}

std::string SanitizeEncoding(const std::string &encoding)
{
    std::string out;
    for (char c : encoding) {
        out.push_back((std::isalnum(static_cast<unsigned char>(c)) || c == '_') ? c : '_');
    }
    return out.empty() ? "unknown" : out;
}

// ---------------------------------------------------------------------------
// Recorder
// ---------------------------------------------------------------------------
class DepthRecorder {
public:
    explicit DepthRecorder(Config config) : config_(std::move(config)) {}

    void OnImage(const void *msg)
    {
        const auto *image = static_cast<const Image *>(msg);
        if (image == nullptr || image->height() == 0 || image->width() == 0) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        Frame &frame = latest_;
        frame.valid = true;
        frame.seq = ++received_seq_;
        frame.encoding = image->encoding();
        frame.width = image->width();
        frame.height = image->height();
        frame.step = image->step() != 0 ? image->step() : image->width();
        frame.bigendian = image->is_bigendian() != 0;
        frame.stamp_sec = image->header().stamp().sec();
        frame.stamp_nsec = image->header().stamp().nanosec();
        frame.recv_unix_ns = NowUnixNs();

        const auto &src = image->data();
        // Keep only the bytes that can actually be addressed by width/height/step.
        const size_t max_needed =
            static_cast<size_t>(frame.step) * (frame.height - 1) +
            static_cast<size_t>(frame.width) * BytesPerPixel(frame.encoding);
        const size_t copy_size = std::min(src.size(), max_needed);
        frame.data.assign(src.begin(), src.begin() + static_cast<std::ptrdiff_t>(copy_size));
    }

    void Run()
    {
        fs::create_directories(config_.out_dir);
        csv_path_ = (fs::path(config_.out_dir) / "frames.csv").string();
        csv_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_) {
            std::cerr << "[depth_recorder] failed to open " << csv_path_ << std::endl;
            return;
        }
        csv_ << "index,file,stamp_sec,stamp_nsec,recv_unix_ns,width,height,step,"
                "encoding,min_m,max_m,mean_m,valid_pixels\n";

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / config_.rate_hz));
        auto next_tick = std::chrono::steady_clock::now();
        auto last_rx = std::chrono::steady_clock::now();
        uint64_t saved = 0;
        uint64_t last_saved_seq = 0;

        std::cout << "[depth_recorder] saving at " << config_.rate_hz
                  << " Hz to " << fs::absolute(config_.out_dir).string() << std::endl;

        while (!g_stop) {
            const auto now = std::chrono::steady_clock::now();
            if (next_tick < now) {
                next_tick = now; // do not burst-catch up after a slow frame
            }
            next_tick += period;
            std::this_thread::sleep_until(next_tick);

            Frame frame;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!latest_.valid || latest_.seq == last_saved_seq) {
                    if (std::chrono::steady_clock::now() - last_rx > std::chrono::seconds(3) &&
                        received_seq_ == 0) {
                        last_rx = std::chrono::steady_clock::now();
                        std::cout << "[depth_recorder] waiting for messages on topic '"
                                  << config_.topic << "' ..." << std::endl;
                    }
                    continue;
                }
                frame = latest_;
                last_saved_seq = latest_.seq;
            }

            if (SaveFrame(frame, saved + 1)) {
                ++saved;
                if (saved % 10 == 0 || saved == 1) {
                    std::cout << "[depth_recorder] saved " << saved << " frame(s)" << std::endl;
                }
            }

            if (config_.max_frames != 0 && saved >= config_.max_frames) {
                g_stop = true;
                break;
            }
        }

        if (csv_.is_open()) {
            csv_.flush();
        }
        std::cout << "[depth_recorder] stopped after " << saved << " frame(s)" << std::endl;
    }

private:
    static size_t BytesPerPixel(const std::string &encoding)
    {
        if (encoding == "mono8" || encoding == "8UC1") {
            return 1;
        }
        if (encoding == "16UC1" || encoding == "mono16") {
            return 2;
        }
        if (encoding == "32FC1") {
            return 4;
        }
        return 1;
    }

    bool SaveFrame(const Frame &frame, uint64_t index)
    {
        const std::string encoding = frame.encoding;
        const size_t bpp = BytesPerPixel(encoding);
        if (encoding != "mono8" && encoding != "8UC1" && encoding != "16UC1" &&
            encoding != "mono16" && encoding != "32FC1") {
            std::cerr << "[depth_recorder] unsupported encoding '" << encoding
                      << "', skipping frame" << std::endl;
            return false;
        }

        const uint32_t width = frame.width;
        const uint32_t height = frame.height;
        const size_t row_bytes_in =
            static_cast<size_t>(frame.step != 0 ? frame.step : width * bpp);
        const size_t needed = row_bytes_in * (height - 1) + static_cast<size_t>(width) * bpp;
        if (frame.data.size() < needed) {
            std::cerr << "[depth_recorder] short image buffer (" << frame.data.size()
                      << " < " << needed << "), skipping frame" << std::endl;
            return false;
        }

        if (!printed_encoding_) {
            printed_encoding_ = true;
            std::cout << "[depth_recorder] first frame: encoding='" << encoding
                      << "', " << width << "x" << height << ", step=" << frame.step
                      << ", bigendian=" << (frame.bigendian ? 1 : 0);
            if (bpp == 1) {
                std::cout << " -> raw pixels are uint8 [0,255] (no metric scaling applied)";
            } else if (bpp == 2) {
                std::cout << " -> raw pixels are uint16 (scale " << config_.depth_scale
                          << " to meters)";
            } else {
                std::cout << " -> raw pixels are float32 (meters)";
            }
            std::cout << std::endl;
        }

        std::vector<uint8_t> preview(static_cast<size_t>(width) * height, 0);
        std::vector<uint8_t> raw16;
        if (config_.save_raw && (encoding == "16UC1" || encoding == "mono16")) {
            raw16.resize(static_cast<size_t>(width) * height * 2);
        }

        double min_m = std::numeric_limits<double>::infinity();
        double max_m = 0.0;
        double sum_m = 0.0;
        uint64_t valid = 0;

        for (uint32_t y = 0; y < height; ++y) {
            const uint8_t *row = frame.data.data() + static_cast<size_t>(y) * row_bytes_in;
            for (uint32_t x = 0; x < width; ++x) {
                uint8_t &out = preview[static_cast<size_t>(y) * width + x];
                if (bpp == 1) {
                    out = row[x];
                } else if (bpp == 2) {
                    const uint16_t value = ReadU16(row + static_cast<size_t>(x) * 2,
                                                    frame.bigendian);
                    if (config_.save_raw) {
                        const size_t o = (static_cast<size_t>(y) * width + x) * 2;
                        raw16[o] = static_cast<uint8_t>(value & 0xff);
                        raw16[o + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
                        std::swap(raw16[o], raw16[o + 1]); // PNG stores big-endian
                    }
                    const double meters = static_cast<double>(value) * config_.depth_scale;
                    if (value != 0 && std::isfinite(meters)) {
                        out = NormalizeToU8(meters, config_.min_depth, config_.max_depth);
                        if (meters < min_m) min_m = meters;
                        if (meters > max_m) max_m = meters;
                        sum_m += meters;
                        ++valid;
                    }
                } else {
                    const float meters = ReadF32(row + static_cast<size_t>(x) * 4,
                                                 frame.bigendian);
                    if (std::isfinite(meters) && meters > 0.0f) {
                        out = NormalizeToU8(meters, config_.min_depth, config_.max_depth);
                        if (meters < min_m) min_m = meters;
                        if (meters > max_m) max_m = meters;
                        sum_m += meters;
                        ++valid;
                    }
                }
            }
        }

        if (valid == 0) {
            min_m = 0.0;
        }

        std::ostringstream stem;
        stem << "frame_" << std::setw(6) << std::setfill('0') << index << "_"
             << SanitizeEncoding(encoding);
        const fs::path base = fs::path(config_.out_dir) / stem.str();

        const std::string preview_path = base.string() + ".png";
        if (!WritePngGray(preview_path, width, height, 8, preview)) {
            std::cerr << "[depth_recorder] failed to write " << preview_path << std::endl;
            return false;
        }

        if (config_.save_raw && !raw16.empty()) {
            const std::string raw_path = base.string() + "_raw.png";
            WritePngGray(raw_path, width, height, 16, raw16);
        }

        const double mean_m = valid > 0 ? sum_m / static_cast<double>(valid) : 0.0;
        csv_ << index << "," << (stem.str() + ".png") << "," << frame.stamp_sec << ","
             << frame.stamp_nsec << "," << frame.recv_unix_ns << "," << width << ","
             << height << "," << frame.step << "," << encoding << "," << std::fixed
             << std::setprecision(4) << min_m << "," << max_m << "," << mean_m << ","
             << valid << "\n";
        csv_.flush();
        return true;
    }

    Config config_;
    std::mutex mutex_;
    Frame latest_;
    uint64_t received_seq_ = 0;
    bool printed_encoding_ = false;
    std::string csv_path_;
    std::ofstream csv_;
};

void PrintUsage(const char *argv0)
{
    std::cout
        << "Usage: " << argv0 << " [options]\n\n"
        << "Records depth images (sensor_msgs::msg::Image) at a fixed rate.\n\n"
        << "Options:\n"
        << "  --topic <name>       DDS topic (real robot default: rt/boostercamera/head/depth;\n"
        << "                       with --sim: rt/depthimage)\n"
        << "  --out <dir>          output directory (default: depth_frames)\n"
        << "  --rate <hz>          save rate in Hz (default: 16.6)\n"
        << "  --max-frames <n>     stop after n saved frames (default: 0 = unlimited)\n"
        << "  --real               real robot (default): domain 0 via\n"
        << "                       InitWithConfigPath(0, fastdds_profile.xml)\n"
        << "  --sim                simulation (booster_mujoco): Init(1, \"lo\"),\n"
        << "                       topic defaults to rt/depthimage\n"
        << "  --profile <path>     FastDDS profile for --real (default:\n"
        << "                       /opt/booster/config/dds_configs/\n"
        << "                       single-board_configs/fastdds_profile.xml)\n"
        << "  --depth-scale <s>    16UC1/mono16 unit -> meters (default: 0.001, i.e. mm)\n"
        << "  --min-depth <m>      preview near clip in meters (default: 0.1)\n"
        << "  --max-depth <m>      preview far clip in meters (default: 3.0)\n"
        << "  --save-raw           also save 16UC1/mono16 frames as 16-bit PNG\n"
        << "  --reliable           request reliable DDS delivery (default: best effort)\n"
        << "  -h, --help           show this help\n";
}

bool ParseArgs(int argc, char **argv, Config &config)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char *name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << std::endl;
                return {};
            }
            return argv[++i];
        };

        if (arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            return false;
        } else if (arg == "--topic") {
            config.topic = next("--topic");
            config.topic_explicit = true;
        } else if (arg == "--out") {
            config.out_dir = next("--out");
        } else if (arg == "--rate") {
            config.rate_hz = std::stod(next("--rate"));
        } else if (arg == "--max-frames") {
            config.max_frames = std::stoull(next("--max-frames"));
        } else if (arg == "--real") {
            config.sim = false;
        } else if (arg == "--sim") {
            config.sim = true;
        } else if (arg == "--profile") {
            config.profile = next("--profile");
        } else if (arg == "--depth-scale") {
            config.depth_scale = std::stod(next("--depth-scale"));
        } else if (arg == "--min-depth") {
            config.min_depth = std::stod(next("--min-depth"));
        } else if (arg == "--max-depth") {
            config.max_depth = std::stod(next("--max-depth"));
        } else if (arg == "--save-raw") {
            config.save_raw = true;
        } else if (arg == "--reliable") {
            config.reliable = true;
        } else {
            std::cerr << "unknown option: " << arg << std::endl;
            PrintUsage(argv[0]);
            return false;
        }
    }

    if (config.rate_hz <= 0.0) {
        std::cerr << "--rate must be positive" << std::endl;
        return false;
    }
    if (config.max_depth <= config.min_depth) {
        std::cerr << "--max-depth must be greater than --min-depth" << std::endl;
        return false;
    }
    if (config.sim && !config.topic_explicit) {
        config.topic = "rt/depthimage";
    }
    if (!config.sim) {
        if (config.profile.empty()) {
            config.profile = kRobotDdsProfile;
        }
        if (!fs::exists(config.profile)) {
            std::cerr << "[depth_recorder] real-robot DDS profile not found: "
                      << config.profile
                      << "\n  run with --sim for booster_mujoco, or pass --profile <path>."
                      << std::endl;
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char **argv)
{
    Config config;
    if (!ParseArgs(argc, argv, config)) {
        return 1;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    if (config.sim) {
        ChannelFactory::Instance()->Init(1, "lo");
    } else {
        ChannelFactory::Instance()->InitWithConfigPath(0, config.profile);
    }

    DepthRecorder recorder(config);
    ChannelSubscriber<Image> subscriber(
        config.topic, [&recorder](const void *msg) { recorder.OnImage(msg); },
        config.reliable);
    subscriber.InitChannel();

    std::cout << "[depth_recorder] mode=" << (config.sim ? "sim" : "real")
              << ", subscribed to '" << config.topic << "' (";
    if (config.sim) {
        std::cout << "domain 1, interface lo";
    } else {
        std::cout << "domain 0, profile " << config.profile;
    }
    std::cout << ")" << std::endl;

    recorder.Run();
    subscriber.CloseChannel();
    return 0;
}
