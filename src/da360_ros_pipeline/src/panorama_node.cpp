#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

namespace {

static enum AVPixelFormat get_hw_format(
    AVCodecContext *ctx, const enum AVPixelFormat *pixel_formats)
{
    (void)ctx;
    for (const enum AVPixelFormat *format = pixel_formats;
         *format != AV_PIX_FMT_NONE;
         ++format) {
        if (*format == AV_PIX_FMT_CUDA) {
            return *format;
        }
    }
    return AV_PIX_FMT_NONE;
}

}  // namespace

class PanoramaNode : public rclcpp::Node
{
private:
    struct DecodedFrame
    {
        cv::Mat bgr;
        std_msgs::msg::Header header;
    };

    // FFmpeg decoder state.
    const AVCodec *codec_ = nullptr;
    AVCodecContext *codec_ctx_ = nullptr;
    AVCodecParserContext *parser_ctx_ = nullptr;
    AVPacket *packet_ = nullptr;
    AVFrame *hw_frame_ = nullptr;
    AVFrame *sw_frame_ = nullptr;
    SwsContext *sws_ctx_ = nullptr;
    AVBufferRef *hw_device_ctx_ = nullptr;
    enum AVHWDeviceType hw_type_ = AV_HWDEVICE_TYPE_NONE;
    cv::Mat bgr_frame_;

    // ROS interfaces.
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    // Decoded frames stay in-process until they are projected.  No raw ROS
    // image topic is created, which avoids serializing the 3 MB fisheye frame.
    std::thread processing_thread_;
    std::queue<DecodedFrame> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> stop_processing_{false};
    size_t max_queue_size_ = 3;

    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;
    bool use_hardware_decoder_ = false;

    // Equirectangular parameters.
    double cx_offset_ = 0.0;
    double cy_offset_ = 4.0;
    double back_cx_offset_ = 0.0;
    double back_cy_offset_ = 4.0;
    double back_radius_scale_ = 1.0;
    int crop_size_ = 1920;
    int mount_roll_deg_ = 90;
    double tx_ = -0.034;
    double ty_ = -0.004;
    double tz_ = -0.226;
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
    bool gpu_enabled_ = true;
    int out_width_ = 1440;
    int out_height_ = 720;

    // Equirectangular mapping state.
    double cx_ = 0.0;
    double cy_ = 0.0;
    double back_cx_ = 0.0;
    double back_cy_ = 0.0;
    cv::Mat back_to_front_rotation_;
    cv::Vec3d back_to_front_translation_;
    cv::Mat front_map_x_;
    cv::Mat front_map_y_;
    cv::Mat back_map_x_;
    cv::Mat back_map_y_;
    cv::Mat front_mask_;
    cv::Mat back_mask_;
    bool maps_initialized_ = false;
    bool params_changed_ = true;
    int img_height_ = 0;
    int img_width_ = 0;
    std::mutex processing_mutex_;

    void init_ffmpeg_decoder()
    {
        if (!use_hardware_decoder_) {
            codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
            if (!codec_) {
                RCLCPP_ERROR(get_logger(), "No software H.264 decoder available");
                return;
            }
        } else {
            hw_type_ = AV_HWDEVICE_TYPE_CUDA;
            codec_ = avcodec_find_decoder_by_name("h264_cuvid");
            if (!codec_) {
                RCLCPP_WARN(
                    get_logger(),
                    "Hardware decoder not available, falling back to software");
                hw_type_ = AV_HWDEVICE_TYPE_NONE;
                codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
            } else {
                RCLCPP_INFO(get_logger(), "Using hardware H.264 decoder (NVDEC)");
            }

            if (hw_type_ != AV_HWDEVICE_TYPE_NONE && codec_) {
                const int error = av_hwdevice_ctx_create(
                    &hw_device_ctx_, hw_type_, nullptr, nullptr, 0);
                if (error < 0) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Failed to create hardware device context, falling back to software");
                    hw_type_ = AV_HWDEVICE_TYPE_NONE;
                    codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
                }
            }
            if (!codec_) {
                RCLCPP_ERROR(get_logger(), "No H.264 decoder available");
                return;
            }
        }

        parser_ctx_ = av_parser_init(codec_->id);
        codec_ctx_ = avcodec_alloc_context3(codec_);
        if (!parser_ctx_ || !codec_ctx_) {
            cleanup_ffmpeg_decoder();
            return;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE && hw_device_ctx_) {
            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->get_format = get_hw_format;
        }

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open H.264 decoder");
            cleanup_ffmpeg_decoder();
            return;
        }

        packet_ = av_packet_alloc();
        hw_frame_ = av_frame_alloc();
        if (!packet_ || !hw_frame_) {
            cleanup_ffmpeg_decoder();
            return;
        }
        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            sw_frame_ = av_frame_alloc();
            if (!sw_frame_) {
                cleanup_ffmpeg_decoder();
            }
        }
    }

    void cleanup_ffmpeg_decoder()
    {
        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }
        if (sw_frame_) {
            av_frame_free(&sw_frame_);
            sw_frame_ = nullptr;
        }
        if (hw_frame_) {
            av_frame_free(&hw_frame_);
            hw_frame_ = nullptr;
        }
        if (packet_) {
            av_packet_free(&packet_);
            packet_ = nullptr;
        }
        if (codec_ctx_) {
            avcodec_free_context(&codec_ctx_);
            codec_ctx_ = nullptr;
        }
        if (parser_ctx_) {
            av_parser_close(parser_ctx_);
            parser_ctx_ = nullptr;
        }
        if (hw_device_ctx_) {
            av_buffer_unref(&hw_device_ctx_);
            hw_device_ctx_ = nullptr;
        }
        codec_ = nullptr;
    }

    void load_parameters()
    {
        cx_offset_ = get_parameter("cx_offset").as_double();
        cy_offset_ = get_parameter("cy_offset").as_double();
        back_cx_offset_ = get_parameter("back_cx_offset").as_double();
        back_cy_offset_ = get_parameter("back_cy_offset").as_double();
        back_radius_scale_ = get_parameter("back_radius_scale").as_double();
        if (back_radius_scale_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "back_radius_scale must be positive; using 1.0");
            back_radius_scale_ = 1.0;
        }
        crop_size_ = get_parameter("crop_size").as_int();
        mount_roll_deg_ = get_parameter("mount_roll_deg").as_int();
        out_width_ = get_parameter("out_width").as_int();
        out_height_ = get_parameter("out_height").as_int();
        gpu_enabled_ = get_parameter("gpu").as_bool();

        const auto translation = get_parameter("translation").as_double_array();
        if (translation.size() >= 3) {
            tx_ = translation[0];
            ty_ = translation[1];
            tz_ = translation[2];
        }
        const auto rotation_deg = get_parameter("rotation_deg").as_double_array();
        if (rotation_deg.size() >= 3) {
            roll_ = rotation_deg[0] * M_PI / 180.0;
            pitch_ = rotation_deg[1] * M_PI / 180.0;
            yaw_ = rotation_deg[2] * M_PI / 180.0;
        }

        RCLCPP_INFO(get_logger(), "Loaded equirectangular parameters");
        RCLCPP_INFO(
            get_logger(), "  crop=%d, mount_roll=%d, output=%dx%d",
            crop_size_, mount_roll_deg_, out_width_, out_height_);
        RCLCPP_INFO(
            get_logger(), "  front offset=(%.1f, %.1f), back offset=(%.1f, %.1f)",
            cx_offset_, cy_offset_, back_cx_offset_, back_cy_offset_);
    }

    void update_camera_parameters()
    {
        const cv::Mat rotation_x = (cv::Mat_<double>(3, 3) <<
            1.0, 0.0, 0.0,
            0.0, std::cos(roll_), -std::sin(roll_),
            0.0, std::sin(roll_), std::cos(roll_));
        const cv::Mat rotation_y = (cv::Mat_<double>(3, 3) <<
            std::cos(pitch_), 0.0, std::sin(pitch_),
            0.0, 1.0, 0.0,
            -std::sin(pitch_), 0.0, std::cos(pitch_));
        const cv::Mat rotation_z = (cv::Mat_<double>(3, 3) <<
            std::cos(yaw_), -std::sin(yaw_), 0.0,
            std::sin(yaw_), std::cos(yaw_), 0.0,
            0.0, 0.0, 1.0);

        back_to_front_rotation_ = rotation_z * rotation_y * rotation_x;
        back_to_front_translation_ = cv::Vec3d(tx_, ty_, tz_);
        params_changed_ = true;
        maps_initialized_ = false;
    }

    void init_mapping(int image_height, int image_width)
    {
        RCLCPP_INFO(
            get_logger(),
            "Initializing equirectangular projection: %dx%d -> %dx%d",
            image_width, image_height, out_width_, out_height_);

        img_height_ = image_height;
        img_width_ = image_width;
        cx_ = image_width / 2.0 + cx_offset_;
        cy_ = image_height / 2.0 + cy_offset_;
        back_cx_ = image_width / 2.0 + back_cx_offset_;
        back_cy_ = image_height / 2.0 + back_cy_offset_;

        cv::Mat x_grid;
        cv::Mat y_grid;
        cv::Mat x_range = cv::Mat::zeros(1, out_width_, CV_32F);
        cv::Mat y_range = cv::Mat::zeros(out_height_, 1, CV_32F);
        for (int x = 0; x < out_width_; ++x) {
            x_range.at<float>(0, x) = static_cast<float>(x);
        }
        for (int y = 0; y < out_height_; ++y) {
            y_range.at<float>(y, 0) = static_cast<float>(y);
        }
        cv::repeat(x_range, out_height_, 1, x_grid);
        cv::repeat(y_range, 1, out_width_, y_grid);

        const cv::Mat longitude = (x_grid / static_cast<float>(out_width_)) *
            2.0F * static_cast<float>(M_PI) - static_cast<float>(M_PI);
        const cv::Mat latitude = (y_grid / static_cast<float>(out_height_)) *
            static_cast<float>(M_PI) - static_cast<float>(M_PI) / 2.0F;

        cv::Mat x;
        cv::Mat y;
        cv::Mat z;
        cv::Mat cos_latitude = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        cv::Mat sin_latitude = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        cv::Mat cos_longitude = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        cv::Mat sin_longitude = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        for (int row = 0; row < out_height_; ++row) {
            for (int col = 0; col < out_width_; ++col) {
                const float lat = latitude.at<float>(row, col);
                const float lon = longitude.at<float>(row, col);
                cos_latitude.at<float>(row, col) = std::cos(lat);
                sin_latitude.at<float>(row, col) = std::sin(lat);
                cos_longitude.at<float>(row, col) = std::cos(lon);
                sin_longitude.at<float>(row, col) = std::sin(lon);
            }
        }

        x = cos_latitude.mul(sin_longitude);
        y = sin_latitude;
        z = cos_latitude.mul(cos_longitude);
        front_mask_ = z >= 0;
        back_mask_ = z < 0;

        front_map_x_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        front_map_y_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        back_map_x_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        back_map_y_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);

        for (int row = 0; row < out_height_; ++row) {
            for (int col = 0; col < out_width_; ++col) {
                if (front_mask_.at<uchar>(row, col)) {
                    const float x_value = x.at<float>(row, col);
                    const float y_value = y.at<float>(row, col);
                    const float z_value = z.at<float>(row, col);
                    float radius = std::sqrt(x_value * x_value + y_value * y_value);
                    if (radius < 1e-6F) {
                        radius = 1e-6F;
                    }
                    const float theta = std::atan2(radius, std::fabs(z_value));
                    const float fisheye_radius =
                        2.0F * theta / static_cast<float>(M_PI) * (image_width / 2.0F);
                    front_map_x_.at<float>(row, col) =
                        static_cast<float>(cx_) + x_value / radius * fisheye_radius;
                    front_map_y_.at<float>(row, col) =
                        static_cast<float>(cy_) + y_value / radius * fisheye_radius;
                } else {
                    const cv::Vec3d point(
                        x.at<float>(row, col), y.at<float>(row, col), z.at<float>(row, col));
                    const cv::Mat transformed =
                        back_to_front_rotation_ * cv::Mat(point) +
                        cv::Mat(back_to_front_translation_);
                    const float x_back = -transformed.at<double>(0);
                    const float y_back = transformed.at<double>(1);
                    const float z_back = transformed.at<double>(2);
                    float radius = std::sqrt(x_back * x_back + y_back * y_back);
                    if (radius < 1e-6F) {
                        radius = 1e-6F;
                    }
                    const float theta = std::atan2(radius, std::fabs(z_back));
                    const float fisheye_radius =
                        2.0F * theta / static_cast<float>(M_PI) * (image_width / 2.0F) *
                        static_cast<float>(back_radius_scale_);
                    back_map_x_.at<float>(row, col) =
                        static_cast<float>(back_cx_) + x_back / radius * fisheye_radius;
                    back_map_y_.at<float>(row, col) =
                        static_cast<float>(back_cy_) + y_back / radius * fisheye_radius;
                }
            }
        }

        maps_initialized_ = true;
        RCLCPP_INFO(get_logger(), "Equirectangular mapping initialized");
    }

    cv::Mat create_equirectangular(const cv::Mat &front_image, const cv::Mat &back_image)
    {
        if (!maps_initialized_ || params_changed_ ||
            front_image.rows != img_height_ || front_image.cols != img_width_) {
            init_mapping(front_image.rows, front_image.cols);
            params_changed_ = false;
        }

        cv::Mat front_result;
        cv::Mat back_result;
        cv::remap(
            front_image, front_result, front_map_x_, front_map_y_, cv::INTER_CUBIC,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::remap(
            back_image, back_result, back_map_x_, back_map_y_, cv::INTER_CUBIC,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        cv::Mat equirectangular = cv::Mat::zeros(
            out_height_, out_width_, CV_8UC3);
        front_result.copyTo(equirectangular, front_mask_);
        back_result.copyTo(equirectangular, back_mask_);
        return equirectangular;
    }

    void enqueue_decoded_frame(const std_msgs::msg::Header &header)
    {
        if (bgr_frame_.empty()) {
            return;
        }
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (frame_queue_.size() >= max_queue_size_) {
            frame_queue_.pop();
        }
        frame_queue_.push(DecodedFrame{bgr_frame_.clone(), header});
        queue_cv_.notify_one();
    }

    void decode_packet(
        AVPacket *packet, const std_msgs::msg::Header &header)
    {
        int result = avcodec_send_packet(codec_ctx_, packet);
        if (result < 0) {
            return;
        }

        while (result >= 0) {
            result = avcodec_receive_frame(codec_ctx_, hw_frame_);
            if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
                return;
            }
            if (result < 0) {
                return;
            }

            AVFrame *frame_to_convert = hw_frame_;
            if (hw_frame_->format == AV_PIX_FMT_CUDA && sw_frame_) {
                if (av_hwframe_transfer_data(sw_frame_, hw_frame_, 0) < 0) {
                    av_frame_unref(hw_frame_);
                    continue;
                }
                frame_to_convert = sw_frame_;
            }

            if (!sws_ctx_ && frame_to_convert->width > 0 && frame_to_convert->height > 0) {
                sws_ctx_ = sws_getContext(
                    frame_to_convert->width,
                    frame_to_convert->height,
                    static_cast<AVPixelFormat>(frame_to_convert->format),
                    frame_to_convert->width,
                    frame_to_convert->height,
                    AV_PIX_FMT_BGR24,
                    SWS_POINT,
                    nullptr,
                    nullptr,
                    nullptr);
                if (!sws_ctx_) {
                    av_frame_unref(hw_frame_);
                    if (frame_to_convert == sw_frame_) {
                        av_frame_unref(sw_frame_);
                    }
                    return;
                }
                bgr_frame_.create(
                    frame_to_convert->height, frame_to_convert->width, CV_8UC3);
            }

            if (sws_ctx_ && !bgr_frame_.empty()) {
                uint8_t *destination_data[4] = {bgr_frame_.data, nullptr, nullptr, nullptr};
                int destination_linesize[4] = {
                    static_cast<int>(bgr_frame_.step[0]), 0, 0, 0};
                sws_scale(
                    sws_ctx_,
                    reinterpret_cast<const uint8_t *const *>(frame_to_convert->data),
                    frame_to_convert->linesize,
                    0,
                    frame_to_convert->height,
                    destination_data,
                    destination_linesize);

                bool should_enqueue = true;
                if (skip_frame_ > 0 && !i_frame_only_) {
                    should_enqueue =
                        (frame_counter_++ % (skip_frame_ + 1)) == 0;
                }
                if (should_enqueue) {
                    enqueue_decoded_frame(header);
                }
            }

            av_frame_unref(hw_frame_);
            if (frame_to_convert == sw_frame_) {
                av_frame_unref(sw_frame_);
            }
        }
    }

    void compressed_image_callback(
        const sensor_msgs::msg::CompressedImage::SharedPtr message)
    {
        if (message->format != "h264" || message->data.empty() ||
            !codec_ctx_ || !packet_ || !hw_frame_) {
            return;
        }

        // The camera supplies one complete Annex-B access unit per callback.
        // Keep the callback boundary intact; parsing it again can separate the
        // SPS/PPS from the IDR frame on the X3 stream.
        av_packet_unref(packet_);
        if (av_new_packet(packet_, static_cast<int>(message->data.size())) < 0) {
            return;
        }
        std::memcpy(packet_->data, message->data.data(), message->data.size());
        decode_packet(packet_, message->header);
        av_packet_unref(packet_);
    }

    void publish_panorama(const DecodedFrame &decoded)
    {
        if (decoded.bgr.empty()) {
            return;
        }

        std::lock_guard<std::mutex> lock(processing_mutex_);
        cv::Mat rgb_image;
        cv::cvtColor(decoded.bgr, rgb_image, cv::COLOR_BGR2RGB);

        const int image_height = rgb_image.rows;
        const int image_width = rgb_image.cols;
        const int midpoint = image_width / 2;
        if (midpoint <= 0 || image_height <= 0) {
            return;
        }

        cv::Mat front_image = rgb_image(cv::Rect(midpoint, 0, midpoint, image_height));
        cv::Mat back_image = rgb_image(cv::Rect(0, 0, midpoint, image_height));
        cv::rotate(front_image, front_image, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(back_image, back_image, cv::ROTATE_90_CLOCKWISE);

        const int mount_roll = ((mount_roll_deg_ % 360) + 360) % 360;
        if (mount_roll == 90) {
            cv::rotate(front_image, front_image, cv::ROTATE_90_CLOCKWISE);
            cv::rotate(back_image, back_image, cv::ROTATE_90_COUNTERCLOCKWISE);
        } else if (mount_roll == 180) {
            cv::rotate(front_image, front_image, cv::ROTATE_180);
            cv::rotate(back_image, back_image, cv::ROTATE_180);
        } else if (mount_roll == 270) {
            cv::rotate(front_image, front_image, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::rotate(back_image, back_image, cv::ROTATE_90_CLOCKWISE);
        } else if (mount_roll != 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "mount_roll_deg must be one of -90, 0, 90, or 180");
        }

        const int current_crop_size = crop_size_;
        const int original_height = front_image.rows;
        const int original_width = front_image.cols;
        if (original_height != current_crop_size || original_width != current_crop_size) {
            const int y_start = (original_height - current_crop_size) / 2;
            const int x_start = (original_width - current_crop_size) / 2;
            if (y_start >= 0 && x_start >= 0 &&
                y_start + current_crop_size <= original_height &&
                x_start + current_crop_size <= original_width) {
                front_image = front_image(
                    cv::Rect(x_start, y_start, current_crop_size, current_crop_size));
                back_image = back_image(
                    cv::Rect(x_start, y_start, current_crop_size, current_crop_size));
            }
        }

        const cv::Mat panorama = create_equirectangular(front_image, back_image);
        if (compressed_publisher_->get_subscription_count() > 0) {
            cv::Mat bgr_panorama;
            cv::cvtColor(panorama, bgr_panorama, cv::COLOR_RGB2BGR);
            auto output = sensor_msgs::msg::CompressedImage();
            output.header = decoded.header;
            output.format = "jpeg";
            if (cv::imencode(".jpg", bgr_panorama, output.data,
                             {cv::IMWRITE_JPEG_QUALITY, 95})) {
                compressed_publisher_->publish(std::move(output));
            }
        }
        if (publisher_->get_subscription_count() > 0) {
            cv_bridge::CvImage output(
                decoded.header, sensor_msgs::image_encodings::RGB8, panorama);
            publisher_->publish(*output.toImageMsg());
        }
    }

    void processing_thread_loop()
    {
        while (!stop_processing_) {
            DecodedFrame decoded;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] {
                    return stop_processing_ || !frame_queue_.empty();
                });
                if (stop_processing_ && frame_queue_.empty()) {
                    return;
                }
                if (frame_queue_.empty()) {
                    continue;
                }
                decoded = std::move(frame_queue_.front());
                frame_queue_.pop();
            }
            publish_panorama(decoded);
        }
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        bool update_needed = false;
        for (const auto &parameter : parameters) {
            const std::string &name = parameter.get_name();
            if (name == "cx_offset" || name == "cy_offset" ||
                name == "back_cx_offset" || name == "back_cy_offset" ||
                name == "back_radius_scale" || name == "crop_size" ||
                name == "mount_roll_deg" || name == "translation" ||
                name == "rotation_deg" || name == "out_width" ||
                name == "out_height" || name == "gpu") {
                update_needed = true;
            }
        }

        if (update_needed) {
            std::lock_guard<std::mutex> lock(processing_mutex_);
            load_parameters();
            update_camera_parameters();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

public:
    PanoramaNode()
    : Node("equirectangular_node")
    {
        declare_parameter("compressed_topic", "/dual_fisheye/image/compressed");
        declare_parameter("output_topic", "/equirectangular/image");
        declare_parameter("compressed_output_topic", "/equirectangular/image/compressed");
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);
        declare_parameter("use_hardware_decoder", false);
        declare_parameter("max_queue_size", 3);

        declare_parameter("cx_offset", 0.0);
        declare_parameter("cy_offset", 4.0);
        declare_parameter("back_cx_offset", 0.0);
        declare_parameter("back_cy_offset", 4.0);
        declare_parameter("back_radius_scale", 1.0);
        declare_parameter("crop_size", 1920);
        declare_parameter("mount_roll_deg", 90);
        declare_parameter(
            "translation", std::vector<double>{-0.034, -0.004, -0.226});
        declare_parameter("rotation_deg", std::vector<double>{-0.5, 0.0, 1.1});
        declare_parameter("gpu", true);
        declare_parameter("out_width", 1440);
        declare_parameter("out_height", 720);

        const std::string compressed_topic =
            get_parameter("compressed_topic").as_string();
        const std::string output_topic = get_parameter("output_topic").as_string();
        const std::string compressed_output_topic =
            get_parameter("compressed_output_topic").as_string();
        skip_frame_ = get_parameter("skip_frame").as_int();
        i_frame_only_ = get_parameter("i_frame_only").as_bool();
        use_hardware_decoder_ = get_parameter("use_hardware_decoder").as_bool();
        max_queue_size_ = static_cast<size_t>(std::max<int64_t>(
            1, get_parameter("max_queue_size").as_int()));

        load_parameters();
        update_camera_parameters();
        parameter_callback_handle_ = add_on_set_parameters_callback(
            std::bind(
                &PanoramaNode::parameters_callback, this,
                std::placeholders::_1));

        subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            compressed_topic,
            rclcpp::QoS(10).reliable(),
            std::bind(
                &PanoramaNode::compressed_image_callback, this,
                std::placeholders::_1));
        publisher_ = create_publisher<sensor_msgs::msg::Image>(
            output_topic, rclcpp::SensorDataQoS().keep_last(1));
        compressed_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            compressed_output_topic, rclcpp::SensorDataQoS().keep_last(1));

        init_ffmpeg_decoder();
        processing_thread_ = std::thread(&PanoramaNode::processing_thread_loop, this);

        RCLCPP_INFO(get_logger(), "Integrated H.264-to-equirectangular node initialized");
        RCLCPP_INFO(get_logger(), "  input: %s", compressed_topic.c_str());
        RCLCPP_INFO(get_logger(), "  output: %s", output_topic.c_str());
        RCLCPP_INFO(get_logger(), "  compressed output: %s", compressed_output_topic.c_str());
        RCLCPP_INFO(
            get_logger(), "  raw /dual_fisheye/image topic is intentionally not published");
    }

    ~PanoramaNode() override
    {
        stop_processing_ = true;
        queue_cv_.notify_one();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        cleanup_ffmpeg_decoder();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PanoramaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
