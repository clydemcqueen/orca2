#include "h264_msgs/msg/packet.hpp"
#include "rclcpp/rclcpp.hpp"

extern "C" {
#include <libavutil/imgutils.h>
#include <libavdevice/avdevice.h>
}

namespace orca_driver
{

  constexpr int QUEUE_SIZE = 10;

  class H264CamNode : public rclcpp::Node
  {
    AVFormatContext *format_context_{nullptr};
    std::thread cam_thread_;
    std::atomic<bool> stop_signal_{false};
    rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr h264_pub_;

  public:

    H264CamNode(const std::string &input_fn, const std::string &fps, const std::string &size) :
      Node{"h264_cam_node"}
    {
      std::cout << "start ffmpeg setup" << std::endl;

      av_register_all();
      avdevice_register_all();
      av_log_set_level(AV_LOG_WARNING);

      // Device drivers appear as formats in ffmpeg
      // Find the v4l driver
      AVInputFormat *input_format = av_find_input_format("video4linux2");
      if (!input_format) {
        std::cerr << "Could not find the v4l driver" << std::endl;
        exit(1);
      }

      // Allocate a format context
      format_context_ = avformat_alloc_context();
      if (!format_context_) {
        std::cerr << "Could not allocate a format context" << std::endl;
        exit(1);
      }

      // Set format options, this will allocate an AVDictionary
      AVDictionary *format_options = nullptr;
      av_dict_set(&format_options, "input_format", "h264", 0);
      av_dict_set(&format_options, "framerate", fps.c_str(), 0);
      av_dict_set(&format_options, "video_size", size.c_str(), 0);

      // Open input, pass ownership of format_options
      if (avformat_open_input(&format_context_, input_fn.c_str(), input_format, &format_options) < 0) {
        std::cerr << "Could not open the v4l device " << input_fn << std::endl;
        exit(1);
      }

      // Get stream info from the input
      if (avformat_find_stream_info(format_context_, nullptr) < 0) {
        std::cerr << "Could not find the device stream info" << std::endl;
        exit(1);
      }

      // TODO select a stream?

      // Find the video stream (vs audio, metadata, etc.)
//      int stream_idx = av_find_best_stream(format_context_, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
//      if (stream_idx < 0) {
//        std::cerr << "Could not find a video stream on the device" << std::endl;
//        exit(1);
//      }

//      AVStream *stream = format_context_->streams[stream_idx];

      std::cout << "finish ffmpeg setup" << std::endl;

      h264_pub_ = create_publisher<h264_msgs::msg::Packet>("image_raw/h264", QUEUE_SIZE);

      cam_thread_ = std::thread(
        [this]()
        {
          while (!stop_signal_ && rclcpp::ok()) {
            // Block until a frame is ready
            AVPacket packet;
            if (av_read_frame(format_context_, &packet) < 0) {
              RCLCPP_INFO(get_logger(), "EOS");
              break;
            }

            // Publish h264 message
            h264_msgs::msg::Packet h264_msg;
            h264_msg.data.insert(h264_msg.data.end(), &packet.data[0], &packet.data[packet.size]);
            h264_msg.header.stamp = now();
            h264_msg.header.frame_id = "temp_frame_id"; // TODO
            h264_pub_->publish(h264_msg);

            av_packet_unref(&packet);
          }

          RCLCPP_INFO(get_logger(), "cam thread stopped");
        });

      RCLCPP_INFO(get_logger(), "h264_cam_node running");
    }

    ~H264CamNode() override
    {
      avformat_free_context(format_context_);
      stop_signal_ = true;
      cam_thread_.join();
    }
  };

} // namespace orca_driver

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_driver::H264CamNode>("/dev/video2", "30", "800x600");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
