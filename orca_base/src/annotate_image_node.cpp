#include <iomanip>
#include <orca_shared/mw/mw.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/node.hpp"

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/monotonic.hpp"

namespace orca_base
{

  //=============================================================================
  // Perf testing
  //=============================================================================

#undef RUN_PERF
#ifdef RUN_PERF
#define START_PERF()\
auto __start__ = std::chrono::high_resolution_clock::now();

#define STOP_PERF(msg)\
auto __stop__ = std::chrono::high_resolution_clock::now();\
std::cout << msg << " " << std::chrono::duration_cast<std::chrono::microseconds>(__stop__ - __start__).count()\
  << " microseconds" << std::endl;
#else
#define START_PERF()
#define STOP_PERF(msg)
#endif

  //=============================================================================
  // Utilities
  //=============================================================================

  void draw_text(cv::Mat &image, const std::string &s, const cv::Point2d &p, cv::Scalar color, int scale)
  {
    putText(image, s.c_str(), p, cv::FONT_HERSHEY_PLAIN, scale, std::move(color), scale);
  }

  void draw_observations(cv::Mat &image, const mw::Observations &observations, const cv::Scalar &color, int scale)
  {
    auto o = observations.observations().begin();
    auto p = observations.polar_observations().begin();
    for (; o != observations.observations().end() && p != observations.polar_observations().end(); ++o, ++p) {
      // Draw marker name
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << "m" << o->id() << " d=" << p->distance();
      draw_text(image, ss.str(), cv::Point2d{o->c3().x, o->c3().y + scale * 30}, color, scale);

      // Draw bounding box
      cv::line(image, o->c0(), o->c1(), color, scale);
      cv::line(image, o->c1(), o->c2(), color, scale);
      cv::line(image, o->c2(), o->c3(), color, scale);
      cv::line(image, o->c3(), o->c0(), color, scale);
    }
  }

  //=============================================================================
  // AnnotateNode
  //=============================================================================

  // Default queue size
  // Testing with a value of 1 means that some messages get dropped
  constexpr int QUEUE_SIZE = 10;

  class AnnotateNode : public rclcpp::Node
  {
    // Subscriptions
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Callback wrappers
    monotonic::Valid<AnnotateNode *, orca_msgs::msg::Control::SharedPtr>
      control_cb_{this, &AnnotateNode::control_callback};
    monotonic::Valid<AnnotateNode *, sensor_msgs::msg::Image::SharedPtr>
      image_cb_{this, &AnnotateNode::image_callback};

    // Publications
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // Latest control message
    orca_msgs::msg::Control control_msg_;

    void control_callback(orca_msgs::msg::Control::SharedPtr msg)
    {
      control_msg_ = *msg;
    }

    void image_callback(sensor_msgs::msg::Image::SharedPtr msg)
    {
      START_PERF()

      // Wait for a valid control message
      if (!monotonic::valid(control_msg_.header.stamp)) {
        return;
      }

      if (image_pub_->get_subscription_count() > 0) {
        // Scale up for high res images
        // TODO
//        int scale = (int)std::round(control_msg_.estimate_observations.camera_info.width / 800.);
        int scale = 1;

        cv_bridge::CvImagePtr marked;
        marked = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if (control_msg_.mode == orca_msgs::msg::Control::AUV) {
          draw_observations(marked->image,
                            mw::Observations{control_msg_.mission.pose.fp.observations},
                            CV_RGB(255, 0, 0), scale);
        }
        draw_observations(marked->image,
                          mw::Observations{control_msg_.estimate.observations},
                          CV_RGB(0, 255, 0), scale);
        write_status(marked->image, scale);

        image_pub_->publish(*marked->toImageMsg());
      }

      STOP_PERF("image_callback")
    }

    void write_status(cv::Mat &image, int scale)
    {
      cv::Point2d p{scale * 10., scale * 20.};
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2);

      // Line 1: mission status
      if (control_msg_.mode == orca_msgs::msg::Control::AUV) {
        ss << "AUV target m" << control_msg_.mission.target_marker_id;
        if (control_msg_.mission.planner == orca_msgs::msg::MissionState::PLAN_RECOVERY_MTM) {
          // Expect exactly 1 planned observation
          if (control_msg_.mission.pose.fp.observations.polar_observations.size() == 1) {
            ss << " RECOVERY move to marker m" << control_msg_.mission.pose.fp.observations.polar_observations[0].id;
          } else {
            ss << " RECOVERY move to marker [ERROR " << control_msg_.mission.pose.fp.observations.polar_observations.size()
               << "planned polar observations]";
          }
        } else {
          ss << " " << control_msg_.mission.segment_info;
        }
      } else {
        ss << "ROV";
      }
      draw_text(image, ss.str(), p, CV_RGB(255, 0, 0), scale);

      // Line 2: pose estimate
      p.y += scale * 20.;
      ss.str("");
      if (control_msg_.good_pose) {
        ss << "GOOD POSE";
      } else if (control_msg_.has_good_observation) {
        ss << "GOOD OBSERVATION";
      } else if (!control_msg_.estimate.observations.observations.empty()) {
        ss << "TOO FAR AWAY";
      } else {
        ss << "NO OBSERVATIONS";
      }
      ss << " z=" << control_msg_.estimate.pose.pose.position.z;

      draw_text(image, ss.str(), p, CV_RGB(0, 255, 0), scale);
    }

  public:

    explicit AnnotateNode() : Node{"annotate_image_node"}
    {
      // Subscribe
      control_sub_ = create_subscription<orca_msgs::msg::Control>(
        "/control", QUEUE_SIZE, [this](const orca_msgs::msg::Control::SharedPtr msg) -> void
        { this->control_cb_.call(msg); });
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "image_raw", QUEUE_SIZE, [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
        { this->image_cb_.call(msg); });

      // Advertise
      image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_annotated", QUEUE_SIZE);

      RCLCPP_INFO(get_logger(), "annotate_image_node ready");
    }

    ~AnnotateNode() override = default;
  };

} // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_base::AnnotateNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}