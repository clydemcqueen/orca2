#include "orca_filter/filter_base.hpp"

#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_filter
{

  //==================================================================
  // Utility functions
  //==================================================================

  constexpr int CONTROL_DIM = 4;          // [ax, ay, az, ayaw]T

  // Create control matrix u
  void to_u(const mw::Acceleration &in, Eigen::VectorXd &out)
  {
    out = Eigen::VectorXd(CONTROL_DIM);
    out << in.x(), in.y(), in.z(), in.yaw();
  }

  // Flatten a 6x6 covariance matrix
  void flatten_6x6_covar(const Eigen::MatrixXd &m, std::array<double, 36> &covar, int offset)
  {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        covar[i * 6 + j] = m(i + offset, j + offset);
      }
    }
  }

  //==================================================================
  // FilterBase
  //==================================================================

  constexpr int QUEUE_SIZE = 10;

  FilterBase::FilterBase(Type type,
                         const rclcpp::Logger &logger,
                         const FilterContext &cxt,
                         rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped2>::SharedPtr filtered_odom_pub,
                         rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub,
                         int state_dim) :

    type_{type},
    logger_{logger},
    cxt_{cxt},
    filtered_odom_pub_{filtered_odom_pub},
    tf_pub_{tf_pub},
    state_dim_{state_dim},
    filter_{state_dim, cxt_.ukf_alpha_, cxt_.ukf_beta_, cxt_.ukf_kappa_},
    odom_time_{0, 0, RCL_ROS_TIME}
  {
    reset();
  }

  void FilterBase::reset()
  {
    reset(Eigen::VectorXd::Zero(state_dim_));
  }

  void FilterBase::reset(const Eigen::VectorXd &x)
  {
    // Clear all pending measurements
    measurement_q_ = std::priority_queue<Measurement, std::vector<Measurement>, Measurement>();

    // Clear history
    state_history_.clear();
    measurement_history_.clear();

    // Reset filter time
    filter_time_ = {0, 0, RCL_ROS_TIME};

    // Start with a default state and a large covariance matrix
    filter_.set_x(x);
    filter_.set_P(Eigen::MatrixXd::Identity(state_dim_, state_dim_));
  }

  void FilterBase::predict(const rclcpp::Time &stamp, const mw::Acceleration &u_bar)
  {
    // Filter time starts at 0, test for this
    if (!orca::valid_stamp(filter_time_)) {
      RCLCPP_INFO(logger_, "start %s filter, stamp %s", name().c_str(), orca::to_str(stamp).c_str());
    } else {
      // Compute delta from last message, must be zero or positive
      double dt = (stamp - filter_time_).seconds();
      assert(dt >= 0);

      if (dt > cxt_.max_dt_) {
        // Delta is too large for some reason, use a smaller delta to avoid screwing up the filter
        RCLCPP_WARN(logger_, "dt %g is too large, use default %g", dt, cxt_.default_dt_);
        dt = cxt_.default_dt_;
      }

      if (dt < cxt_.min_dt_) {
        // Delta is quite small, possibly 0
        RCLCPP_DEBUG(logger_, "skip predict, stamp %s, filter %s",
                     orca::to_str(stamp).c_str(), orca::to_str(filter_time_).c_str());
      } else {
        // Run the prediction
        RCLCPP_DEBUG(logger_, "predict, stamp %s, filter %s",
                     orca::to_str(stamp).c_str(), orca::to_str(filter_time_).c_str());
        Eigen::VectorXd u;
        to_u(u_bar, u);
        filter_.predict(dt, u);
      }
    }

    // Always advance filter_time_
    filter_time_ = stamp;
  }

  // TODO take control input, will need to get estimated yaw to turn control into u_bar
  bool FilterBase::process_measurements(const rclcpp::Time &stamp)
  {
    // Trim state_history_
    while (!state_history_.empty() && state_history_.front().stamp_ < stamp - HISTORY_LENGTH) {
      RCLCPP_DEBUG(logger_, "pop old state history %s", orca::to_str(state_history_.front().stamp_).c_str());
      state_history_.pop_front();
    }

    // Trim measurement_history_
    while (!measurement_history_.empty() && measurement_history_.front().stamp_ < stamp - HISTORY_LENGTH) {
      RCLCPP_DEBUG(logger_, "pop old measurement history %s", orca::to_str(measurement_history_.front().stamp_).c_str());
      measurement_history_.pop_front();
    }

    // Set outlier distance, by doing this each time we can change this on-the-fly
    filter_.set_outlier_distance(cxt_.outlier_distance_);

    // Keep track of inliers and outliers
    int inliers = 0;
    int outliers = 0;

    // Process all measurements
    while (!measurement_q_.empty()) {
      RCLCPP_DEBUG(logger_, "processing measurement %s", orca::to_str(measurement_q_.top().stamp_).c_str());

      Measurement m = measurement_q_.top();
      measurement_q_.pop();

      predict(m.stamp_, {});

      filter_.set_h_fn(m.h_fn_);
      filter_.set_r_z_fn(m.r_z_fn_);
      filter_.set_mean_z_fn(m.mean_z_fn_);

      if (filter_.update(m.z_, m.R_)) {
        inliers++;

        /* If we're running a pose filter and getting both pose messages at 30Hz and depth messages at 10Hz
         * we can get into a state where the pose messages and depth messages have roughly the same timestamps,
         * but the depth messages arrive at the filter first. This is because vloc_node takes longer to calculate
         * a pose from an image -- so the "odom lag" for pose messages is higher than depth messages. The filter
         * works fine in this case:
         *
         * -- the depth message arrives with timestamp t, the filter is updated, odom is published at t
         * -- the pose messages arrives with timestamp t-e, the filter is rewound to t-e, then updated again with
         *    both messages. Odom is not published, because it has already been published at t.
         *
         * But a problem arises because auv_node is using an ExactSync message filter to combine odom and fiducial
         * observations. When this situation arises the ExactSync starts dropping messages because messages arrive
         * at timestamp t, not timestamp t-e.
         *
         * The solution is to avoid publishing odometry for depth messages when we're in a pose filter.
         * This behavior can be overridden using the cxt_.always_publish_odom_ flag.
         */
        if (type_ == Type::depth || m.type_ != Measurement::Type::depth || cxt_.always_publish_odom_) {
          publish_odom(m);
        }
      } else {
        outliers++;
      }

      // Save measurement in history
      measurement_history_.push_back(m);

      // Save state in history
      state_history_.emplace_back(m.stamp_, filter_.x(), filter_.P());
    }

    if (outliers) {
      RCLCPP_DEBUG(logger_, "rejected %d outlier(s)", outliers);
    }

    return inliers > 0 && filter_.valid();
  }

  // Rewind to a previous state, return true if successful, false if there was no change
  bool FilterBase::rewind(const rclcpp::Time &stamp)
  {
    if (state_history_.empty() || stamp < state_history_.front().stamp_) {
      RCLCPP_WARN(logger_, "can't rewind to %s, dropping message", orca::to_str(stamp).c_str());
      return false;
    }

    // Pop newer states
    while (!state_history_.empty() && state_history_.back().stamp_ > stamp) {
      RCLCPP_DEBUG(logger_, "rewind: pop state %s", orca::to_str(stamp).c_str());
      state_history_.pop_back();
    }

    // Set the filter state
    RCLCPP_DEBUG(logger_, "rewind %dms", (stamp - state_history_.back().stamp_).nanoseconds() / 1000000);
    filter_time_ = state_history_.back().stamp_;
    filter_.set_x(state_history_.back().x_);
    filter_.set_P(state_history_.back().P_);

    // Pop newer measurements and put them back into the priority queue
    while (!measurement_history_.empty() && measurement_history_.back().stamp_ > stamp) {
      RCLCPP_DEBUG(logger_, "rewind: re-queue measurement %s %s",
                   measurement_history_.back().name().c_str(), orca::to_str(stamp).c_str());
      measurement_q_.push(measurement_history_.back());
      measurement_history_.pop_back();
    }

    return true;
  }

  void FilterBase::publish_odom(const Measurement &measurement)
  {
    // If we rewind the filter the timestamp will repeat
    // Downstream consumers might get confused if (curr - prev) == 0
    if (filter_time_ == odom_time_) {
      return;
    }
    odom_time_ = filter_time_;

    // Get pose from the filter
    orca_msgs::msg::FiducialPoseStamped2 odom;
    odom.header.stamp = odom_time_;
    odom.header.frame_id = cxt_.frame_id_map_;
    odom_from_filter(odom.fp);

    // Copy the unfiltered observations from the measurement
    odom.fp.observations = measurement.observations_.msg();

    // Publish odometry
    if (filtered_odom_pub_->get_subscription_count() > 0) {
      filtered_odom_pub_->publish(odom);
    }

    // Publish filtered tf
    if (cxt_.publish_tf_ && tf_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::TransformStamped geo_tf;
      geo_tf.header = odom.header;
      geo_tf.child_frame_id = cxt_.frame_id_base_link_;

      // geometry_msgs::msg::Pose -> tf2::Transform -> geometry_msgs::msg::Transform
      tf2::Transform t_map_base;
      fromMsg(odom.fp.pose.pose, t_map_base);
      geo_tf.transform = toMsg(t_map_base);

      // One transform in this tf message
      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(geo_tf);

      tf_pub_->publish(tf_message);
    }
  }

  std::string FilterBase::name()
  {
    if (type_ == Type::depth) {
      return "depth";
    } else if (type_ == Type::four) {
      return "four";
    } else {
      return "pose";
    }
  }

} // namespace orca_filter
