#include "orca_shared/mw/fiducial_pose_stamped.hpp"
#include "orca_shared/test.hpp"

// New school
bool test_mw_roundtrip()
{
  std::cout << "=== TEST ROUNDTRIP ===" << std::endl;

  const mw::Header header1{rclcpp::Time{0, 999, RCL_ROS_TIME}, "map"};
  const mw::Header header2 = mw::Header{header1.msg()};

  std::cout << header1 << std::endl;
  std::cout << header2 << std::endl;

  if (header1 != header2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  const mw::Observation observation1{1, {2, 3}, {4, 5}, {6, 7}, {8, 9}};
  const mw::Observation observation2 = mw::Observation{observation1.msg()};

  std::cout << observation1 << std::endl;
  std::cout << observation2 << std::endl;

  if (observation1 != observation2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  const mw::PolarObservation polar_observation1{1, 10, -0.5};
  const mw::PolarObservation polar_observation2 = mw::PolarObservation{polar_observation1.msg()};

  std::cout << polar_observation1 << std::endl;
  std::cout << polar_observation2 << std::endl;

  if (polar_observation1 != polar_observation2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  mw::Point point1{1, 2, 3};
  const mw::Point point2 = mw::Point{point1.msg()};

  std::cout << point1 << std::endl;
  std::cout << point2 << std::endl;

  if (point1 != point2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  mw::Quaternion quaternion1{0, 0, 1};
  const mw::Quaternion quaternion2 = mw::Quaternion{quaternion1.msg()};

  std::cout << quaternion1 << std::endl;
  std::cout << quaternion2 << std::endl;

  if (quaternion1 != quaternion2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  const mw::Pose pose1{point1, quaternion1};
  const mw::Pose pose2 = mw::Pose{pose1.msg()};

  std::cout << pose1 << std::endl;
  std::cout << pose2 << std::endl;

  if (pose1 != pose2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  mw::CovarianceType covariance1{};
  double variance = 0.1;
  for (auto &item : covariance1) {
    item = variance++;
  }

  const mw::PoseWithCovariance pose_with_covariance1{pose1, covariance1};
  const mw::PoseWithCovariance pose_with_covariance2 = mw::PoseWithCovariance{pose_with_covariance1.msg()};

  std::cout << pose_with_covariance1 << std::endl;
  std::cout << pose_with_covariance2 << std::endl;

  if (pose_with_covariance1 != pose_with_covariance2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(make_camera_info());
  tf2::Transform t_base_cam;
  tf2::fromMsg(make_cam_f_base(), t_base_cam);
  const mw::Observer observer1{0.2, cam_model, t_base_cam};
  const mw::Observer observer2 = mw::Observer{observer1.msg()};

  std::cout << observer1 << std::endl;
  std::cout << observer2 << std::endl;

  if (observer1 != observer2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  mw::Observations observations1{observer1};
  observations1.add(observation1);
  observations1.add_polar(polar_observation1);
  const mw::Observations observations2 = mw::Observations{observations1.msg()};

  std::cout << observations1 << std::endl;
  std::cout << observations2 << std::endl;

  if (observations1 != observations2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  const mw::FiducialPose fp1{observations1, pose_with_covariance1};
  const mw::FiducialPose fp2 = mw::FiducialPose{fp1.msg()};

  std::cout << fp1 << std::endl;
  std::cout << fp2 << std::endl;

  if (fp1 != fp2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  const mw::FiducialPoseStamped fps1{header1, fp1};
  const mw::FiducialPoseStamped fps2 = mw::FiducialPoseStamped{fps1.msg()};

  std::cout << fps1 << std::endl;
  std::cout << fps2 << std::endl;

  if (fps1 != fps2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  std::cout << "success" << std::endl;
  return true;
}
