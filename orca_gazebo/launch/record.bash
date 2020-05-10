# Be sure to install ros2bag:
#sudo apt install ros-eloquent-rosbag2-storage-default-plugins ros-eloquent-ros2bag

ros2 bag record -o bags/wall_bag control barometer forward_camera/camera_pose left_camera/camera_pose right_camera/camera_pose tf tf_static