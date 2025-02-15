source install/setup.bash

ros2 run magbot magbot

./updateURDFAndBuildGazebo.sh

ros2 run rtabmap_odom icp_odometry

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world odom

ros2 run magbot_input_interfacing magbot_keyboard_interfacing

ros2 run rviz2 rviz2