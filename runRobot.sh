source install/setup.bash

ros2 run magbot magbot

./updateURDFAndBuildGazebo.sh

ros2 run rtabmap_odom icp_odometry

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world odom

ros2 run magbot_input_interfacing magbot_keyboard_interfacing

ros2 run rviz2 rviz2

ros2 launch slam_toolbox online_async_launch.py params_file:=./src/magbot_gazebo/config/mapper_params_online_async.yaml use_sim_time:=true