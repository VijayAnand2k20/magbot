source install/setup.bash

ros2 run magbot magbot

./updateURDFAndBuildGazebo.sh

ros2 run rtabmap_odom icp_odometry --ros-args --remap use_sim_time:=true

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world odom

ros2 run robot_localization ekf_node --ros-args --params-file ./src/magbot_gazebo/config/ekf.yaml

ros2 run rviz2 rviz2 # Use default rviz, select /odom and /odometry/filtered topics to see them

# No need for now
# ros2 launch slam_toolbox online_async_launch.py params_file:=./src/magbot_gazebo/config/mapper_params_online_async.yaml use_sim_time:=true

ros2 run magbot_input_interfacing magbot_keyboard_interfacing