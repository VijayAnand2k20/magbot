cd src/magbot_gazebo/description/urdf

xacro dingo.urdf.xacro -o dingo.urdf

cd /home/jarvis/magic/ROS/magbot_ws/

colcon build --packages-select magbot_gazebo magbot

source install/setup.bash

ros2 launch magbot_gazebo rsp.launch.py