cd src/magbot_gazebo/description/urdf

xacro dingo.urdf.xacro -o dingo.urdf

gz sdf -p ./dingo.urdf > ./model.sdf

cp model.sdf ~/.gazebo/models/magbot_gazebo/model.sdf

cd /home/jarvis/magic/ROS/magbot_ws/

colcon build --packages-select magbot_gazebo magbot

source install/setup.bash

ros2 launch magbot_gazebo rsp.launch.py