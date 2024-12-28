cd src/magbot_gazebo/description/urdf

xacro dingo.urdf.xacro -o dingo.urdf

gz sdf -p ./dingo.urdf > ./model.sdf

cp model.sdf ~/.gazebo/models/magbot_gazebo/model.sdf

cd /home/jarvis/magic/ROS/magbot_ws/
