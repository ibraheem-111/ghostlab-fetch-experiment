export ROS_MASTER_URI=http://fetch1090:11311
source devel/setup.bash 
rosrun rviz rviz
rosrun fetch_latency_real delayed_camera.py
rosrun fetch_latency_real admittance_controller.py 
rosrun fetch_latency_real teleop.py
