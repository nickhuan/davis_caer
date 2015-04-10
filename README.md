# DAVIS_CAER ROS

The davis_caer package creates ROS node davis_caer, publishing topic davis_frame . This allows ROS to get davis frames via TCP. 

davis_frame is a stream of images of type mono16 and dimensions 180*240.


Installing (with ROS):
cd ~/catkin_ws/src  
git clone https://github.com/nickhuan/davis_caer.git
Compiling (catkin):
cd ~/catkin_ws  
catkin_make davis_caer
Running:
roscore
rosrun davis_caer davis_caer
