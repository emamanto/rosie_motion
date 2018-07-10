# rosie_motion
Motion control ROS node for rosie

## Installation requirements

1. Ubuntu 16.04 or bust

2. ROS Kinetic, [installation instructions here](http://wiki.ros.org/kinetic/Installation/Ubuntu)

3. Package dependencies: `sudo apt-get install cmake gazebo7 git-core git-gui
ros-kinetic-rosbridge-server ros-kinetic-rgdb-launch ros-kinetic-moveit ros-kinetic-costmap-2d
ros-kinetic-moveit-python ros-kinetic-control-toolbox python-catkin-tools`

4. Create `~/catkin_ws` and `~/catkin_ws/src`, `cd ~/catkin_ws`, run `catkin build`

5. Add line to `~/.bashrc`: `source ~/catkin_ws/devel/setup.bash`

6. Clone these repos into `~/catkin_ws/src`:  
  [fetchrobotics/fetch_ros](https://github.com/fetchrobotics/fetch_ros)  
  [fetchrobotics/fetch_gazebo](https://github.com/fetchrobotics/fetch_gazebo) - then check out the gazebo7 branch  
  [fetchrobotics/robot_controllers](https://github.com/fetchrobotics/robot_controllers)  
  [wg-perception/opencv_candidate](https://github.com/wg-perception/opencv_candidate)  
  [emamanto/rosie_msgs](https://github.com/emamanto/rosie_msgs)  
  ...and this repo, of course
  
7. Download the [rapidjson](https://github.com/Tencent/rapidjson/) header files and put them in `~/catkin_ws/src/rosie_motion/include`, and
make sure the path in the init() function in ObjectDatabase.cpp is correct for your computer

8. In `~/catkin_ws`, run `catkin build` again to check that all dependencies have been met

9. Launch main program with `roslaunch rosie_motion sim_rosie_motion.launch`
