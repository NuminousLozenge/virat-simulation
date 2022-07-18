# virat-simulation
Packages for simulating Virat using ros, gazebo and rviz

## Steps for installing
- Create a catkin workspace (catkin_ws)
- Clone the repository into the src folder of catkin workspace (catkin_ws/src/)
- run catkin_make in catkin_ws

## Steps for running the simulation
- Open 3 separate terminals and source the packages using source devel/setup.bash inside catkin_ws
- Run the following commands in the separate terminals in order

Gazebo:

    roslaunch virat_gazebo virat_world.launch
    
Rviz:

    roslaunch virat_description virat_rviz.launch
    
ROS control:

     rosrun virat_control virat_teleop_key.py
