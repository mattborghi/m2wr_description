## ROS Tutorials: Exploring ROS using a 2 wheeled Robot

[Part 1](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/)

We are going to explore the basics of robot modeling using the Unified Robot Description Format (URDF) and we will have a model ready and running in Gazebo simulator.

* inside src/:
```sh$ catkin_create_pkg m2wr_description rospy```

* inside catkin_ws/:
```sh
$ catkin_make

$ source devel/setup.bash
```

* Launch rviz:
```sh
$ roslaunch m2wr_description rviz.launch 
```

* Launch gazebo:
```sh
$ roslaunch gazebo_ros empty_world.launch
```

* Load our model:
```sh
$ roslaunch m2wr_description spawn.launch 
```

[Part 2](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part2)

We are going to explore the macros for URDF files, using XACRO files and we will have the same model organized in different files, in a organized way.



