## ROS Tutorials: Exploring ROS using a 2 wheeled Robot

[Part 1](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/)

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
