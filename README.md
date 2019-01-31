## ROS Tutorials: Exploring ROS using a 2 wheeled Robot

### [Part 1: Explore the basics of robot modeling using the URDF](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/)

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

* Run the following code in order to control the robot using the keyboard:
```sh
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### [Part 2: Explore the macros for URDF files using XACRO files](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part2)

We are going to explore the macros for URDF files, using XACRO files and we will have the same model organized in different files, in a organized way.

### [Part 3: Insert a laser scan sensor to the robot](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part3)

We are going to insert a laser scan sensor to a 2 wheeled robot the robot.

We will modify the urdf file (m2wr.xacro) as follows:

* Add a link element to our robot. This link will be cylindrical in shape and will represent the sensor.

* Add a joint element to our robot. This will connect the sensor to robot body rigidly.

* Define a new macro to calculate the inertial property of a cylinder using its dimensions (length and radius).

* Finally a laser scan sensor plugin element will add sensing ability to the link that we created (the cylinder representing the sensor).

* Having aded object in front of the robot in gazebo, open rviz and do the following:

	1. After starting rviz open the Graphical Tools window. Once rviz window loads you need to do the following settings

	2. Select odom in the Fixed Frame field (see the image below)
	
	3. Add two new displays using the Add button on the left bottom of rviz screen. The first display should be RobotModel and the other should be LaserScan
	
	4. Expand the LaserScan display by double clicking on its name and choose Topic as /m2wr/laser/scan

* Then move the robot and rviz should mark the sensor detecting this objects.

### [Part 4: Read the values of the laser scanner](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/#part4)

We are going to read the values of the laser scanner and filter a small part to work with.

* Clone the following repository, but just keep the 'my_worlds' folder:
```sh
$ cd simulation_ws/src

$ git clone https://marcoarruda@bitbucket.org/theconstructcore/two-wheeled-robot-simulation.git
```

The 'my_worlds' package consists of the following directories:

	1.	launch : Contains a launch file.

	2.	worlds : Contains multiple world description files.

Create a new catkin package named ```motion_plan``` with dependencies ```rospy```, ```std_msgs```, ```geometry_msgs``` and ```sensor_msgs```. Write the following commands:

```sh
$ cd ~/catkin_ws/src 

$ catkin_create_pgk motion_plan rospy std_msgs geometry_msgs sensor_msgs 

$ cd motion_plan 

$ mkdir scripts 

$ touch scripts/reading_laser.py
```


