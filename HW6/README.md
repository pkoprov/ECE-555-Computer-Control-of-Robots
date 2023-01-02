# Course Project

In this project involves computer control of the sawyer robot for a continous pick and place action using the ROS/Gazebo Development environment. The course project consist of the following:

## Part 1: Object Pose Detections

* A. Pose of the three targets with the repect to the robot base frame
* B. The joint variables to grasp the object at each location, ie. inverse kinematics
* C. The Jacobian matrix at each location using the joint variables from Part B

An example ROS Package, ece_sawyer_project, has been created to assist the project. This package is available to download using the following:


```commandline
git clone https://<your_username>@bitbucket.org/livingston_ai/ece_sawyer_project.git
```

The repository also contains example source code for motion planning of the sawyer robot. Part 2 involves writing a software algorithm to manipulate the sawyer robot. You may use the sample code as a starting place.

## Part 2: Computer Control of Robot System

Write a computer algorithm the performs the following task. (Example video available)
* 1. Pick up and discard target 2 from the scene
* 2. Pick up target from location 1 and place location 2
* 3. Pick up target from location 3 and place in location 1
* 4. Pick up target from location 2 and place in location 3
* Repeat steps 2 - 4

The program should be demonstrated using the following actions

Bring up the simulator.  Execute in terminal 1:
```commandline
./intera.sh sim
roslaunch ece_sawyer_project sawyer_world.launch
```

Enable the sawyer robot. Execute in terminal 2:
```commandline
./intera.sh sim
rosrun intera_interface enable_robot.py -e
```

Let’s start by launch the trajectory action server of the robot with the following command.  
```commandline
rosrun intera_interface joint_trajectory_action_server.py
```

Then run the moveit package again with the controller. Execute in terminal 3:
```commandline
./intera.sh sim
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
```
Execute motion planning algorithm. 
```commandline
./intera.sh sim 
rosrun ece_sawyer_project sawyer_pick_and_place
```
Try to plan the trajectory and execute again, you should see the robot move while executing the trajectory.