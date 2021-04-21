# RTAB 3D SLAM
# Project 4: Map My World

An application of [rtabmap-ros](http://wiki.ros.org/rtabmap_ros) package for 
simultaneous localization and mapping (SLAM) of a mobile robot in an environment.

<table style="width:200%">
  <tr>
    <th><p>
           <img src="https://github.com/AnkushKansal/Map-my-World/blob/master/Output_Images/Capture1.PNG"
            alt="Capture1" width="400" height="400"></a>
           <br>Initial state
        </p>
    </th>
    <th><p>
           <img src="https://github.com/AnkushKansal/Map-my-World/blob/master/Output_Images/3D%20Map.PNG"
            alt="3D Map" width="400" height="400"></a>
           <br>3D Map
      </p>
    </th>
  </tr>
  <tr>
    <th><p>
           <img src="https://github.com/AnkushKansal/Map-my-World/blob/master/Output_Images/Keypoint%20Detection.PNG"
            alt="Keypoints Detection" width="400" height="400"></a>
           <br>Keypoints Detection
      </p>
    </th>
    <th><p>
           <img src="https://github.com/AnkushKansal/Map-my-World/blob/master/Output_Images/Keypoint%20Detection%20with%20Loop%20Closure.PNG"
            alt="Keypoint Detection with Loop Closure" width="400" height="400"></a>
           <br>Loop Closure
      </p>
    </th>
  </tr>
</table>

## Description
The project consists of the following parts:
1. A Gazebo world and a mobile robot from this [project](https://github.com/AnkushKansal/Map-my-World).
2. ROS package: [rtabmap-ros](http://wiki.ros.org/rtabmap_ros)

## Prerequisites
1. ROS (Melodic or Kinetic), Gazebo on Linux
2. CMake & g++/gcc
3. Install `rtabmap-ros` package `$ sudo apt-get install ros-${ROS_DISTRO}-rtabmap-ros`

## Build and Launch

1. Clone project and initialize a catkin workspace
```
$ mkdir catkin_ws && cd catkin_ws. mkdir src
$ git clone https://github.com/huuanhhuynguyen/RoboND-Map-My-World.git
$ mv Map-My-World src
$ cd src && catkin_init_workspace
```

2. Within the `catkin_ws/src` folder, clone the `teleop` project
```
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard
```

3. Move back to `catkin_ws\` and build
```
$ cd ..
$ catkin_make
```

4. Launch the world and robot
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

5. Open another terminal (Ctrl+Shift+T), and launch the `mapping.launch` file. 
Here, the rtabmap-ros package will be launched.
```
$ source devel/setup.bash
$ roslaunch my_robot mapping.launch
```

6. Open another terminal, and run the `teleop` node.
```
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

7. Click on this terminal, type keyboard to navigate the robot around. Navigate 
the robot to scan its surrounding environment. The rtabmap-ros package will save
the resulted map with the localized trajectory of the robot in a database file 
`~/.ros/rtabmap.db`.

8. Open another terminal, and open up the database file using `rtabmap-databaseViewer`
```
$ rtabmap-databaseViewer ~/.ros/rtabmap.db
```

* Choose View -> Constraints View and Graph View
* To see 3D Map, Choose Edit -> View 3D Map ...
    
## You could also open the database I already generated in this project on Google drive.
https://drive.google.com/file/d/1t26iNeIpGks3n3VmiPjC3fDNcfrjrOmo/view?usp=sharing
