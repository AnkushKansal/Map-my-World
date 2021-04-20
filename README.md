# RTAB 3D SLAM
# Project 4: Map My World

An application of [rtabmap-ros](http://wiki.ros.org/rtabmap_ros) package for 
simultaneous localization and mapping (SLAM) of a mobile robot. 
This project is part of Udacity Robotics Software Engineer Nanodegree.

<table style="width:100%">
  <tr>
    <th><p>
           <img src="Output Images/Capture.png"
            alt="Initial State" width="400" height="200"></a>
           <br>Initial state
        </p>
    </th>
    <th><p>
           <img src="Output Images/Rviz 2D Map.png"
            alt="2D map" width="200" height="200"></a>
           <br>2D Map
      </p>
    </th>
  </tr>
  <tr>
    <th><p>
           <img src="Output Images/Keypoint Detection.png"
            alt="Keypoints Detection" width="400" height="200"></a>
           <br>Keypoints Detection
      </p>
    </th>
    <th><p>
           <img src="Output Images/Keypoint Detection with Loop Closure.png"
            alt="Loop Closure" width="200" height="200"></a>
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
    
You could also open the database I already generated in this project. The number
of loop closures can be found in [overview.png](Output Images/Keypoint Detection with Loop Closure).
