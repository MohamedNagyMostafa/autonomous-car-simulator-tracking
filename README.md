# Autonomous Car Simulator with DFR-FastMOT Tracker

This repository contains packages of other repositories. **Thank you to all contributors for completing this work, particularly CatVehicle Simulator developers**.


[![DFR-FastMOT: 2D/3D Tracking via Gazeb/ROS Using YOLOv3](https://img.youtube.com/vi/abcdef12345/0.jpg)](https://youtu.be/QWIZmrVTaWw)


## How to use

This work is tested on the following specifications:
* Ubuntu: 20.04
* cmake: 3.0.2


  
Some dependencies need to be installed to guarantee running the packages without issues. 

```bash
sudo apt-get install python-yaml
sudo apt-get install ros-noetic-controller-manager \
ros-noetic-ros-control ros-noetic-ros-controllers \
ros-noetic-gazebo-ros-control libpcap-dev ros-noetic-velodyne
```

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone --recursive https://github.com/MohamedNagyMostafa/autonomous-car-simulator-tracking/
cd ..
catkin_make
source devel/setup.bash
```

## Get started

(1) You initially need to run the Catvehicle simulator.

```bash
roslaunch catvehicle catvehicle_neighborhood.launch
```

(2) You can also run gazebo and rviz with a pre-defined configuration

In a new terminal, launch gazebo:
```bash
gzclient
```

In another terminal, launch rviz:
```bash
rviz -d ./src/autonomous-car-simulator-tracking/catvehicle/config/catvehicle.rviz
```

You should see Gazebo and Rviz windows as follows:

![Screenshot from 2023-09-18 09-58-12](https://github.com/MohamedNagyMostafa/autonomous-car-simulator-tracking/assets/20774864/4047721a-f3a9-4881-8f4a-fa2955442360)


(3) To run the DFR-FastMot tracker, run the following line in a new terminal:

```bash
source devel/sectup.bash
rosrun dfr-fastmot tracking-data
```

This command should open two windows: a window for the Camera sensor stream and another window that displays LiDAR point cloud.

![Screenshot from 2023-09-18 10-01-01](https://github.com/MohamedNagyMostafa/autonomous-car-simulator-tracking/assets/20774864/dd36a725-dabe-4589-9189-0b2568b479e3)

You may add a car/truck model in front of the Camera to test the tracker.


![prjection](https://github.com/MohamedNagyMostafa/autonomous-car-simulator-tracking/assets/20774864/87e0fdd3-8f47-417c-b4d0-4a281259d291)


## To modify

You may follow these steps to integrate the tracker into another simulator or real car.

### (1) Object Type (YOLOv3):

The tracker utilizes YOLOv3 to perform 2D detection and projects the point cloud into the 2D image to obtain a 3D bounding box on LiDAR. 
To pick the desired objects, you must update the following line in `main.cpp`, line 62. You can remove or add new classes to the detector. 
```c++
const vector<ObjectDetection::Class> DETECTION_CLASSES  = {  ObjectDetection::Class::CAR, ObjectDetection::Class::TRUCK};
```

Feel free to add new classes to the detector in `ObjectDetection.h` file.

```c++
    /**
     * @enum @Class classes considered in object detection module.
     */
    enum Class
    {
        PERSON,
        BYCYCLE,
        CAR,
        MOTORBIKE,
        BUS =5,
        TRUCK = 7
    };
```

### (2) Projection.

Since the tracker uses only 2D detector, it projects the point cloud to obtain 3D detection. You may need to change the Camera/LiDAR frame based on the targeted system.
Modify the source/target frame in lines 180-186. In this case, the Camera frame is `catvehicle/triclops_link`, and the LiDAR frame is `catvehicle/velodyne_link`.

```c++
    if(!tfListener->waitForTransform("catvehicle/triclops_link", "catvehicle/velodyne_link",ros::Time(), ros::Duration(1.0)))
    {
        ROS_WARN("Transform error");
        return;
    }

    pcl_ros::transformPointCloud("catvehicle/triclops_link", *pointClouds, *transformed_cloud, *tfListener);

```

Once the projection succeeds, you must transform the point cloud into The Camera image plane using the Camera's intrinsic parameters. In the same file (lines 173-174), modify the Camera focal length, cx, and cy.

```c++
double fx = 762.7249337622711;
double fy = 762.7249337622711;
double cx = 640.5;
double cy = 480.5;
....
pt.x = ((*it).x / (*it).z) * 1 + cx; // replace 1 by the fx
pt.y = ((*it).y / (*it).z) * 1 + cy;// replace 1 by the fy
```

### (3) Reading from sensors.

You should replace the Camera data stream source with the desired sensor's topic inside `ImageStream.cpp` line 9. 

```c++
this->cameraSensorSubscriber  = nodeHandle.subscribe("/catvehicle/triclops/triclops/left/image", 10, & ImageStream::cameraSubscriberCallback, this);
``` 

Do the same for the LiDAR sensor inside `LidarStream.cpp` line 8.

```c++
liderSubscriber = nodeHandle.subscribe("/catvehicle/lidar_points", 10, &LidarStream::lidarCallback, this);
```

## Citation

If you found this repository useful, kindly cite our paper.

```latex
@INPROCEEDINGS{10160328,
  author={Nagy, Mohamed and Khonji, Majid and Dias, Jorge and Javed, Sajid},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={DFR-FastMOT: Detection Failure Resistant Tracker for Fast Multi-Object Tracking Based on Sensor Fusion}, 
  year={2023},
  volume={},
  number={},
  pages={827-833},
  doi={10.1109/ICRA48891.2023.10160328}}
```

## Contact

Feel free to contact me for any clarification. mohamed.nagy@ieee.org
