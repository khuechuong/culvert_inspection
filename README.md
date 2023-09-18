# Data-Driven Autonomous Culvert Inspection Robot

- This work was done in the [Advance Robotics and Autonmation (ARA) Lab](https://ara.cse.unr.edu/) 

## Abstract
Culvert condition assessment is essential to maintaining roadways to ensure adequate road surface drainage and public safety since thousands of culverts are in the United States. Culvert deteriorates with time due to aging material, traffic load, and sometimes inadequate maintenance. Culvert maintenance relies on manual inspection, which has drawbacks, including risks to workers, time-consuming, high labor hours, and prone to human errors. In addition, to the author's best knowledge, only superficial methods (visual inspection) have been used for condition assessment. Non-destructive evaluation (NDE) methods are preferred in other civil structure inspections like bridges since they do not require inspection sites to be evacuated during the inspection and can offer an interior assessment of the structure. This paper presents a data-driven autonomous robotic exploration system equipped with visual and NDE's electrical resistivity (ER) sensors for a comprehensive culvert condition assessment. The system produces a 3D map highlighting defects (i.e. cracks and spalls) and an ER condition map highlighting corrosion. 

![alt text](https://github.com/khuechuong/culvert_inspection/blob/main/pic/flowchart.png)  

## Sofware used:
- Ubuntu 20.04 
- ROS Noetic

## Technology used:
- Rover Zero 3
- Zed Mini
- Linear Actuator & servos (for arm)
- Intel NUC Extreme 11
- LED lights
- ER sensor


## ROS Packages (and Modifications):
- [detection](https://github.com/khuechuong/culvert_inspection/tree/main/detection) contains our defect localization ROS implementation by fusing YOLOv8 model result with pointcloud (x,y,z). It subscribe both rbg and pointcloud map from zed.
- [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper) and [zed-ros-examples](https://github.com/stereolabs/zed-ros-examples) is the package we used for the zed mini.
- [explore_lite](https://github.com/hrnr/m-explore) is the 2D exploration package we used for navigation. Modification in [modified ROS code](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/modified%20ROS%20code) folder.
- [gmapping](https://github.com/ros-perception/slam_gmapping) is just a simple 2D mapping node we used for exploration. (sudo apt install ros-<distro>-gmapping)
- [rtabmap](https://github.com/introlab/rtabmap_ros) is the tool we used for 3D mapping. Our rtabmap launch is [rtabmap.launch](https://github.com/khuechuong/culvert_inspection/blob/main/robotic%20mod/launch/rtabmap.launch).
- [zed](https://github.com/khuechuong/culvert_inspection/tree/main/zed) folder contains the customized config for our zed mini.
- [robotic mod](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod) is our modification of ROS packages that we clone and used:
  - [arduino](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/arduino) shows our arduino file using rossarduino and rosserial to control our arm.
  - [config](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/config) shows our ROS navigation stack config.
  - [launch](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/launch) shows our launch file for launching ROS navigation stack,zed camera, rover zero 3, exploration, rosserial_node, 2d mapping, rtabmap.
  - [modified ROS code](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/modified%20ROS%20code) shows our changes made to the explore_lite package to our purpose from pure exploration to data-driven exploration by subscribing to the [detection]((https://github.com/khuechuong/culvert_inspection/tree/main/detection)) node results.
 
## Segmentation:
- [Segmentation](https://github.com/khuechuong/culvert_inspection/tree/main/Segmentation) shows our semantic segmentation we we cloned from [Image Segmentation Keras](https://github.com/divamgupta/image-segmentation-keras) and modified to our need. [Code Guide.docx](https://github.com/khuechuong/culvert_inspection/blob/main/Segmentation/Code%20Guide.docx) contains instruction on how to run it.

## Run:

launch both:

```cpp
roslaunch zed_nav.launch
```
and 
```cpp
roslaunch rtabmap.launch
```

or just run 
```cpp
roslaunch zed_nav.launch
```
save data and post-process it.

**Note: Running rtabmap.launch gives better quality and accuracy since it uses rtabmap_odom.

## Contact:
- [Chuong Le](mailto:cle@nevada.unr.edu)
- [Hung La](mailto:hla@unr.edu)
