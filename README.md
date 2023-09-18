# Data-Driven Autonomous Culvert Inspection Robot

- This work was done in the [Advance Robotics and Autonmation (ARA) Lab](https://ara.cse.unr.edu/) 

## Abstract
Culvert condition assessment is essential to maintaining roadways to ensure adequate road surface drainage and public safety since thousands of culverts are in the United States. Culvert deteriorates with time due to aging material, traffic load, and sometimes inadequate maintenance. Culvert maintenance relies on manual inspection, which has drawbacks, including risks to workers, time-consuming, high labor hours, and prone to human errors. In addition, to the author's best knowledge, only superficial methods (visual inspection) have been used for condition assessment. Non-destructive evaluation (NDE) methods are preferred in other civil structure inspections like bridges since they do not require inspection sites to be evacuated during the inspection and can offer an interior assessment of the structure. This paper presents a data-driven autonomous robotic exploration system equipped with visual and NDE's electrical resistivity (ER) sensors for a comprehensive culvert condition assessment. The system produces a 3D map highlighting defects (i.e. cracks and spalls) and an ER condition map highlighting corrosion. 

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

## Package used:
- We use many packages like zed, roverrobotic_driver, ros_numpy, explore_lite.

## Packages (and Modifications):
- [detection](https://github.com/khuechuong/culvert_inspection/tree/main/detection) contains our defect localization ROS implementation by fusing YOLOv8 model result with pointcloud (x,y,z).
- [zed](https://github.com/khuechuong/culvert_inspection/tree/main/zed) shows our zed configuration
- [robotic mod](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod) shows our robotic config:
  - [arduino](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/arduino) shows our arduino file using rossarduino and rosserial to control our arm.
  - [config](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/config) shows our ROS navigation stack config
  - [launch](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/launch) shows our launch file for launching ROS navigation stack,zed camera, rover zero 3, exploration, rosserial_node, 2d mapping, rtabmap.
  - [modified ROS code](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/modified%20ROS%20code) shows our changes made to the explore_lite package to our purpose. 

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


## Contact:
- [Chuong Le](mailto:cle@nevada.unr.edu)
- [Hung La](mailto:hla@unr.edu)
