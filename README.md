# Data-Driven Autonomous Culvert Inspection Robot


## Sofware used:
- Ubuntu 20.04 
- ROS Noetic

## Technology used:
- Rover Zero 3
- Zed Mini
- Linear Actuator & servos (for arm)
- Intel NUC Extreme 11

## Package used:
- We use many packages like zed, roverrobotic_driver, ros_numpy, explore_lite.

## Packages (and Modifications):

- [detection](https://github.com/khuechuong/culvert_inspection/tree/main/detection) contains our YOLOv8 ROS implementation for defect localization.

- [zed](https://github.com/khuechuong/culvert_inspection/tree/main/zed) shows our zed configuration

- [robotic mod](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod) shows our robotic config:
  - [arduino](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/arduino) shows our arduino file using rossarduino and rosserial to control our arm.
  - [config](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/config) shows our ROS navigation stack config
  - [launch](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/launch) shows our launch file for launching ROS navigation stack,zed camera, rover zero 3, exploration, rosserial_node, 2d mapping.
  - [modified ROS code](https://github.com/khuechuong/culvert_inspection/tree/main/robotic%20mod/modified%20ROS%20code) shows our changes made to the explore_lite package to our purpose. 

