# Object Detection with MMW and Vision in Bad Weather

## Overview

haze weather is the most frequently reported harsh traffic weather, as haze affects the driver's field of vision and causes the situations of the drivers almost unclear the lane and vehicle, and finally leads to incorrect driving decisions. there are two main parts of this project:
* Haze removal by dark channel and down sampling methods
* Object detection using MMW and other sensors

## Dependencies
* Kinetic of ROS
* Opencv2.4

## Sensors
* ESR MMW
* IMU
* SEC
* Monocular Camera
* GPS

## Results

### Haze Removal
![image](https://github.com/dongdonghy/ADAS_Vision_MMV/raw/master/outputs/haze_removal.jpg)

### rainy and foggy weather
![image](https://github.com/dongdonghy/ADAS_Vision_MMV/raw/master/outputs/rainy_haze.png)

### Night
![image](https://github.com/dongdonghy/ADAS_Vision_MMV/raw/master/outputs/night.png)
