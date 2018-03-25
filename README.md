# Object Detection with MMW and Vision in Bad Weather

## Overview

haze weather is the most frequently reported harsh traffic weather, as haze affects the driver's field of vision and causes the situations of the drivers almost unclear the lane and vehicle, and finally leads to incorrect driving decisions. there are two main parts of this project:
* Haze removal by dark channel and down sampling methods
* Object detection using MMW and other sensors

## Dependencies
* Kinetic of ROS
* Opencv2.4

## Sensors
* Millimeter Wave Radar: Delphi ESR
* IMU: JY-901
* SEC: 315
* Monocular Camera
* GPS: Trimble BD910

## Results

### 1:Haze Removal
![image](https://github.com/dongdonghy/ADAS_Vision_MMW/raw/master/outputs/haze_removal.jpg)

### 2:Rainy and Foggy Weather
![image](https://github.com/dongdonghy/ADAS_Vision_MMW/raw/master/outputs/rainy_haze.png)

### 3:Night
![image](https://github.com/dongdonghy/ADAS_Vision_MMW/raw/master/outputs/night.png)
