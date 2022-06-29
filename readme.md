# Localization and Navigation of Rosbot Under Namespace

## Upgrade ROS Kinetci to Melodic

### 1. Install ROS Melodic

Please following the official guidance for [ROSbot 2.0](https://husarion.com/manuals/rosbot/#rosbot-20) or [ROSbot 2.0 Pro](https://husarion.com/manuals/rosbot/#rosbot-20-pro) to update ROS framework in the robot to version Melodic. 

Notes for Installation

* Here is the [URL](https://robot-os-images.s3.eu-central-1.amazonaws.com/ros-melodic-arm-2022-03-06.img) to download the image.

* **For ROSbot 2.0 PRO, make sure you have right BIOS settings as [official warning](https://husarion.com/downloads/#rosbot-pro)**
* **After burning the image to robot, make sure to Flash the firmware for STM32 microcontroller**

### 2. Update Package `rosbot_ekf`

* On robot, update package `rosbot_ekf` with the latest version [github url](https://github.com/husarion/rosbot_ekf.git)

```
roscd rosbot_ekf
cd .. & mv rosbot_ekf rosbot_ekf_bak
git clone https://github.com/husarion/rosbot_ekf.git
```

* change topics name by modifying `msgs_conversion.cpp`

```
roscd rosbot_ekf/src
mv msgs_conversion.cpp msgs_conversion.cpp.bak
# replace msgs_conversion.cpp with https://github.com/wdxpz/launch_rosbot/blob/main/rosbot_ekf/src/msgs_conversion.cpp
curl https://raw.githubusercontent.com/husarion/rosbot_ekf/master/src/msgs_conversion.cpp --output msgs_conversion.cpp
```

* compile the package

```
cd ../../..
catkin_make -DCATKIN_WHITELIST_PACKAGES="rosbot_ekf"
```

remember to restore the compile option to all pakcage by setting `-DCATKIN_WHITELIST_PACKAGES=""`

### 3. Update Package `rplidar_ros` (optional)

download and update pakcage ['rplidar_ros'](https://github.com/Slamtec/rplidar_ros.git)

```
roscd rplidar_ros
cd .. & mv rplidar_ros rplidar_ros_bak
git clone https://github.com/Slamtec/rplidar_ros.git
cd ../../..
catkin_make -DCATKIN_WHITELIST_PACKAGES="rplidar_ros"
```

