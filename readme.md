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

## Test Navigation Under Namespace

### 1. Clone the robot launch project into local ROS workspace

```
roscd & cd ..
git clone https://github.com/wdxpz/launch_rosbot.git
```

### 2. Launch the map server

* if we run the test on the robot, make sure to change `ROS_MASTER_URI` to local IP in `.bashrc`

```
export ROS_MASTER_URI=http://local_ip:11311
```

* now, we can start the map server 

```
cd path-to-project-launch_rosbot/launch
roslaunch map_server.launch
```

**remember to change the path to map file in `map_server.launch` at first**

```
<launch>
    <arg name="map_file" default="$(find package_where_holds_maps)/sub_path_to_maps/map.yaml"/>       <!-- path of map file -->
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" respawn="true" />
</launch>
```

* Launch localization and navigation under namespace

```
cd path-to-project-launch_rosbot/launch
roslaunch launch_rosbot_with_name.launch
```

* Test the navigation

now, we can finally test the navigation under namespace with the scripts:

```
cd path-to-project-launch_rosbot/scripts
python goto_m.py
```

after that, the robot will navigate to the points defined in [line 34](https://github.com/wdxpz/launch_rosbot/blob/607314dfc22e2963f6537b9507c5ca5e84bf60b6/scripts/goto_m.py#L34) one by one, we can also modify this [line 34](https://github.com/wdxpz/launch_rosbot/blob/607314dfc22e2963f6537b9507c5ca5e84bf60b6/scripts/goto_m.py#L34)  in goto_m.py to change the target positions.

