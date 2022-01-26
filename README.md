# MAVROS-Gazebo simulation(SITL) build and application + mavros setup (also applicable for real drones)
+ Need Ubuntu with ROS installed
+ Simple [Joystick](https://asia.playstation.com/ko-kr/accessories/dualshock4/) Controller code implemented

## Result Clips - [1](https://youtu.be/5t-6g7UWA7o),  [2](https://youtu.be/t3ysB8Y_GCU),  [3](https://youtu.be/6K-QSb1Aq5E),  [4](https://youtu.be/yjt3zLSrg6o),  [5](https://youtu.be/ifB_i0f9hj4)

<br>
<br>

## Index
### 1. Installation - [here](#installation)
### 2. Joystick controller code - [here](#mission--joystick-controller---supports-kobuki-and-jackal)
### 3. Fly without GPS - [here](#fly-without-gps---result-clip-here)
### 4. Change IMU rate for real drones / SITL - [here](#change-imu-rate-for-actual-drones) / [here](#change-imu-rate-for-sitl)
### 5. Making new PX4 model(airframe) for SITL - [here](https://github.com/engcang/mavros-gazebo-application/tree/master/px4_model_making)
### 6. Making new Gazebo world and Gazebo model - [here](https://github.com/engcang/mavros-gazebo-application/tree/master/world_making)
### 7. Use mobile robots(Kobuki, Jackal) for Gazebo - [here](https://github.com/engcang/mavros-gazebo-application/tree/master/mobile_robot)

<br><br>

### Installation 
+ Mavros and requirements
~~~shell
    $ sudo apt-get install ros-<distro>-mavros ros-<distro>-mavros-extras
    $ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    $ chmod +x install_geographiclib_datasets.sh
    $ sudo ./install_geographiclib_datasets.sh
    $ cd ~/ && git clone https://github.com/PX4/Firmware.git --recursive
    
    $ source Firmware/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
    $ sudo apt-get install python-catkin-tools python-numpy python3-pip python3-numpy python3-empy python3-toml python3-packaging python3-jinja2 gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev
~~~
+ To run, set up the path and environments
~~~shell
    $ cd ~/Firmware
    
    # important!!
    $ sudo apt install ros-melodic-gazebo-plugins
    
    $ export LANG=C.UTF-8
    $ export LC_ALL=C.UTF-8
    $ DONT_RUN=1 make px4_sitl_default gazebo

    # when undefined iginition error (gazebo: symbol lookup error: /usr/lib/x86_64-linux-gnu/libgazebo_common.so.9: undefined symbol: ZN8ignition10fuel_tools12ClientConfi..........
    # $ sudo apt upgrade libignition-math4
    
    $ source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
    $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
    $ roslaunch px4 mavros_posix_sitl.launch
~~~
+ To remember setup,
~~~shell
    $ gedit ~/.bashrc
    # Copy - past below and save, (path should be edited)
    #PX4, mavros
    export GAZEBO_PLUGIN_PATH=/home/<PCname>/Firmware/build/px4_sitl_default/build_gazebo
    export GAZEBO_MODEL_PATH=/home/<PCname>/.gazebo/models:/home/<PCname>/Firmware/Tools/sitl_gazebo/models
    export LD_LIBRARY_PATH=/home/<PCname>/catkin_ws/devel/lib:/opt/ros/melodic/lib:/home/<PCname>/Firmware/build/px4_sitl_default/build_gazebo

    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/<PCname>/Firmware
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/<PCname>/Firmware/Tools/sitl_gazebo
~~~

<br><br>

### Setting faster simulator
+ Referred this [viedeo](https://youtu.be/1Bs98kOwuK4)
~~~shell
    $ export PX4_SIM_SPEED_FACTOR=<how many times faster you want>
~~~

<br><br>

### Execution
+ To run Gazebo with default setup, with ROS
~~~shell
    $ roslaunch px4 mavros_posix_sitl.launch
~~~
+ To control with simple square path,
~~~shell
    $ git clone <this repository>
    $ python <clone directory>/python_offb.py
~~~

<br>

## Mission / Joystick Controller - supports [Kobuki and Jackal](https://github.com/engcang/mavros-gazebo-application/tree/master/mobile_robot)
+ To use the other model with sensors => edit "models" param in mavros_posix.sitl.launch file
+ use Joystick to manually control -> it supports **mode 1,2** and **Position/Rate control**
    + Velocity/Attitude control are not supported anymore due to their error.
~~~shell
    $ git clone <this repository>
    $ python <clone directory>/scripts/mavors_joy_controller.py
~~~

<br><br>

## From here, configs can be used to real drone

<br>

## Fly without GPS - result clip [here](https://engcang.github.io/mavros_vision_pose/)
#### Visual Inertial Odometry (VIO) · PX4 User Guide - [here](https://docs.px4.io/master/en/computer_vision/visual_inertial_odometry.html)
### T265 Camera for PX4
+ Guide [here](https://docs.px4.io/master/en/computer_vision/visual_inertial_odometry.html)
+ ***tf*** is needed when attached with different direction
    + edit *launch* file
~~~xml
    <node pkg="tf" type="static_transform_publisher" name="tf_baseLink_cameraPose" args="0 0 0 0 1.5708 0 t265_pose_frame test 1000"/>
    <!-- when T265 is attached upward -->
~~~
<br>

### VINS-Fusion for PX4 with Masking [here](https://github.com/engcang/vins-application/tree/master/vins-fusion-px4)
+ check [here](https://dev.px4.io/v1.9.0/en/ros/external_position_estimation.html)
+ Publish vision data e.g. VIO as **geometry_msgs/PoseStamped** type message into topic **/mavros/vision_pose/pose**
+ Change **EKF2_AID_MASK** parameter at QGroundControl
  + Uncheck **use GPS**
  + Check **vision position fusion** and **vision yaw fusion**, consider **vision velocity fusion** too.
+ Change **EKF2_HGT_MODE** parameter at QGC
  + Set to Vision or Range (1D LiDAR, Altitude laser sensor)
+ Change **EKF2_EV_POS_X, EKF2_EV_POS_Y, EKF2_EV_POS_Z** at QGC
  + ***tf body_T_cam***

<br>

+ If using 1D LiDAR altitude sensor, Check
  + **EKF2_HGT_MODE**, **EKF2_RNG_AID**, and **SENS_TFMINI_CFG**
  + Also check **EKF2_RNG_A_HMAX** and **EKF2_RNG_NOISE**
  
<br>


<br><br>


### Add user to dialout group
+ To use /dev/ttyACM as root admission
~~~shell
  # check the groups current user is engaged in
  $ id -Gn
  # add to dialout group
  $ sudo adduser $USER dialout
~~~

<br>

### Change IMU rate for actual Drones
+ **/fs/microsd/etc/extras.txt** file from QGC's nutshell
+ Or place **etc/extras.txt** file **in the SD card** of Pixhawk as **below** or as [this file](https://github.com/engcang/mavros-gazebo-application/blob/master/extras.txt)
+ add *HIGHRES_IMU* for imu/data_raw and *ATTITUDE* for imu/data
~~~shell
  #!nsh
  mavlink start -b 921600 -d /dev/ttyACM0 -m onboard
  mavlink stream -d /dev/ttyACM0 -s PARAM_VALUE -r 200
  usleep 100000
  mavlink stream -d /dev/ttyACM0 -s NAMED_VALUE_FLOAT -r 10
  usleep 100000
  mavlink stream -d /dev/ttyACM0 -s OPTICAL_FLOW -r 50
  usleep 100000
  mavlink stream -d /dev/ttyACM0 -s ATTITUDE -r 150
  usleep 100000
  mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 150
  usleep 100000
  mavlink stream -d /dev/ttyACM0 -s LOCAL_POSITION_NED -r 10
  usleep 100000
~~~
+ edit **mavros/px4.launch** file to same *baudrate*
~~~xml
  <arg name="fcu_url" default="/dev/ttyACM0:921600" />
~~~

<br>
<br>

### Change IMU rate for SITL
+ for ***ROS Topics***
    + Edit the rates in **Firmware/ROMFS/px4fmu_common/init.d-posix/rcS** as [here](https://github.com/engcang/mavros-gazebo-application/blob/master/rcS#L290)
    + and then rebuild the **Firmware** with `$ DONT_RUN=1 make px4_sitl_default gazebo`
    + or simply edit the rates in **Firmware/build/px4_sitl_default/etc/init.d-posix/rcS** as [here](https://github.com/engcang/mavros-gazebo-application/blob/master/rcS#L290)
    ~~~shell
    # API/Offboard link
    mavlink start -x -u $udp_offboard_port_local -r 4000000 -m onboard -o $udp_offboard_port_remote
    mavlink stream -r 100 -s LOCAL_POSITION_NED -u $udp_offboard_port_local
    mavlink stream -r 100 -s HIGHRES_IMU -u $udp_offboard_port_local
    mavlink stream -r 100 -s ATTITUDE -u $udp_offboard_port_local
    mavlink stream -r 100 -s ATTITUDE_QUATERNION -u $udp_offboard_port_local
    ~~~
+ for ***QGC*** Mavlink Inspector
    + Edit the same **Firmware/ROMFS/px4fmu_common/init.d-posix/rcS** as [here](https://github.com/engcang/mavros-gazebo-application/blob/master/rcS#L275)
    + and then rebuild the **Firmware** with `$ DONT_RUN=1 make px4_sitl_default gazebo`
    + or simply edit the rates in **Firmware/build/px4_sitl_default/etc/init.d-posix/rcS** as [here](https://github.com/engcang/mavros-gazebo-application/blob/master/rcS#L290)
    ~~~shell
    # GCS link
    #################### -> This block is for Mavlink in QGC
    mavlink start -x -u $udp_gcs_port_local -r 4000000
    mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u $udp_gcs_port_local
    mavlink stream -r 100 -s LOCAL_POSITION_NED -u $udp_gcs_port_local
    mavlink stream -r 50 -s GLOBAL_POSITION_INT -u $udp_gcs_port_local
    mavlink stream -r 100 -s HIGHRES_IMU -u $udp_gcs_port_local
    mavlink stream -r 100 -s ATTITUDE -u $udp_gcs_port_local
    mavlink stream -r 100 -s ATTITUDE_QUATERNION -u $udp_gcs_port_local
    mavlink stream -r 100 -s ATTITUDE_TARGET -u $udp_gcs_port_local
    mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u $udp_gcs_port_local
    mavlink stream -r 20 -s RC_CHANNELS -u $udp_gcs_port_local
    mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u $udp_gcs_port_local
    ~~~

    + Or type below as [here](https://zhuanlan.zhihu.com/p/33075247)
    ~~~shell
      $ mavlink stream -u 14557 -s ATTITUDE -r 200
      $ mavlink stream -u 14557 -s HIGHRES_IMU -r 200
      $ mavlink stream -u 14557 -s LOCAL_POSITION_NED -r 200
    ~~~
