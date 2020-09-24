# MAVROS-Gazebo simulation build and application
+ Need Ubuntu with ROS installed
+ Simple [Joystick](https://asia.playstation.com/ko-kr/accessories/dualshock4/) Controller code implemented

## For real Drone - [here](#from-here-configs-can-be-used-to-real-drone)

<br><br>

### Installation 
+ Mavros and requirements
~~~shell
    $ sudo apt-get install ros-<distro>-mavros ros-<distro>-mavros-extras
    $ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    $ ./install_geographiclib_datasets.sh
    $ cd ~/ && git clone https://github.com/PX4/Firmware.git
    $ source Firmware/Tools/setup/ubuntu.sh
    # if above makes errors, just run below instead
    $ sudo apt-get install python-catkin-tools python-numpy python3-pip python3-numpy python3-empy python3-toml python3-packaging python3-jinja2
~~~
+ To run, set up the path and environments
~~~shell
    $ cd ~/Firmware
    
    # important!!
    $ sudo apt install ros-melodic-gazebo-plugins
    
    $ DONT_RUN=1 make px4_sitl_default gazebo
    
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


### Mission
+ To use the other model with sensors => edit "models" param in mavros_posix.sitl.launch file
+ use Joystick to manually control -> it supports **mode 1,2** and **Velocity/Attitude/Rate control**
~~~shell
    $ git clone <this repository>
    $ python <clone directory>/scripts/mavors_joy_controller.py
~~~

<br><br>

## From here, configs can be used to real drone

<br>

### Use without GPS
+ check [here](https://dev.px4.io/v1.9.0/en/ros/external_position_estimation.html)
+ Publish vision data e.g. VIO as **geometry_msgs/PoseStamped** type message into topic **/mavros/vision_pose/pose**
+ Change **EKF2_AID_MASK** parameter at QGroundControl
  + Uncheck *use GPS*
  + Check *vision position fusion* and *vision yaw fusion*
+ Change **EKF2_HGT_MODE** parameter at QGC
  + Set to Vision or 1D LiDAR
+ Change **EKF2_EV_POS_X, EKF2_EV_POS_Y, EKF2_EV_POS_Z** at QGC
  + ***tf body_T_cam***

<br>

### Add user to dialout group
+ To use /dev/ttyACM as root admission
~~~shell
  # check the groups current user is engaged in
  $ id -Gn
  # add to dialout group
  $ sudo adduser $USER dialout
~~~

<br>

### Change IMU rate
+ **/fs/microsd/etc/extras.txt** file from QGC's nutshell
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
+ For SITL Gazebo simulation, Type below as [here](https://zhuanlan.zhihu.com/p/33075247)
~~~shell
  $ mavlink stream -u 14557 -s ATTITUDE -r 200
  $ mavlink stream -u 14557 -s HIGHRES_IMU -r 200
  $ mavlink stream -u 14557 -s LOCAL_POSITION_NED -r 200
~~~
