## Exploration, Active SLAM
+ Mainly used RGB-D camera Intel Realsense D435i
+ Installation : [here](https://github.com/engcang/VINS-application/tree/Intel-D435i#-necessary-for-ros-too)
+ Tuning Guide : [here1](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/BKMs_Tuning_RealSense_D4xx_Cam.pdf) [here2](https://github.com/IntelRealSense/librealsense/issues/3139)
<br>
<br>

### ● Prerequsties
+ Octomap
~~~shell
  $ sudo apt install ros-<distro>-octomap* -y
  $ roslaunch octomap_server mapping.launch
~~~
+ RGB-D Sensor or LiDAR sensor in Gazebo (ROS compatible)
  + Used Kinect in my case, [made one as here](http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros)

<br>
<br>

### 1. [MBP](https://github.com/unr-arl/mbplanner_ros) from Kostas Alexsis
+ **Important!!** -> when having **PX4 SITL**, make sure to backup one **.so** file
  + **librotors_gazebo_multirotor_base_plugin.so** file needs proper **libmav_msgs.so** file
~~~shell
$ cd Firmware/build/px4_sitl_default/build_gazebo
$ mv libmav_msgs.so libmav_msgs.so.backup
~~~
+ When try to use it in custom **.world**, do not forget to include **rotors_gazebo_ros_interface_plugin**
~~~xml
<sdf version='1.6'>
  <world name='blah blah'>
   ...
   <plugin name="ros_interface_plugin" filename="librotors_gazebo_ros_interface_plugin.so"/>
   ...
  </world>
</sdf>
~~~

<br>
