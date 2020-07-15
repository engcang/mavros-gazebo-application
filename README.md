# MAVROS-Gazebo simulation build and application
+ Need Ubuntu with ROS installed
+ Simple [Joystick](https://asia.playstation.com/ko-kr/accessories/dualshock4/) Controller code implemented

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
+ use Joystick to manually control
+ simple rectangular trajectory following code implemented
