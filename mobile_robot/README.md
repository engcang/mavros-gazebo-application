## Cooperation with [Kobuki](http://kobuki.yujinrobot.com/), [kobuki wiki](http://wiki.ros.org/kobuki)
+ Installation
~~~shell
  $ sudo apt-get install pyqt5-dev-tools ros-melodic-yocs-* ros-melodic-ecl* ros-melodic-kobuki-ftdi ros-melodic-ar-track-alvar-msgs
  $ cd ~/workspace_src
  
  # git checkout -b melodic for every repos.
  $ git clone https://github.com/yujinrobot/kobuki_msgs
  $ git clone https://github.com/yujinrobot/kobuki_core
  $ sudo rm -r kobuki_core/kobuki_ftdi
  $ git clone https://github.com/yujinrobot/kobuki
  $ sudo rm -r kobuki/kobuki_auto_docking
  $ git clone https://github.com/yujinrobot/kobuki_desktop
~~~
+ Launch Gazebo with Kobuki
  + Include or launch ***kobuki_desktop/kobuki_gazebo/launch/includes/robot.launch.xml*** with world
  + Edit ***kobuki/kobuki_description/urdf/kobuki.urdf.xacro***
