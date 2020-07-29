## ● Making world
+ using .world
+ using .sdf files [format](http://sdformat.org/spec?ver=1.7&elem=sdf)
+ using .dae files from [SketchUp data](https://3dwarehouse.sketchup.com) and [SketchUp](https://app.sketchup.com/app?hl=en)
+ using Gazebo tutorials

<br>

### ● Make new model with .dae
+ download .dae file from [SketchUp data](https://3dwarehouse.sketchup.com)
+ And edit it, as wanted [SketchUp](https://app.sketchup.com/app?hl=en)
+ remove ***lines***
+ make new folders including ***.config***, ***.sdf***
+ .sdf file should be like below
~~~xml
<?xml version="1.0" ?>
<sdf version='1.5'>
  <model name='tank1'>
    <static>true</static>
      <link name='tank1'>
        <pose frame=''>0 0 0 0 0 0</pose>
         <collision name='tank1'>
          <geometry>
            <mesh>
              <uri>model://tank1/model.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='tank1'>
          <geometry>
            <mesh>
              <scale>1 1 1</scale>
              <uri>model://tank1/model.dae</uri>
            </mesh>
          </geometry>
          <material>
            <script>
              <name>tank1</name>
              <uri>model://tank1/model.dae</uri>
            </script>
          </material>
        </visual>
      </link>
  </model>
</sdf>
~~~

<br>

### ● Making new model combining two or more models
~~~xml
<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='iris_fpv_cam'>
    <include>
      <uri>model://iris</uri>
    </include>
    <include>
      <uri>model://fpv_cam</uri>
      <pose>0 0 0 0 0.26179938779851 0</pose>
    </include>
    <joint name="fpv_cam_joint" type="fixed">
      <child>fpv_cam::robot_camera_link</child>
      <parent>iris::base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
~~~
+ Publish joint as ***ROS tf*** using the relationship below
  <p align="left">
  <img src="https://github.com/engcang/mavros-gazebo-application/blob/master/world_making/tf.png" width="300"/>
  </p>
