## ● Making new models requires 
  + Model making - mass, appearance(collision), inertia, motor specification
  + Gain tuning

<br>

### ● Making new model
  + Refer [here, official homepage](https://dev.px4.io/v1.9.0/en/airframes/adding_a_new_frame.html)
    + For a new airframe belonging to an existing group, you don't need to do anything more than provide documentation in the airframe description located at ROMFS/px4fmu_common/init.d
  + Refer [here](https://discuss.px4.io/t/create-custom-model-for-sitl/6700/4)
    + Create an **airframe file under ROMFS/px4fmu_common/init.d-posix/airframes**
      + give it a number and name it number_my_vehicle
    + Add it in the `CMakeLists.txt` in **ROMFS/px4fmu_common/init.d-posix/airframes**
    + Add the airframe name to the file **platforms/posix/cmake/sitl_target.cmake** in the command set -> models …
  + Now you are free to tweak any of the model values in the SDF file and any of the controller gains in the airframe file.
  + You can launch SITL with your model with 
  ~~~shell
  (when error) $ make clean
  
  $ make px4_sitl gazebo_my_vehicle
  ~~~
  1. When increasing the mass of the vehicle, also increase the inertias. I found that Gazebo does not like really small inertias.
  2. With the larger vehicle, tweak the motor values
  3. If the quad is unstable, it is probably due to bad controller gains. Tweak them for the larger vehicle.

<br>

### ● Motor
  + Need Thrust Curver using [this kind of device](https://www.banggood.com/Lantian-RC-6-40V-Multifuntional-Motor-ESC-Propeller-Tester-for-RC-Drone-p-1299859.html?cur_warehouse=CN)
  + Edit this part
~~~xml
    <plugin name='front_right_motor_model' filename='libgazebo_motor_model.so'>
      <robotNamespace/>
      <jointName>rotor_0_joint</jointName>
      <linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1100</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.06</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>0.000175</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/0</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
~~~
  + Refer [here](https://github.com/PX4/sitl_gazebo/issues/110)
~~~
Ω or Max Rotational Velocity = Kv * Max Applied Voltage * Max Motor Efficiency * 2π / 60
Rotor Drag Coefficient = Thrust / (ρ * (Kv * Max Applied Voltage * Max Motor Efficiency / 60) ² * Propeller diameter ⁴)
Rolling Moment Coefficient = Using SST turbulence model from reference, and bent/round wings/propellers: ~0.0220 for angle of attack smaller than 16 degrees 
-> I'm keeping this defaulted to 1E-06 for now.

where,
Kv [RPM/V]
Max Thrust [N]
Max Applied Voltage [V]
Ω Rotor angular velocity [rad/s]
ρ Air density at 20ºC: 1.2041 [kg/m³]
~~~
  + and [this](https://github.com/mvernacc/gazebo_motor_model_docs/blob/master/notes.pdf)
    + for *motor_constant* and *moment_constant*

### ● Gain tuning
  + refer [here](https://docs.px4.io/v1.9.0/en/config_mc/pid_tuning_guide_multicopter.html)
  + Use log file with [here](https://review.px4.io/)

### ● Airframe
  + [here](https://dev.px4.io/v1.9.0/en/airframes/airframe_reference.html)
