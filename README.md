Packages required:


labust-ros-pkg/labust_control (for velocity_control, depth_controller2)

labust-ros-pkg/labust_sim

labust-ros-pkg/snippets

labust-common-msgs




*TO RUN:*

-navigate to amussel/src/

> python aMussel_random_generator.py aMussel_number [north_min] [north_max] [east_min] [east_max]

-navigate to amussel/launch/simulation/

> source ../../data/simulation/simulation_config.bash

> roslaunch amussel_standard_simulation.launch


*SAMPLE DEPTH CONTROLLER ACTIVATION*

>rosservice call /amussel1/DEPTH_enable "enable: true" 


>rosservice call /amussel1/VelCon_enable "enable: true" 


>rosservice call /amussel1/ConfigureVelocityController "ControllerName: 'DEPTH'
desired_mode: [0, 0, 2, 0, 0, 0]" 


>rostopic pub /amussel1/stateRef auv_msgs/NavSts "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
global_position: {latitude: 0.0, longitude: 0.0}
origin: {latitude: 0.0, longitude: 0.0}
position: {north: 0.0, east: 0.0, depth: 5.0}
altitude: 0.0
body_velocity: {x: 0.0, y: 0.0, z: 0.0}
gbody_velocity: {x: 0.0, y: 0.0, z: 0.0}
orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
orientation_rate: {roll: 0.0, pitch: 0.0, yaw: 0.0}
position_variance: {north: 0.0, east: 0.0, depth: 0.0}
orientation_variance: {roll: 0.0, pitch: 0.0, yaw: 0.0}
status: 0"