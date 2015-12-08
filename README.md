Packages required:


labust-ros-pkg/labust_control (for velocity_control, depth_controller2)

labust-ros-pkg/labust_sim

labust-ros-pkg/snippets

labust-common-msgs


*TO RUN:*

-navigate to subcultron_launch/src/

>  python setup.py amussel_number apad_number [first_index_amussel] [first_index_apad] [north_min_am] [north_max_am] [east_min_am] [east_max_am] [north_min_ap] [north_max_ap] [east_min_ap] [east_max_ap]

> source ../data/simulation/simulation_config.bash

> roslaunch subcultron_launch standard_simulation.launch

-start scenario simulation:

> rostopic pub /scenario_start std_msgs/Bool "data: true"


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

*SAMPLE POSITION CONTROLLER ACTIVATION*

>rosservice call /apad1/FADP_enable "enable: true" 

>rosservice call /apad1/ConfigureVelocityController "ControllerName: 'FADP' 
desired_mode: [2, 2, 0, 0, 0, 0]" 

>rosservice call /apad1/VelCon_enable "enable: true" 

>rostopic pub /apad1/stateRef auv_msgs/NavSts "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
global_position: {latitude: 0.0, longitude: 0.0}
origin: {latitude: 0.0, longitude: 0.0}
position: {north: 10.0, east: 10.0, depth: 0.0}
altitude: 0.0
body_velocity: {x: 0.0, y: 0.0, z: 0.0}
gbody_velocity: {x: 0.0, y: 0.0, z: 0.0}
orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
orientation_rate: {roll: 0.0, pitch: 0.0, yaw: 0.0}
position_variance: {north: 0.0, east: 0.0, depth: 0.0}
orientation_variance: {roll: 0.0, pitch: 0.0, yaw: 0.0}
status: 0"