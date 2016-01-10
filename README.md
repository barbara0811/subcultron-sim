subCULTron project trust scenario implementation.

*TO RUN:*

- navigate to subcultron_launch/src/

>  python setup.py apad_number afish_number amussel_number [first_index_apad] [first_index_afish] [first_index_amussel] [north_min] [north_max] [east_min] [east_max]

> source ../data/simulation/simulation_config.bash

> roslaunch subcultron_launch standard_simulation.launch

- start scenario simulation:

> rostopic pub /scenario_start std_msgs/Bool "data: true"

- change noise source amplitude (setting a negative number turns the source off):

> rostopic pub /noise_intensity std_msgs/Float64 "data: 10"
> rostopic pub /noise_intensity std_msgs/Float64 "data: -1"
