subCULTron project trust scenario implementation.

*HOW TO DEFINE NEW SCENARIO STRUCTURES:*

- Open /subcultron_launch/data/scenario/scenario_spec.xml and define basic structures (as specified for other scenarios)

- Controller files are specified in \<vehicle\>/src/, e.g. amussel/src/controller_for_scenario_one.py

- Simulation specification files are located in \<vehicle\>/data/simulation/scenario/, e.g. amussel/data/simulation/scenario/simulation_scenario_one.xml. To generate simulation specification file, simply copy standard_simulation.xml file and modify it to meet specific scenario's needs.

*TO RUN:*

- Navigate to subcultron_launch/src/

> roscd subcultron_launch/src

- Prepare the simulation structures (has to be done only once for each configuration):

>  python setup.py scenario_name apad_number afish_number amussel_number [first_index_apad] [first_index_afish] [first_index_amussel] [north_min] [north_max] [east_min] [east_max]

> source ../data/simulation/simulation_config.bash

- In separate terminals, run:

> roscore

> roslaunch subcultron_launch standard_simulation.launch 

To stop the simulation, simply press ctrl-c in the second terminal (roscore can be left running at all times).

- Start scenario simulation:

> rostopic pub /scenario_start std_msgs/Bool "data: true"

- Change noise source amplitude (setting a negative number turns the source off, a positive number attracts all aFishes in the area of noise source):

> rostopic pub /noise_intensity std_msgs/Float64 "data: 10"

> rostopic pub /noise_intensity std_msgs/Float64 "data: -1"
