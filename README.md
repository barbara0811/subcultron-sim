subCULTron project trust scenario implementation.

**TO RUN:**

**1. Prepare simulation structures:**

- Navigate to subcultron_launch/src/

> roscd subcultron_launch/src

- Prepare the simulation structures (has to be done only once for each configuration):

>  python setup.py scenario_name apad_number afish_number amussel_number [first_index_apad] [first_index_afish] [first_index_amussel] [north_min] [north_max] [east_min] [east_max]

For example,

>  python setup.py trust_scenario 0 5 10 0 0 0 -15 15 -15 15

> source ../data/simulation/simulation_config.bash

- Prepare aFish controller file for launch with specified parameters (modify PARAMETERS at the begining of afish/src/controller_for_trust_scenario.py).

- Make sure you have prepared logging structures (folder ~/Desktop/logs_trust). If such folder does not exist, run:

> cd ~/Desktop/

> mkdir logs_trust

**2. Run the code:**

- In separate terminals, run:

> roscore

> roslaunch subcultron_launch standard_simulation.launch 

- If you want to log zeta values, run:

> rosrun subcultron_launch zeta_logger.py

To stop the simulation, simply press ctrl-c in the second terminal (roscore can be left running at all times).

- Start scenario simulation:

> rostopic pub /scenario_start std_msgs/Bool "data: true"

- Change noise source amplitude (setting a negative number turns the source off, a positive number attracts all aFishes in the area of noise source):

> rostopic pub /noise_intensity std_msgs/Float64 "data: 10"

> rostopic pub /noise_intensity std_msgs/Float64 "data: -1"
