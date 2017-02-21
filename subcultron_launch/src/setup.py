#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = "barbanas"

import sys
import rospy
import rospkg
from copy import deepcopy
from random import uniform
from auv_msgs.msg import NED
import xml.etree.ElementTree
import argparse
import ast

########################################
scenario = "docking_scenario"
########################################

parser = argparse.ArgumentParser(description='Define setup parameters.')

# find available simulation scenarios
rospack = rospkg.RosPack()
scenarioSpec = xml.etree.ElementTree.parse(rospack.get_path('subcultron_launch') + '/data/scenario/scenario_spec.xml').getroot()
scenario_choices = []
for child in scenarioSpec:
    scenario_choices.append(child.find('name').text)

parser.add_argument('scenario_name', help='Scenario name', choices=scenario_choices, nargs='?')
parser.add_argument('apad_number', help='Number of aPad agents.', type=int, nargs='?')
parser.add_argument('afish_number', help='Number of aFish agents.', type=int, nargs='?')
parser.add_argument('amussel_number', help='Number of aMussel agents.', type=int, nargs='?')
parser.add_argument('first_index_apad', help='aPad first index.', type=int, nargs='?')
parser.add_argument('first_index_afish', help='aFish first index.', type=int, nargs='?')
parser.add_argument('first_index_amussel', help='aMussel first index.', type=int, nargs='?')
parser.add_argument('north_min', help='Minimum north position.', type=int, nargs='?')
parser.add_argument('north_max', help='Maximum north position.', type=int, nargs='?')
parser.add_argument('east_min', help='Minimum east position.', type=int, nargs='?')
parser.add_argument('east_max', help='Maximum east position.', type=int, nargs='?')

parser.add_argument('-f', '--file', help="Load from file. If file passed, there's no need for positional arguments.")

args = parser.parse_args()
if not args.file:
    if None in (args.scenario_name, args.apad_number, args.afish_number, args.amussel_number):
        parser.parse_args(['-h'])


sceneSpecTemplate = "swarm_test_raw.xml"
sceneSpecFile = "swarm_test.xml"

launchFileTemplate = "standard_simulation_raw.launch"
launchFile = "standard_simulation.launch"

agents = []
controllerFile = "" #"API_test.py"
simulationSpecFile = ""

outputToScreen = True


def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def fill_up_simulation_spec_file(root, n_pad, positions_pad, first_index_pad, n_fish, positions_fish, first_index_fish, n_mussel, positions_mussel, first_index_mussel):
    
      
    if n_pad > 0:
        vehicleRoot = xml.etree.ElementTree.parse(rospack.get_path('apad') + '/data/simulation/' + 'vehicle.xml').getroot()
        size = len(root)
    
    for i in range(n_pad):
        tmp = deepcopy(vehicleRoot)
        name = "apad" + str(first_index_pad + i + 1)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_pad + i + 1)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_pad + i + 1)
        root.insert(size - 1, tmp)    

    if n_fish > 0:
        vehicleRoot = xml.etree.ElementTree.parse(rospack.get_path('afish') + '/data/simulation/' + 'vehicle.xml').getroot()
        size = len(root)
    
    for i in range(n_fish):
        tmp = deepcopy(vehicleRoot)
        name = "afish" + str(first_index_fish + i + 1)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_fish + i + 1)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_fish + i + 1)
        root.insert(size - 1, tmp)  
    
    if n_mussel > 0:
        vehicleRoot = xml.etree.ElementTree.parse(rospack.get_path('amussel') + '/data/simulation/' + 'vehicle.xml').getroot()    
        size = len(root)

    for i in range(n_mussel):
        tmp = deepcopy(vehicleRoot)
        name = "amussel" + str(first_index_mussel + i + 1)
        tmp.find("name").text = name
        tmp.find("imu").find("name").text = "imu" + str(first_index_mussel + i + 1)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_mussel + i + 1)
        root.insert(size - 1, tmp)    

    root.append(xml.etree.ElementTree.Element("rosInterfaces"))    
    for i in range(n_pad):
        name = "apad" + str(first_index_pad + i + 1)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)

    for i in range(n_fish):
        name = "afish" + str(first_index_fish + i + 1)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)
          
    for i in range(n_mussel):
        name = "amussel" + str(first_index_mussel + i + 1)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)


def fill_up_launch_file(root, n_pad, positions_pad, first_index_pad, n_fish, positions_fish, first_index_fish, n_mussel, positions_mussel, first_index_mussel):
    
    if "aPad" in agents:
        for i in range(n_pad):
            name = 'apad' + str(first_index_pad + i + 1)
            group = xml.etree.ElementTree.Element("group", {"ns":name})
            
            # Load parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find apad)/data/navigation/params/nav_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find apad)/data/control/params/controllers_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find apad)/data/dynamics/apad.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"param":"eta0"}))
            group[-1].text = "[" + str(positions_pad[i].north) + "," + str(positions_pad[i].east) + "," + str(positions_pad[i].depth) + ",0,0,0]"
            # Location parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"command":"load", "file":"$(find apad)/data/locations/swarm_loc.yaml"}))
            
            # Static TF
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/devices/static_frames.xml"}))
            # Load the simulation
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/simulation/scenario/" + simulationSpecFile}))
            
            # Load the controllers
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/control/control_standard.xml"}))
            # Load the primitives
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/primitives/primitives_standard.xml"}))
            # Load visualization
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/simulation/visualization_standard.xml"}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name":"hook_sel", "value":"apad" + str(first_index_pad + i + 1) + "/uwsim_hook"}))
    
            
            if outputToScreen:
                group.append(xml.etree.ElementTree.Element("node", {"pkg":"apad", "type":controllerFile, "name":"scenario_controller", "output":"screen"}))
                group.append(xml.etree.ElementTree.Element("node", {"pkg":"apad", "type":"action_server.py", "name":"action_server", "output":"screen"}))
            else:	
                group.append(xml.etree.ElementTree.Element("node", {"pkg":"apad", "type":controllerFile, "name":"scenario_controller"}))
                group.append(xml.etree.ElementTree.Element("node", {"pkg":"apad", "type":"action_server.py", "name":"action_server"}))
            
            root.append(group)

    if "aFish" in agents:
        for i in range(n_fish):
            name = 'afish' + str(first_index_fish + i + 1)
            group = xml.etree.ElementTree.Element("group", {"ns":name})
            
            # Load parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find afish)/data/navigation/params/nav_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find afish)/data/control/params/controllers_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find afish)/data/dynamics/afish.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"param":"eta0"}))
            group[-1].text = "[" + str(positions_fish[i].north) + "," + str(positions_fish[i].east) + "," + str(positions_fish[i].depth) + ",0,0,0]"
            # Location parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"command":"load", "file":"$(find afish)/data/locations/swarm_loc.yaml"}))
            
            # Static TF
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find afish)/data/devices/static_frames.xml"}))
            # Load the simulation
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find afish)/data/simulation/scenario/" + simulationSpecFile}))
            
            # Load the controllers
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find afish)/data/control/control_standard.xml"}))
            # Load the primitives
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find afish)/data/primitives/primitives_standard.xml"}))
            # Load visualization
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find afish)/data/simulation/visualization_standard.xml"}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name":"hook_sel", "value":"afish" + str(first_index_fish + i + 1) + "/uwsim_hook"}))
    
            if outputToScreen:
                group.append(xml.etree.ElementTree.Element("node", {"pkg":"afish", "type":controllerFile, "name":"scenario_controller", "output":"screen"}))
            else:   
                group.append(xml.etree.ElementTree.Element("node", {"pkg":"afish", "type":controllerFile, "name":"scenario_controller"}))
            root.append(group)
	
    if "aMussel" in agents:
        for i in range(n_mussel):
            name = 'amussel' + str(first_index_mussel + i + 1)
            group = xml.etree.ElementTree.Element("group", {"ns":name})
            
            # Load parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find amussel)/data/navigation/params/nav_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find amussel)/data/control/params/controllers_standard.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find amussel)/data/dynamics/amussel.yaml"}))
            group.append(xml.etree.ElementTree.Element("rosparam", {"param":"eta0"}))
            group[-1].text = "[" + str(positions_mussel[i].north) + "," + str(positions_mussel[i].east) + "," + str(positions_mussel[i].depth) + ",0,0,0]"
            # Location parameters
            group.append(xml.etree.ElementTree.Element("rosparam", {"command":"load", "file":"$(find amussel)/data/locations/swarm_loc.yaml"}))
            
            # Static TF
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find amussel)/data/devices/static_frames.xml"}))
            # Load the simulation
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find amussel)/data/simulation/scenario/" + simulationSpecFile}))
            
            # Load the controllers
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find amussel)/data/control/control_standard.xml"}))
            # Load the primitives
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find amussel)/data/primitives/primitives_standard.xml"}))
            # Load visualization
            group.append(xml.etree.ElementTree.Element("include", {"file":"$(find amussel)/data/simulation/visualization_standard.xml"}))
            group[-1].append(xml.etree.ElementTree.Element("arg", {"name":"hook_sel", "value":"amussel" + str(first_index_mussel + i + 1) + "/uwsim_hook"}))
    
            #README --> to run a different controller, instead of "controller_for_scenario_one.py" write the name of your function, for example "type":"my_new_controller_for_scenario_one.py" 
            if outputToScreen:
                pass
                group.append(xml.etree.ElementTree.Element("node", {"pkg":"amussel", "type":controllerFile, "name":"scenario_controller", "output":"screen"}))
            else:	
                pass
                group.append(xml.etree.ElementTree.Element("node", {"pkg":"amussel", "type":controllerFile, "name":"scenario_controller"}))
            
            root.append(group)

if __name__ == "__main__":

    if args.file:
        root = xml.etree.ElementTree.parse(args.file).getroot()
        for child in root:
            if child.tag == 'Scenario':
                if child.text not in scenario_choices:
                    raise Exception(
                            'Defined scenario {0} in file {1} not valid. Choose scenario from: {2}'.format(
                                child.text, args.file, scenario_choices
                            )
                        )
                else:
                    scenario = child.text
            if child.tag == 'aPad':
                for child2 in child:
                    if child2.tag == 'Number':
                        n_pad = int(child2.text)
                    if child2.tag == 'FirstIndex':
                        first_index_apad = int(child2.text)
                    if child2.tag == 'Positions':
                        apad_positions = ast.literal_eval(child2.text)
            if child.tag == 'aFish':
                for child2 in child:
                    if child2.tag == 'Number':
                        n_fish = int(child2.text)
                    if child2.tag == 'FirstIndex':
                        first_index_afish = int(child2.text)
                    if child2.tag == 'Positions':
                        afish_positions = ast.literal_eval(child2.text)
            if child.tag == 'aMussel':
                for child2 in child:
                    if child2.tag == 'Number':
                        n_mussel = int(child2.text)
                    if child2.tag == 'FirstIndex':
                        first_index_amussel = int(child2.text)
                    if child2.tag == 'Positions':
                        amussel_positions = ast.literal_eval(child2.text)
    else:
        scenario = args.scenario_name
        n_pad = args.apad_number
        n_fish = args.afish_number
        n_mussel = args.amussel_number


    north_range_pad = [-100, 100]
    east_range_pad = [-100, 100]
    first_index_pad = 0

    north_range_fish = [-100, 100]
    east_range_fish = [-100, 100]
    first_index_fish = 0

    north_range_mussel = [-100, 100]
    east_range_mussel = [-100, 100]
    first_index_mussel = 0

    if not args.file:
        if args.east_max is not None:
            east_range_pad[1] = east_range_fish[1] = east_range_mussel[1] = args.east_max
        if args.east_min is not None:
            east_range_pad[0] = east_range_fish[0] = east_range_mussel[0] = args.east_min
        if args.north_max is not None:
            north_range_pad[1] = north_range_fish[1] = north_range_mussel[1] = args.north_max
        if args.north_min is not None:
            north_range_pad[0] = north_range_fish[0] = north_range_mussel[0] = args.north_min

        if args.first_index_apad is not None:
            first_index_mussel = args.first_index_apad
        if args.first_index_afish is not None:
            first_index_fish = args.first_index_afish
        if args.first_index_amussel is not None:
            first_index_pad = args.first_index_amussel

    rospack = rospkg.RosPack()

    scenarioSpec = xml.etree.ElementTree.parse(
        rospack.get_path('subcultron_launch') + '/data/scenario/scenario_spec.xml').getroot()

    for child in scenarioSpec:
        if child.find('name').text == scenario:
            controllerFile = child.find('controllerFile').text
            simulationSpecFile = child.find('simulationXmlFile').text
            for ag in child.findall('agent'):
                agents.append(ag.text)

    print ""
    print "apad number"
    if "aPad" in agents:
        print n_pad
        print "north range"
        print north_range_pad
        print "east range"
        print east_range_pad
        print "first index"
        print first_index_pad
    else:
        print "0"
        n_pad = 0

    print ""
    print ""
    print "afish number"
    if "aFish" in agents:
        print n_fish
        print "north range"
        print north_range_fish
        print "east range"
        print east_range_fish
        print "first index"
        print first_index_fish
    else:
        print "0"
        n_fish = 0

    print ""
    print ""
    print "amussel number"
    if "aMussel" in agents:
        print n_mussel
        print "north range"
        print north_range_mussel
        print "east range"
        print east_range_mussel
        print "first index"
        print first_index_mussel
    else:
        print "0"
        n_mussel = 0
    print ""

    print "Scenario: " + scenario

    # generate random positions  
    positions_pad = []
    posID_pad = []

    positions_fish = []
    posID_fish = []
       
    positions_mussel = []
    posID_mussel = []

    index = 0
    while len(positions_pad) < n_pad:
        if not args.file:
            north = uniform(north_range_pad[0], north_range_pad[1])
            east = uniform(east_range_pad[0], east_range_pad[1])
        else:
            north = apad_positions[index][0]
            east = apad_positions[index][1]
        tmp = '%.2f%.2f' % (north, east)
        if tmp not in posID_pad:
            posID_pad.append(tmp)
            positions_pad.append(NED(north, east, 0))
        index += 1

    index = 0
    while len(positions_fish) < n_fish:
        if not args.file:
            north = uniform(north_range_fish[0], north_range_fish[1])
            east = uniform(east_range_fish[0], east_range_fish[1])
        else:
            north = afish_positions[index][0]
            east = afish_positions[index][1]
        tmp = '%.2f%.2f' % (north, east)
        print tmp
        if tmp not in posID_pad:
            posID_pad.append(tmp)
            positions_fish.append(NED(north, east, 0))
        index += 1

    index = 0
    while len(positions_mussel) < n_mussel:
        if not args.file:
            north = uniform(north_range_mussel[0], north_range_mussel[1])
            east = uniform(east_range_mussel[0], east_range_mussel[1])
        else:
            north = amussel_positions[index][0]
            east = amussel_positions[index][1]
        tmp = '%.2f%.2f' % (north, east)
        if tmp not in posID_mussel:
            posID_mussel.append(tmp)
            positions_mussel.append(NED(north, east, 0))
        index += 1
    
    # write into scene specification file (swarm_test.xml)


    fileOut = open(rospack.get_path('subcultron_launch') + '/data/simulation/' + sceneSpecFile, 'w')
   
    # remove file content
    fileOut.seek(0)
    fileOut.truncate()
    fileOut.write("<?xml version=\"1.0\"?>\n")
    fileOut.write("<!DOCTYPE UWSimScene SYSTEM \"UWSimScene.dtd\" >\n")
    
    tree = xml.etree.ElementTree.parse(rospack.get_path('subcultron_launch') + '/data/simulation/' + sceneSpecTemplate)
    root = tree.getroot()

    print '\n root: ', root
    print '\n n_pad: ', n_pad
    print '\n positions_pad: ', positions_pad
    print '\n first_index_pad: ', first_index_pad
    print '\n n_fish: ', n_fish
    print '\n positions_fish: ', positions_fish
    print '\n first_index_fish: ', first_index_fish
    print '\n n_mussel: ', n_mussel
    print '\n positions_mussel: ', positions_mussel
    print '\n first_index_mussel: ', first_index_mussel
    
    fill_up_simulation_spec_file(
        root,
        n_pad,
        positions_pad,
        first_index_pad,
        n_fish,
        positions_fish,
        first_index_fish,
        n_mussel,
        positions_mussel,
        first_index_mussel
    )
    
    indent(root)
    tree.write(fileOut)
    
    fileOut.close()
    
    # write into launchfile
    fileOut = open(rospack.get_path('subcultron_launch') + '/launch/simulation/' + launchFile, 'w')
   
    # remove file content
    fileOut.seek(0)
    fileOut.truncate()
       
    tree = xml.etree.ElementTree.parse(rospack.get_path('subcultron_launch') + '/launch/simulation/' + launchFileTemplate)
    root = tree.getroot()
    
    fill_up_launch_file(root, n_pad, positions_pad, first_index_pad, n_fish, positions_fish, first_index_fish, n_mussel, positions_mussel, first_index_mussel)
    
    indent(root)
    tree.write(fileOut)
     
    fileOut.close()
    
    import uwsim_location_holder_generator as gen
    gen.generate(n_fish, n_mussel)

    import zeta_logger_generator 
    zeta_logger_generator.generate(n_fish, n_mussel, east_range_mussel[1] - east_range_mussel[0], north_range_mussel[1] - north_range_mussel[0])


