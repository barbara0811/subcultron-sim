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

sceneSpecTemplate = "swarm_test_raw.xml"
sceneSpecFile = "swarm_test.xml"

launchFileTemplate = "standard_simulation_raw.launch"
launchFile = "standard_simulation.launch"

controllerFile = "controller_for_docking_scenario.py"
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
            
def fill_up_simulation_spec_file(root, n_pad, positions_pad, first_index_pad, n_mussel, positions_mussel, first_index_mussel):
    
    vehicleRoot = xml.etree.ElementTree.parse(rospack.get_path('apad') + '/data/simulation/' + 'vehicle.xml').getroot()
    size = len(root)
    
    for i in range(n_pad):
        tmp = deepcopy(vehicleRoot)
        name = "apad" + str(first_index_pad + i + 1)
        tmp.find("name").text = name
        #pos = tmp.find("position")
        #pos.find("x").text = str(positions[i].north)
        #pos.find("y").text = str(positions[i].east)
        #pos.find("z").text = "0"
        tmp.find("imu").find("name").text = "imu" + str(first_index_pad + i + 1)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index_pad + i + 1)
        root.insert(size - 1, tmp)    
    
    vehicleRoot = xml.etree.ElementTree.parse(rospack.get_path('amussel') + '/data/simulation/' + 'vehicle.xml').getroot()    

    for i in range(n_mussel):
        tmp = deepcopy(vehicleRoot)
        name = "amussel" + str(first_index_mussel + i + 1)
        tmp.find("name").text = name
        #pos = tmp.find("position")
        #pos.find("x").text = str(positions[i].north)
        #pos.find("y").text = str(positions[i].east)
        #pos.find("z").text = "0"
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
          
    for i in range(n_mussel):
        name = "amussel" + str(first_index_mussel + i + 1)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)
        
def fill_up_launch_file(root, n_pad, positions_pad, first_index_pad, n_mussel, positions_mussel, first_index_mussel):
    
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
        #group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/devices/static_frames.xml"}))
        # Load the simulation
        group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/simulation/simulation_standard.xml"}))
        
        # Load the controllers
        group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/control/control_standard.xml"}))
        # Load the primitives
        group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/primitives/primitives_standard.xml"}))
        # Load visualization
        group.append(xml.etree.ElementTree.Element("include", {"file":"$(find apad)/data/simulation/visualization_standard.xml"}))
        group[-1].append(xml.etree.ElementTree.Element("arg", {"name":"hook_sel", "value":"apad" + str(first_index_pad + i + 1) + "/uwsim_hook"}))

        #README --> to run a different controller, instead of "controller_for_scenario_one.py" write the name of your function, for example "type":"my_new_controller_for_scenario_one.py" 
        if outputToScreen:
            group.append(xml.etree.ElementTree.Element("node", {"pkg":"apad", "type":controllerFile, "name":"scenario_controller", "output":"screen"}))
            group.append(xml.etree.ElementTree.Element("node", {"pkg":"apad", "type":"action_server.py", "name":"action_server", "output":"screen"}))
        else:	
            group.append(xml.etree.ElementTree.Element("node", {"pkg":"apad", "type":controllerFile, "name":"scenario_controller"}))
            group.append(xml.etree.ElementTree.Element("node", {"pkg":"apad", "type":"action_server.py", "name":"action_server"}))
        
        root.append(group)
	
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
        #group.append(xml.etree.ElementTree.Element("include", {"file":"$(find amussel)/data/devices/static_frames.xml"}))
        # Load the simulation
        group.append(xml.etree.ElementTree.Element("include", {"file":"$(find amussel)/data/simulation/simulation_standard.xml"}))
        
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
    
    if len(sys.argv) < 3:
        print "USAGE: python setup.py amussel_number apad_number [first_index_amussel] [first_index_apad] [north_min_am] \n\
        [north_max_am] [east_min_am] [east_max_am] [north_min_ap] [north_max_ap] [east_min_ap] [east_max_ap]"
        sys.exit(0)
        
    n_mussel = int(sys.argv[1])
    north_range_mussel = [-100, 100]
    east_range_mussel = [-100, 100]
    first_index_mussel = 0
    
    n_pad = int(sys.argv[2])
    north_range_pad = [-100, 100]
    east_range_pad = [-100, 100]
    first_index_pad = 0
    
    if len(sys.argv) >= 13:
        east_range_pad[1] = float(sys.argv[12])
    if len(sys.argv) >= 12:
        east_range_pad[0] = float(sys.argv[11])
    if len(sys.argv) >= 11:
        north_range_pad[1] = float(sys.argv[10])
    if len(sys.argv) >= 10:
        north_range_pad[0] = float(sys.argv[9])

    if len(sys.argv) >= 9:
        east_range_mussel[1] = float(sys.argv[8])
    if len(sys.argv) >= 8:
        east_range_mussel[0] = float(sys.argv[7])
    if len(sys.argv) >= 7:
        north_range_mussel[1] = float(sys.argv[6])
    if len(sys.argv) >= 6:
        north_range_mussel[0] = float(sys.argv[5])

    if len(sys.argv) >= 5:
        first_index_pad = int(sys.argv[4])
    if len(sys.argv) >= 4:
        first_index_mussel = int(sys.argv[3])

    print ""    
    print "amussel number"
    print n_mussel 
    print "north range"
    print north_range_mussel
    print "east range"
    print east_range_mussel
    print "first index"
    print first_index_mussel 
    print "" 
    print ""    
    print "apad number"
    print n_pad 
    print "north range"
    print north_range_pad
    print "east range"
    print east_range_pad
    print "first index"
    print first_index_pad 
    print "" 
    
    # generate random positions     
    positions_mussel = []
    posID_mussel = []
    
    positions_pad = []
    posID_pad = []
    
    while len(positions_mussel) < n_mussel:
        north = uniform(north_range_mussel[0], north_range_mussel[1])
        east = uniform(east_range_mussel[0], east_range_mussel[1])
        tmp = '%.2f%.2f' % (north, east)
        if tmp not in posID_mussel:
            posID_mussel.append(tmp)
            positions_mussel.append(NED(north, east, 0))
            
    while len(positions_pad) < n_pad:
        north = uniform(north_range_pad[0], north_range_pad[1])
        east = uniform(east_range_pad[0], east_range_pad[1])
        tmp = '%.2f%.2f' % (north, east)
        if tmp not in posID_pad:
            posID_pad.append(tmp)
            positions_pad.append(NED(north, east, 0))
    
    rospack = rospkg.RosPack()
            
    # write into scene specification file (swarm_test.xml)
    fileOut = open(rospack.get_path('subcultron_launch') + '/data/simulation/' + sceneSpecFile, 'w')
   
    # remove file content
    fileOut.seek(0)
    fileOut.truncate()
    fileOut.write("<?xml version=\"1.0\"?>\n")
    fileOut.write("<!DOCTYPE UWSimScene SYSTEM \"UWSimScene.dtd\" >\n")
    
    tree = xml.etree.ElementTree.parse(rospack.get_path('subcultron_launch') + '/data/simulation/' + sceneSpecTemplate)
    root = tree.getroot()
    
    fill_up_simulation_spec_file(root, n_pad, positions_pad, first_index_pad, n_mussel, positions_mussel, first_index_mussel)
    
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
    
    fill_up_launch_file(root, n_pad, positions_pad, first_index_pad, n_mussel, positions_mussel, first_index_mussel)
    
    indent(root)
    tree.write(fileOut)
     
    fileOut.close()
    
    # generate yaml files
    #generate_yaml_files(n, positions)
    
        

