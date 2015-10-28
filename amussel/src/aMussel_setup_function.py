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

launchFileTemplate = "amussel_standard_simulation_raw.launch"
launchFile = "amussel_standard_simulation.launch"

controllerFile = "raw_controller.py"
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
            
def fill_up_simulation_spec_file(root, n, first_index):
    
    vehicleRoot = xml.etree.ElementTree.parse(rospack.get_path('amussel') + '/data/simulation/' + 'vehicle.xml').getroot()
    
    size = len(root)
    
    for i in range(n):
        tmp = deepcopy(vehicleRoot)
        name = "amussel" + str(first_index + i + 1)
        tmp.find("name").text = name
        #pos = tmp.find("position")
        #pos.find("x").text = str(positions[i].north)
        #pos.find("y").text = str(positions[i].east)
        #pos.find("z").text = "0"
        tmp.find("imu").find("name").text = "imu" + str(first_index + i + 1)
        tmp.find("gpsSensor").find("name").text = "GPSSensor" + str(first_index + i + 1)
        root.insert(size - 1, tmp)    
        
    root.append(xml.etree.ElementTree.Element("rosInterfaces"))    
    for i in range(n):
        name = "amussel" + str(first_index + i + 1)
        tmp = xml.etree.ElementTree.Element("ROSOdomToPAT")
        tmp.append(xml.etree.ElementTree.Element("topic"))
        tmp[-1].text = name + "/uwsim_hook"
        
        tmp.append(xml.etree.ElementTree.Element("vehicleName"))
        tmp[-1].text = name
        
        root.find("rosInterfaces").append(tmp)
        
def fill_up_launch_file(root, n, positions, first_index):
    
    for i in range(n):
        name = 'amussel' + str(first_index + i + 1)
        group = xml.etree.ElementTree.Element("group", {"ns":name})
        
        # Load parameters
        group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find amussel)/data/navigation/params/nav_standard.yaml"}))
        group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find amussel)/data/control/params/controllers_standard.yaml"}))
        group.append(xml.etree.ElementTree.Element("rosparam", {"file":"$(find amussel)/data/dynamics/amussel.yaml"}))
        group.append(xml.etree.ElementTree.Element("rosparam", {"param":"eta0"}))
        group[-1].text = "[" + str(positions[i].north) + "," + str(positions[i].east) + "," + str(positions[i].depth) + ",0,0,0]"
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
        group[-1].append(xml.etree.ElementTree.Element("arg", {"name":"hook_sel", "value":"amussel" + str(first_index + i + 1) + "/uwsim_hook"}))

        #README --> to run a different controller, instead of "controller_for_scenario_one.py" write the name of your function, for example "type":"my_new_controller_for_scenario_one.py" 
        if outputToScreen:
            group.append(xml.etree.ElementTree.Element("node", {"pkg":"amussel", "type":controllerFile, "name":"scenario_controller", "output":"screen"}))
        else:	
            group.append(xml.etree.ElementTree.Element("node", {"pkg":"amussel", "type":controllerFile, "name":"scenario_controller"}))
        
	root.append(group)
    
def generate_yaml_files(n, positions, first_index):
    
    for i in range(n):
        name = 'amussel' + str(first_index + i + 1)
        fileOut = open(rospack.get_path('amussel') + '/data/dynamics/' + name + ".yaml", 'w')
   
        # remove file content
        fileOut.seek(0)
        fileOut.truncate()
        
        fileOut.write("sampling_time: 0.1\n")
        fileOut.write("mass: 3\n")
        fileOut.write("gravity: 9.81\n")
        fileOut.write("fluid_density: 1025.0\n")
        fileOut.write("\n# The bounding ellipsoid parameters (ax, ay, az) used for buoyancy calculation\n")
        fileOut.write("bounding_ellipsoid: [0.1,0.1,0.1]\n")
        fileOut.write("\n# The dynamic parameters\n")
        fileOut.write("rg: [0, 0, 0]\n")
        fileOut.write("rb: [0, 0, 0]\n")
        fileOut.write("\n# The initial states\n")
        fileOut.write("eta0: [%f,%f,0,0,0,0.0] # Initial pose\n" % (positions[i].north, positions[i].east))
        fileOut.write("current: [0,0,0] # Initial currents (N,E,D)\n")
        fileOut.write("\n# The process and measurement noise specification\n")
        fileOut.write("coupled: false # Use a coupled model\n")
        
        fileOut.close()
       
if __name__ == "__main__":
    
    if len(sys.argv) < 2:
        print "USAGE: python aMussel_random_generator.py aMussel_number [first_index] [north_min] [north_max] [east_min] [east_max]"
        sys.exit(0)
        
    n = int(sys.argv[1])
    north_range = [-100, 100]
    east_range = [-100, 100]
    first_index = 0
    
    if len(sys.argv) == 7:
        first_index = int(sys.argv[6])
    if len(sys.argv) >= 6:
        east_range[1] = float(sys.argv[5])
    if len(sys.argv) >= 5:
        east_range[0] = float(sys.argv[4])
    if len(sys.argv) >= 4:
        north_range[1] = float(sys.argv[3])
    if len(sys.argv) >= 3:
        north_range[0] = float(sys.argv[2])
        
    print ""    
    print "aMussel number"
    print n 
    print "north range"
    print north_range
    print "east range"
    print east_range
    print "first index"
    print first_index 
    print "" 
    # generate random positions     
    positions = []
    posID = []
    
    while len(positions) < n:
        north = uniform(north_range[0], north_range[1])
        east = uniform(east_range[0], east_range[1])
        tmp = '%.2f%.2f' % (north, east)
        if tmp not in posID:
            posID.append(tmp)
            positions.append(NED(north, east, 0))
    
    rospack = rospkg.RosPack()
            
    # write into scene specification file (swarm_test.xml)
    fileOut = open(rospack.get_path('amussel') + '/data/simulation/' + sceneSpecFile, 'w')
   
    # remove file content
    fileOut.seek(0)
    fileOut.truncate()
    fileOut.write("<?xml version=\"1.0\"?>\n")
    fileOut.write("<!DOCTYPE UWSimScene SYSTEM \"UWSimScene.dtd\" >\n")
   
    tree = xml.etree.ElementTree.parse(rospack.get_path('amussel') + '/data/simulation/' + sceneSpecTemplate)
    root = tree.getroot()
    
    fill_up_simulation_spec_file(root, n, first_index)
    
    indent(root)
    tree.write(fileOut)
     
    fileOut.close()
    
    # write into launchfile
    fileOut = open(rospack.get_path('amussel') + '/launch/simulation/' + launchFile, 'w')
   
    # remove file content
    fileOut.seek(0)
    fileOut.truncate()
   
    tree = xml.etree.ElementTree.parse(rospack.get_path('amussel') + '/launch/simulation/' + launchFileTemplate)
    root = tree.getroot()
    
    fill_up_launch_file(root, n, positions, first_index)
    
    indent(root)
    tree.write(fileOut)
     
    fileOut.close()
    
    # generate yaml files
    #generate_yaml_files(n, positions)
    
        
