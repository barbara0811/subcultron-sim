#!/usr/bin/python

"""
Environment simulation class. Initializes the ROS node for complete environment simulation.
It includes current simulation, temperature simulation ...
"""

__author__ = "barbanas"

from std_msgs.msg import Bool
import rospy

class EnvironmentSim(object):

    def __init__(self):
        self.start = False

        # current
        self.currentSim = None
        self.init_current_sim()

        # temperature
        self.tempSim = None
        self.init_temp_sim()

        rospy.spin()

    
    def init_current_sim(self):
        """
        Initializes current simulation object. Depending on current_mode parameter,
        it generates an instance of respective class. If the current_mode is set to 
        "none", the object will not be created.
        """
        currentMode = rospy.get_param('~current_mode')
        if currentMode == "constant":
            try:
                from current_sim_constant import CurrentSimConstant
            except ImportError:
                rospy.logerr("ERROR: Can't import module CurrentSimConstant")
                return -1
            filename = None
            if rospy.has_param('~current_spec_filename'):
                filename = rospy.get_param('~current_spec_filename')
            if filename is None:
                filename = "currentInfo.txt"
            self.currentSim = CurrentSimConstant(filename)

        elif currentMode == "periodic":
            try:
                from current_sim_periodic import CurrentSimPeriodic
            except ImportError:
                rospy.logerr("ERROR: Can't import module CurrentSimPeriodic")
                return -1
            self.currentSim = CurrentSimPeriodic()

            '''
            You can insert new current modes here (just unindent for one tab to be in the
                same level as other elif keywords).
            ***
            Template:
            elif currentMode == "mode":
                try:
                    from current_sim_mode import CurrentSimMode
                except ImportError:
                    rospy.logerr("ERROR: Can't import module CurrentSimMode")
                    return -1
                self.currentSim = CurrentSimMode()
            ***
            IMPORTANT: implement the desired behavior in current_sim_mode.py (for reference, 
                    see already implemented modes)
            '''

        elif currentMode == "none":
            return 0

        else:
            rospy.logerr("Current mode {0} is not implemented!".format(currentMode))

    def init_temp_sim(self):
        """
        Initializes temperature simulation object. Depending on temp_mode parameter,
        it generates an instance of respective class. If the temp_mode is set to 
        "none", the object will not be created.
        """
        tempMode = rospy.get_param('~temp_mode')
        if tempMode == "constant":
            try:
                from temperature_sim_constant import TemperatureSimConstant
            except ImportError:
                rospy.logerr("ERROR: Can't import module TemperatureSimConstant")
                return -1
            filename = None
            if rospy.has_param('~temp_spec_filename'):
                filename = rospy.get_param('~temp_spec_filename')
            if filename is None:
                filename = "temperatureInfo.txt"
            self.tempSim = TemperatureSimConstant(filename)

            '''
            You can insert new temperature modes here (just unindent for one tab to be in the
                    same level as other elif keywords).
            ***
            Template:
            elif tempMode == "mode":
                try:
                    from temperature_sim_mode import TemperatureSimMode
                except ImportError:
                    rospy.logerr("ERROR: Can't import module TemperatureSimMode")
                    return -1
                self.tempSim = TemperatureSimMode()
            ***
            IMPORTANT: implement the desired behavior in temperature_sim_mode.py (for reference, 
                    see already implemented modes)
            '''

        elif tempMode == "none":
            return 0

        else:
            rospy.logerr("Temperature mode {0} is not implemented!".format(tempMode))


if __name__ == '__main__':
    
    rospy.init_node('environment_sim', anonymous=True)
    try:
        EnvironmentSim()
    except rospy.ROSInterruptException:
        pass