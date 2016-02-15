
def generate(fish):
    
    file = open("zeta_logger.py", 'w')
    
    file.write("#!/usr/bin/python\n\"\"\"\n\"\"\"\n\n")
    file.write("__author__ = \"barbanas\"\n")
    file.write("from misc_msgs.msg import zeta\n")
    file.write("from std_msgs.msg import Bool\n")
    file.write("import numpy as np\n")
    file.write("from math import sqrt, pow\n")
    file.write("import rospy\n\n")
    
    file.write(
    "class ZetaLogger(object):\n\n" +
    "    def __init__(self):\n\n")
    
    file.write("        self.zeta = [[] for i in range(" + str(fish) + ")]\n")
    file.write("        self.start = False\n")
    file.write("        self.zetas = set()\n")
    file.write("\n")
    for i in range(fish):
        file.write("        self.afish" + str(i + 1) + "zetaSub = rospy.Subscriber('/afish" + str(i + 1) + "/zeta', zeta, self.afish" + str(i + 1) + "zeta_cb)\n")
    file.write("\n")
    
    file.write("        file = open('/home/barbara/Desktop/logs_trust/zeta.txt','w')\n\n")
    file.write("        rospy.Subscriber('/scenario_start', Bool, self.start_cb)\n")
    file.write("        rospy.Timer(rospy.Duration(0.1), self.save_zeta)\n\n")
    
    file.write("        rospy.spin()\n\n")
    # functions
    
    for i in range(fish):
        file.write("    def afish" + str(i + 1) + "zeta_cb(self, msg):\n" +
                   "        self.zeta[" + str(i) + "] = msg.zeta\n" +
                   "        self.zetas.add(" + str(i) + ")\n")
        file.write("\n")
    file.write("\n")

    file.write("    def start_cb(self, msg):\n" +
               "        self.start = True\n")
    file.write("\n")
    
    file.write("    def save_zeta(self, event):\n" +
               "        if not self.start or len(self.zetas) < " + str(fish) + ":\n" +
               "            return\n" +
               "        file = open('/home/barbara/Desktop/logs_trust/zeta.txt','a')\n" +
               "        output = \"\"\n" +
               "        for i in range(" + str(fish) + "):\n " +
               "            output += \" \".join(str(x) for x in self.zeta[i])\n" + 
               "             output += \" \" \n" +
               "        file.write(output + \"\\n\")\n\n"
               "        Zeta_ = np.zeros(len(self.zeta[0]))\n" + 
               "        for i in range(" + str(fish) + "):\n" +
               "            Zeta_ += self.zeta[i]\n" +
               "        Zeta_ /= " + str(fish) + "\n\n" +
               "        stdDev = np.zeros(len(self.zeta[0]))\n" +
               "        for i in range(" + str(fish) + "):\n" +
               "            stdDev += np.power(Zeta_ - self.zeta[i], 2)\n" +
               "        stdDev /= (" + str(fish) + "*" + str(fish - 1) + ")\n" +
               "        stdDev = np.sqrt(stdDev)\n"
               "        print stdDev\n"
                   )
    file.write("\n")
    
    file.write("if __name__ == \"__main__\":\n" +
    "    rospy.init_node(\"zeta_logger\")\n" +
    "    try:\n" +
    "        logger = ZetaLogger()\n" +
    "    except rospy.ROSInterruptException:\n" +
    "        pass")
