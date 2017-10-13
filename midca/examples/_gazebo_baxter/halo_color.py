#!/usr/bin/env python

# What this code does = Set the halo on Baxters head to the color Green then 
# verbage that the "Halo should be green"

########################################################################### 
#
# Write code = thy_vo Dec 16, 2016
# tvo1@ball.com
#

#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 

#
#############################################################################
import rospy
import baxter_interface
from std_msgs.msg import Float32

class HaloLed():
    def __init__(self):
	#rospy.init_node("Halo_set_color")
        self.pub = {}
        self.pub["red"] = rospy.Publisher("/robot/sonar/head_sonar/lights/set_red_level",Float32,queue_size = 1)
        self.pub["green"] = rospy.Publisher("/robot/sonar/head_sonar/lights/set_green_level",Float32,queue_size = 1)

    def setLed(self,color, intensity):
        """
            Set an intensity value for the halo led
            
            :param color: Color of the led (red, green) 
            :type color: str
            :param intensity: Float value for the intensity between 0.0 and 100.0 
            :type intensity: float
        """
        try:
            self.pub[color].publish(Float32(intensity))
        except Exception,e:
            rospy.logwarn("%s",str(e))
#####
# The code below has 3 different settings available, NOT all def are needed
# to operate this code.  The colors are Green, Red and Yellow to pick from.
# Only one color is needed but the def reset is always needed
#
#####




    def setGreen(self):
	rospy.sleep(0.3)
        self.setLed("green",100)
        self.setLed("red",0)
        
    def setRed(self):
	rospy.sleep(0.3)
        self.setLed("red",100)
        self.setLed("green",0)
        







