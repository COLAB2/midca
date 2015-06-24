from __future__ import print_function
import time
import rospy
from geometry_msgs.msg import Point, PointStamped

OBJ_LOC_TOPIC = 'quad_pos'

class FakeQuad:
    
    '''
    For a more realistic test, finish this class that implements a simulated quad
    copter which moves around a given space.
    '''
    
    def __init__(self, startPos, startVelocity, moveCenter, moveRadius, accel, 
                 maxSpeed):
        '''
        Quadcopter moves pseudo-randomly in a roughly spherical area. Note
        that 'radius' is an imprecise term and the quad may move outside of it.
        '''
        self.pos = startPos
        self.velocity = startVelocity
        self.moveCenter = moveCenter
        self.moveRadius = moveRadius
        self.accel = accel
        self.maxSpeed = maxSpeed
    
    def next_accel_vector(self):
         '''
         the quad accelerates against its current vector in proportion to the fraction
         of its maximum speed it is traveling.
         '''
         counterSpeedFraction = min(1.0, 
                        sqrt(sum([comp ** 2 for comp in self.velocity])) / maxSpeed)
         towardsCenterFraction = (1.0 - counterSpeedFraction)
    
    


def send_msgs():
    rospy.init_node('baxter_pointing_test')
    pub = rospy.Publisher(OBJ_LOC_TOPIC, PointStamped, queue_size=10)
    points = [Point(x = 5.0, y = 0.0, z = 0.0),
              Point(x = 8.0, y = 7.0, z = 2.0),
              Point(x = 7.0, y = -4.0, z = -0.5), Point(x = -1, y = 1, z = 0)]
    n = 0
    while not rospy.is_shutdown():
        n += 1
        time.sleep(2)
        p = points[n % 4]
        print("Sending point command:", p)
        pub.publish(PointStamped(point = p))

def straight_ahead_msgs():
    rospy.init_node('baxter_pointing_test')
    pub = rospy.Publisher(OBJ_LOC_TOPIC, PointStamped, queue_size=10)
    point = Point(x = 5.0, y = 0.0, z = 0.0)    n = 0
    while not rospy.is_shutdown():
        time.sleep(2)
        print("Sending point command:", point)
        pub.publish(PointStamped(point = point))

if __name__ == "__main__":
    send_msgs()
    #straight_ahead_msgs()
