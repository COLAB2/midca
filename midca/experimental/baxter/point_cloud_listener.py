import rospy
import roslib
roslib.load_manifest('sensor_msgs')
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

def callback(data):
    rospy.loginfo("got point cloud")
    points = point_cloud2.read_points(data.data)
    rospy.loginfo(" 1st ten items:" + str(type(data.data[10])))

def listener():

    rospy.init_node('pcl_listener')

    rospy.Subscriber("/camera/depth/points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

