import rospy
from sensor_msgs.msg import LaserScan, Range

laserscan = None

def callback(_laserscan):
    global laserscan
    laserscan = _laserscan

def lidar_360():
    pub = rospy.Publisher('/mavros/obstacle/send', LaserScan, queue_size=10)
    rospy.init_node('lidar_360')
    sub = rospy.Subscriber('/lidar_360', LaserScan, callback)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if (laserscan is not None):
            pub.publish(laserscan)
            print '|',
        rate.sleep()
        # for i in range(9):
        #     obj = Range()
        #     obj.min_range = 20 / 100
        #     obj.max_range = 5000 / 100
        #     obj.orientation = i
        #     obj.range = laserscan.ranges[i] / 100
        #     pub.publish(obj)
        #     rate.sleep()

if __name__ == "__main__":
    try:
        lidar_360()
    except rospy.ROSInitException:
        pass