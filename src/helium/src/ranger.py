import rospy
from sensor_msgs.msg import Range

def ranger():
    pub = rospy.Publisher('/mavros/distance_sensor/rangefinder_sub', Range, queue_size=10)
    rospy.init_node('ranger', anonymous=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        print '|',
        obj = Range()
        obj.header.stamp = rospy.Time.now()
        obj.header.frame_id = "lidar"
        obj.radiation_type = 1
        obj.field_of_view = 0
        obj.min_range = 0.2
        obj.max_range = 50
        obj.range = 30
        pub.publish(obj)
        rate.sleep()

if __name__ == "__main__":
    try:
        ranger()
    except rospy.ROSInterruptException:
        pass