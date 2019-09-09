import rospy
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray

rospy.init_node('mantra_control_pub')
pub = rospy.Publisher('mantra_jog', Float32MultiArray, queue_size=1)

r = rospy.Rate(10)  # 10hz
while not rospy.is_shutdown():
    array = Float32MultiArray()
    array.data = [1, 2, 3]

    pub.publish(array)
    r.sleep()