import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class OdometryNode:
    def __init__(self):
        rospy.init_node('odometry_node')
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('/odom', Float32, self.odom_callback)
        
        self.x = 0
        self.y = 0
        self.theta = 0

    def odom_callback(self, msg):
        linear_velocity = msg.data
        
        # Giả sử robot chỉ quay theo một trục
        delta_x = linear_velocity * math.cos(self.theta)
        delta_y = linear_velocity * math.sin(self.theta)
        
        # Cập nhật vị trí robot
        self.x += delta_x
        self.y += delta_y
        
        # Tạo và xuất thông tin odometry
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        # Gửi thông tin odometry
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    odometry_node = OdometryNode()
    rospy.spin()
