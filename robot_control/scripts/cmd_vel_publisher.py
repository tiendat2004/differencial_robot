#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
import tf
import math

# Các biến toàn cục
x = 0.0
y = 0.0
theta = 0.0
last_time = None  # Thời gian của lần cập nhật trước

def cmd_vel_callback(msg):
    global x, y, theta, last_time

    # Lấy thời gian hiện tại
    current_time = rospy.Time.now()

    # Nếu là lần đầu tiên, chỉ cần cập nhật thời gian và bỏ qua tính toán
    if last_time is None:
        last_time = current_time
        return

    # Tính khoảng thời gian giữa hai lần cập nhật
    dt = (current_time - last_time).to_sec()
    last_time = current_time

    # Lấy tốc độ từ cmd_vel
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    # Cập nhật vị trí (x, y) và góc (theta)
    delta_x = linear_x * dt * math.cos(theta)
    delta_y = linear_x * dt * math.sin(theta)
    delta_theta = angular_z * dt

    x += delta_x
    y += delta_y
    theta += delta_theta

    # Xuất bản dữ liệu odometry
    publish_odometry(x, y, theta, current_time)

def publish_odometry(x, y, theta, current_time):
    # Chuyển đổi theta thành quaternion
    quat = quaternion_from_euler(0, 0, theta)

    # Tạo thông điệp odometry
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # Cập nhật vị trí
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3]

    # Cập nhật tốc độ
    odom.twist.twist.linear.x = linear_x
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.angular.z = angular_z

    # Xuất bản odometry
    odom_pub.publish(odom)

    # Xuất bản TF giữa `odom` và `base_link`
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (x, y, 0),
        quat,
        current_time,
        "base_link",
        "odom"
    )

def odometry_publisher():
    global odom_pub

    # Khởi tạo node
    rospy.init_node('odometry_publisher', anonymous=True)

    # Tạo publisher cho topic `/odom`
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    # Tạo subscriber cho topic `/cmd_vel`
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    # ROS spin để giữ cho node chạy
    rospy.spin()

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException:
        pass

