#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import random

def steering_publisher():
    # Khởi tạo node với tên 'steering_publisher'
    rospy.init_node('steering_publisher', anonymous=True)

    # Tạo publisher để gửi dữ liệu đến topic /steering
    pub = rospy.Publisher('/steering', Float32, queue_size=10)

    # Tần số gửi tin nhắn (10 Hz)
    rate = rospy.Rate(10)  # 10 lần mỗi giây

    rospy.loginfo("Steering publisher node started. Publishing to /steering...")

    while not rospy.is_shutdown():
        # Tạo giá trị ngẫu nhiên cho góc lái từ -1.0 đến 1.0 (có thể thay đổi theo yêu cầu)
        steering_angle = random.uniform(-1.0, 1.0)
        
        # Đưa góc lái vào một message kiểu Float32
        msg = Float32()
        msg.data = steering_angle

        # Gửi message lên topic /steering
        pub.publish(msg)
        rospy.loginfo(f"Published steering angle: {steering_angle}")

        # Ngủ để giữ tần số gửi tin
        rate.sleep()

if _name_ == '_main_':
    try:
        steering_publisher()
    except rospy.ROSInterruptException:
        pass

