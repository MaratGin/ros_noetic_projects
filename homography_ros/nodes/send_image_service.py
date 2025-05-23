#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from homography_ros.srv import ImageData
from typing import Final
from cv_bridge import CvBridge

ROS_IMAGE_TOPIC: Final[str] = "/image_send"


# Предопределенные точки (замените на свои)
src_points = np.array([[100, 100], [700, 100], [700, 500], [100, 500]], dtype=np.float32)
width_px, height_px = 800, int(800 * 210 / 297)
dst_points = np.array([[0, 0], [width_px, 0], [width_px, height_px], [0, height_px]], dtype=np.float32)
H = cv2.getPerspectiveTransform(src_points, dst_points)

def send_request(image):
    print(type(image))
    rospy.wait_for_service('image_service')
    try:
        send_image = rospy.ServiceProxy('image_service', ImageData)
        rospy.loginfo("Sending Image to server...")
        response = send_image(image)
        print("SEND")
        rospy.loginfo("Received response map from server.")

        return response.image

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None
    
def main():
    try:
        rospy.init_node('send_image_client')
        print("Started service-client")
        sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout=3.0)
        received_image = send_request(sample)
        if received_image:
            rospy.loginfo(f"Received") 
            test = received_image
            print(type(test.data)) 
            bridge = CvBridge()
            opcv_image = bridge.imgmsg_to_cv2(received_image)
            res = cv2.resize(opcv_image, dsize=(720, 360), interpolation=cv2.INTER_CUBIC)
            cv2.imshow("Final", res)
            cv2.waitKey(0)  
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()