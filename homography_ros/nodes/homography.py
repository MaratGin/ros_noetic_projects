
import rospy
import os
import cv2
import rospkg
import numpy as np

# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge
from webcam_task.msg import Motion

# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from typing import Final

from homography_ros.srv import ImageData,ImageDataResponse

ROS_NODE_NAME: Final[str] = "detector_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")
ROS_IMAGE_TOPIC: Final[str] = "/usb_cam/image_raw"

class CornerSelector:
    def __init__(self, image=None, window_name="Select 4 сorners"):
        rospy.init_node(ROS_NODE_NAME)
        self.service = rospy.Service('image_service', ImageData, self.handle_map_request)

        rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
        rospy.loginfo(f"OpenCV version: {cv2.__version__}")
        self.cv_bridge: CvBridge = CvBridge()
        # self.sub = rospy.Subscriber(ROS_IMAGE_TOPIC, Image, self.image_callback, queue_size=None)
        self.image = None
        self.display_image = None
        self.window_name = window_name
        self.corners = []
        self.is_completed = False
        
    def select_corners(self):
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
        
        print("Инструкция:")
        print("1. Кликните ЛКМ по 4 углам документа (по часовой стрелке или против)")
        print("2. После выбора 4 точек нажмите ESC для завершения")
        
        while True:
            cv2.imshow(self.window_name, self.display_image)
            key = cv2.waitKey(1) & 0xFF
            
            # ESC для завершения или автоматически после 4 точек
            if key == 27 or self.is_completed:
                break
        
        cv2.destroyWindow(self.window_name)
        if not self.is_completed:
          return None
        
        return np.array(self.corners, dtype=np.float32)
    
    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.corners) < 4:
            self.corners.append((x, y))
            print(f"Выбрана точка {len(self.corners)}: ({x}, {y})")
            

            for i, (cx, cy) in enumerate(self.corners, 1):
                cv2.circle(self.display_image, (cx, cy), 10, (0, 0, 255), -1)
                cv2.putText(self.display_image, str(i), (cx + 15, cy - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            if len(self.corners) == 4:
                self.is_completed = True
                print("Все 4 точки выбраны!")

    def image_callback(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg)
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.prev_frame is None:
            self.prev_frame = frame
            return
        diff = cv2.absdiff(self.prev_frame, frame)     
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)          
        motion_pixels = np.sum(thresh > 0)
        motion_detected = motion_pixels > 2500
        msg = Motion()
        msg.detected = motion_detected
        msg.diff = np.sum(diff)
        print(msg.detected)
        self.pub.publish(msg)
        self.prev_frame = frame

    def handle_map_request(self, req):
        rospy.loginfo("Received request.")
        image = req.image
        print(image)
        self.image = req.image
        flag = False
        opcv_image = self.cv_bridge.imgmsg_to_cv2(self.image)
        self.display_image =  opcv_image

        selected_corners = self.select_corners()
    
        if selected_corners is not None:
            print("\nВыбранные точки:")
            print(selected_corners)
            src_corner_points = selected_corners 
            width, height = 210, 297
            dst_corner_points = np.array([[0,0],[width,0],[width,height],[0,height]], dtype=np.float32)
            H = cv2.getPerspectiveTransform(src_corner_points, dst_corner_points)
            corrected_img = cv2.warpPerspective(opcv_image, H, (width, height))
            h1, w1 = opcv_image.shape[:2]
            h2, w2 = corrected_img.shape[:2]
            if h1 != h2:
                scale = h1 / h2
                img2_resized = cv2.resize(corrected_img, (int(w2 * scale), h1))
            else:
                img2_resized = corrected_img

            image = cv2.hconcat([opcv_image, img2_resized])
            final_image = self.cv_bridge.cv2_to_imgmsg(image)
            flag = True
        else:
            print("Не удалось выбрать 4 точки!")
        print("Finish")
        return ImageDataResponse(final_image)



if __name__ == "__main__":
    rospy.loginfo("Start")
    selector = CornerSelector()
    rospy.spin()

