#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(ros_image):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgra8')
    except CvBridgeError as e:
        rospy.logerr(f"CvBridgeError: {e}")
        return

    # Display the OpenCV image
    cv2.imshow("BGRA Image", cv_image)
    cv2.waitKey(1)

def image_listener():
    # Initialize the ROS node
    rospy.init_node('image_listener', anonymous=True)
    
    # Create a subscriber to the /camera/image topic
    rospy.Subscriber("/camera/image", Image, image_callback)
    
    # Keep the node running
    rospy.spin()

    # Clean up when the node is shutdown
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        image_listener()
    except rospy.ROSInterruptException:
        pass