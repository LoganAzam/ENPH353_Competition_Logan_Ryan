#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DirtroadVisualizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.active = True 
    
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.callback, queue_size=1)

        rospy.loginfo("Visualizer Node Initialized. Press 'q' to exit.")

    def callback(self, data):
        if not self.active:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # 1. Setup Dimensions
        h, w, _ = cv_image.shape
        
        # Define your specific ROI boundaries
        y_start, y_end = int(h/2), int(4*h/5)
        x_start, x_end = int(w/2), int(w)

        # 2. Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 3. Create the Blue Mask
        lower_blue = np.array([100, 120, 30])
        upper_blue = np.array([140, 255, 255])
        blue_mask_full = cv2.inRange(hsv, lower_blue, upper_blue)

        # 4. Apply the restricted ROI
        # We create a black image of the same size and only keep the ROI part
        mask_roi_only = np.zeros_like(blue_mask_full)
        mask_roi_only[y_start:y_end, x_start:x_end] = blue_mask_full[y_start:y_end, x_start:x_end]

        # 5. Count pixels ONLY in that restricted region
        pixel_count = cv2.countNonZero(mask_roi_only)

        # 6. Visual Feedback
        # Draw a bounding box around the ROI we are actually checking
        cv2.rectangle(cv_image, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
        
        # Display the pixel count
        cv2.putText(cv_image, f"Blue Pixels in ROI: {pixel_count}", 
                    (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 7. Visualization
        # Convert mask to BGR so we can stack it
        mask_bgr = cv2.cvtColor(mask_roi_only, cv2.COLOR_GRAY2BGR)
        stacked_view = np.hstack((cv_image, mask_bgr))

        cv2.imshow("Blue Sign Debug (Box shows ROI)", stacked_view)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User requested quit")

if __name__ == '__main__':
    rospy.init_node('dirtroad_visualizer_node')
    try:
        visualizer = DirtroadVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()