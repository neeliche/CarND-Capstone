#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

'''
Creating a simple node that can help visualize what the car is seeing at the given moment.

In order to run it, in a separate terminal, while the rosmaster (through roslaunch) is running for the main task,
source the devel/setup.bash and run the program as a regular python piece of code.

`python camera_preview.py`
'''


class CameraPreview(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber('/image_color', Image, self.camera_callback)

        self.loop()

    def loop(self):
        rospy.init_node('camera_preview_node', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down"
        cv2.destroyAllWindows()

    def camera_callback(self, data):

        try:
            # rospy.loginfo("Attempting to view the images")
            cv_image = self.bridge_object.imgmsg_to_cv2(data)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as error:
            print("Something went wrong - {}".format(error))


if __name__=='__main__':
    CameraPreview()
