import cv2
import numpy as np #library for mathematical functions
import rospy as rp
from cv2 import namedWindow
from cv2 import destroyAllWindows, startWindowThread
from numpy import mean
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError #conversion of ROS data to OpenCV format
from geometry_msgs.msg import Twist


class main:

    def __init__(self):
        rp.loginfo("Starting Node")
        namedWindow("Camera Image")
        #namedWindow("left") #debug
        #namedWindow("right") #debug
        self.laser = rp.Subscriber("/turtlebot_1/scan", LaserScan, self.avoid)
        self.bridge = CvBridge()
        startWindowThread()
        self.image_sub = rp.Subscriber("/turtlebot_1/camera/rgb/image_raw",
                                          Image, self.returnImage)
        self.pub = rp.Publisher("turtlebot_1/cmd_vel", Twist, queue_size = 1)                    
        
    def returnImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        #segmentation of the green channel - so green box can be segmented from the environment
        bgr_thresh = cv2.inRange(cv_image,
                                 np.array((0, 0, 0)),
                                 np.array((0, 255, 0)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 np.array((25, 50, 10)),
                                 np.array((100, 255, 255)))

        bgr_contours, hierachy = cv2.findContours(bgr_thresh.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)

        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
        cv2.imshow("Camera Image", cv_image)
        segImg = cv2.bitwise_and(hsv_img,hsv_img,mask = hsv_thresh)
        imageleft, imageright = np.hsplit(segImg, 2)
        #cv2.imshow("left", imageleft) #debug images
        #cv2.imshow("right", imageright) #debug images
        meanLeft = mean(imageleft)
        meanRight = mean(imageright)
        self.send_velocities(meanLeft, meanRight)

            
    def send_velocities(self, left, right):
        r = rp.Rate(20) #set frequency of commands
        #rp.loginfo("Sending commands")
        twist_msg = Twist()
        if (self.scan > 1):
            #if object is far left, go ccw and forwards
            if (left > right):
                #print("left>right")
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = 0.1
            if (right > left):
                #print("right>left")
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = -0.1
            if (right == 0 and left == 0):
                #print("nothing found")
                twist_msg.linear.x = 0.4
                twist_msg.angular.z = 0.3
        else:
            if ((left + right) > 1.4):
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                print("arrived at love")
            else:
                #print("avoided")
                twist_msg.linear.x = -1;
                twist_msg.linear.y = 0;
                twist_msg.angular.z = 2;
        self.pub.publish(twist_msg) # Sending the command
        r.sleep()

    #object collision is handled, to ensure that it does not bump into the environment        
    def avoid(self, data):
        self.scan = np.nanmin(data.ranges)
            

            
main()
rp.init_node('main', anonymous=True)
main = main()
## Calling the function
rp.spin()
destroyAllWindows()