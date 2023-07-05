#!/usr/bin/env python
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
import rospy
from std_msgs.msg import String,Int64
from sensor_msgs.msg import Image
import sys

bridge = CvBridge()#Create a bridge


def threshold_image(colour):
    if colour > 1500:
        image_pub.publish(colour)
        rate = rospy.Rate(1)
        #rate.sleep()
        
    else:
        image_pub.publish(0)
        rate = rospy.Rate(1)
        #rate.sleep()
    
    #rospy.spin()
def image_callback(ros_image):
    global bridge#Declare the bridge as global
    lower = np.array([36,25,25])
    upper = np.array([86,255,255])
   # global colour
    colour ='i'
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image,'bgr8')#Convert the image to cv2 using the bridge
    except CvBridgeError as e:#Try to catch errors
        print(e)#Print the error
    image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image,lower,upper)
    contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    for cntr in contours:
        x,y,w,h = cv2.boundingRect(cntr)
        cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
        colour = cv2.contourArea(cntr)
        print(colour)
    cv2.imshow('mask',mask)
    cv2.imshow('Image window Original',cv_image)#Show the image
    cv2.waitKey(1)
        #cv2.imshow('Image window Edge',edge_image)#Show the image
        #cv2.waitKey(3)#Refresh the image every 3ms
        #Publish the image
    threshold_image(colour)
        
    

    return colour

def main(args):
    global colour
    colour ='i'
    rospy.init_node('colour_detection',anonymous=True)#Initialize the node with unique id
    image_sub = rospy.Subscriber('/usb_cam/image_raw',Image,image_callback)#Subscribe to the topic
    global image_pub    
    #rate = rospy.Rate(1)
    #rate.sleep()
    image_pub = rospy.Publisher('colour_detection',Int64,queue_size=10)#Publish to the topic
    image_pub.publish(image_sub)#Publish the image
    try:
        rospy.spin()#Run the node until it's shutdown
    except KeyboardInterrupt:#Catch a keyboard interrupt
        print('Shutting down')#Print a message
        cv2.destroyAllWindows()#Close the window






if __name__ == '__main__':
    main(sys.argv)


