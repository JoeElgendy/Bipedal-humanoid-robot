#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64 , Int64

def callback(data):
    print(data.data)
    if (data.data):
        Right_Upper_Shoulder.publish(0.5)
        #print('waving')
    else:
        Right_Upper_Shoulder.publish(0.0)
        #rospy.sleep(1)
        #print('not waving')
    #rospy.sleep(1)
def main():
    rospy.init_node('waving',anonymous=True)
    global Right_Upper_Shoulder
    Right_Upper_Shoulder = rospy.Publisher('/humanoid/Right_Upper_Shoulder_Joint_position/command', Float64, queue_size=10)
    rospy.Subscriber('colour_detection',Int64,callback)
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
        #if not test:
        #    Right_Upper_Shoulder.publish(0.0)
        #rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    main()