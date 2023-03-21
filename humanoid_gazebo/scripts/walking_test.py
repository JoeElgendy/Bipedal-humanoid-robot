#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time

def main():
    rospy.init_node("q_set_control_node")
    
    Right_Hip_Joint = rospy.Publisher(
        '/humanoid/Right_Hip_Joint_position/command', Float64, queue_size=10)
    Right_Thigh_Joint = rospy.Publisher(
        '/humanoid/Right_Thigh_Joint_position/command', Float64, queue_size=10)
    Right_Calf_Joint = rospy.Publisher(
        '/humanoid/Right_Calf_Joint_position/command', Float64, queue_size=10)
    Right_Foot_Joint = rospy.Publisher('/humanoid/Right_Foot_Joint_position/command',
                          Float64, queue_size=10)

    Left_Hip_Joint = rospy.Publisher('/humanoid/Left_Hip_Joint_position/command',
                          Float64, queue_size=10)
    Left_Thigh_joint = rospy.Publisher(
        '/humanoid/Left_Thigh_joint_position/command', Float64, queue_size=10)
    Left_Calf_Joint = rospy.Publisher(
        '/humanoid/Left_Calf_Joint_position/command', Float64, queue_size=10)
    Left_foot_Joint = rospy.Publisher(
        '/humanoid/Left_foot_Joint_position/command', Float64, queue_size=10)


    with open('/home/yousseftarek/catkin_ws/src/humanoid/humanoid_gazebo/scripts/humanoid_poses_v1.csv', 'r') as csvfile:
        spamreader = csvfile.read().split('\n')
        i = 0
        x = 0
        for row in spamreader:
                q_set = map(float, row.split(','))
                            # Right Leg
                Right_Hip_Joint.publish(q_set[0])
                print(q_set[0])
                Right_Thigh_Joint.publish(q_set[1])
                print(q_set[1])
                Right_Calf_Joint.publish(q_set[2])
                print(q_set[2])
                Right_Foot_Joint.publish(q_set[3])
                print(q_set[3])
                            # Left Leg
                Left_Hip_Joint.publish(q_set[4])
                print(q_set[4])
                Left_Thigh_joint.publish(q_set[5])
                print(q_set[5])
                Left_Calf_Joint.publish(q_set[6])
                print(q_set[6])
                Left_foot_Joint.publish(q_set[7])
                print(q_set[7])
                time.sleep(0.5)
                print('\n')
                print(i)
                i = i + 1
                if i==3 :
                    time.sleep(0.5)
                x=x + 1
                time.sleep(0.2)
        time.sleep(5)
        rospy.spin()
                            # Right Leg
        Right_Hip_Joint.publish(0)
        Right_Thigh_Joint.publish(0)
        Right_Calf_Joint.publish(0)
        Right_Foot_Joint.publish(0)
                            # Left Leg
        Left_Hip_Joint.publish(0)
        Left_Thigh_joint.publish(0)
        Left_Calf_Joint.publish(0)
        Left_foot_Joint.publish(0)
if __name__ == '__main__':
    main()