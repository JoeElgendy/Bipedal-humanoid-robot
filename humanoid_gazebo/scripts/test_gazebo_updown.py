#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time


def main():
    rospy.init_node("joint_control_node")
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


    q_set = [
        [0, 0, -0.15, 0.3, -0.15, 0, 0, 0, -0.15, 0.3, -0.15, 0],
        [0, 0, -0.6283, 1.2566, -0.6283, 0, 0, 0, -0.6283, 1.2566, -0.6283, 0]
    ]

    def post(q):
        Right_Hip_Joint.publish(q[0])
        Right_Thigh_Joint.publish(q[1])
        Right_Calf_Joint.publish(q[2])
        Right_Foot_Joint.publish(q[3])
        Left_Hip_Joint.publish(q[4])
        Left_Thigh_joint.publish(q[5])

        Left_Calf_Joint.publish(q[6])
        Left_foot_Joint.publish(q[7])
        time.sleep(0.1)
    

    while not rospy.is_shutdown():
        time.sleep(0.1)
        # Pose 0
        Right_Hip_Joint.publish(0.00000)
        Right_Thigh_Joint.publish(0.34890)
        Right_Calf_Joint.publish(0.00000)
        Right_Foot_Joint.publish(0.00000)
        Left_Hip_Joint.publish(0.00000)
        Left_Thigh_joint.publish(-0.00000)
        Left_Calf_Joint.publish(-0.00000)
        Left_foot_Joint.publish(0.00000)
        #time.sleep(0.1)
        
        # Pose 1
        Right_Hip_Joint.publish(0.00000)
        Right_Thigh_Joint.publish(0.26170)
        Right_Calf_Joint.publish(0.26170)
        Right_Foot_Joint.publish(-0.08720)
        Left_Hip_Joint.publish(0.00000)
        Left_Thigh_joint.publish(0.34890)
        Left_Calf_Joint.publish(-1.04700)
        Left_foot_Joint.publish(0.17440)
        #time.sleep(0.1)
        # Pose 2
        Right_Hip_Joint.publish(0.00000)
        Right_Thigh_Joint.publish(0.00000)
        Right_Calf_Joint.publish(0.08720)
        Right_Foot_Joint.publish(0.08720)
        Left_Hip_Joint.publish(0.00000)
        Left_Thigh_joint.publish(0.52350)
        Left_Calf_Joint.publish(-0.52350)
        Left_foot_Joint.publish(0.00000)
        #time.sleep(0.1)  
        #######################################  
        # Pose 3
        Right_Hip_Joint.publish(0.00000)
        Right_Thigh_Joint.publish(-0.26170)
        Right_Calf_Joint.publish(0.00000)
        Right_Foot_Joint.publish(0.00000)
        Left_Hip_Joint.publish(0.00000)
        Left_Thigh_joint.publish(0.52350)
        Left_Calf_Joint.publish(0.0000)
        Left_foot_Joint.publish(0.0000)
        time.sleep(0.1)
        # Pose 4
        Right_Hip_Joint.publish(0.00000)
        Right_Thigh_Joint.publish(-0.26170)
        Right_Calf_Joint.publish(0.52350)
        Right_Foot_Joint.publish(0.34890)
        Left_Hip_Joint.publish(0.00000)
        Left_Thigh_joint.publish(0.34890)
        Left_Calf_Joint.publish(0.0000)
        Left_foot_Joint.publish(0.0000)
        time.sleep(0.1)
        # Pose 5
        Right_Hip_Joint.publish(0.00000)
        Right_Thigh_Joint.publish(0.34890)
        Right_Calf_Joint.publish(1.04710)
        Right_Foot_Joint.publish(0.17440)
        Left_Hip_Joint.publish(0.00000)
        Left_Thigh_joint.publish(0.26170)
        Left_Calf_Joint.publish(-0.26170)
        Left_foot_Joint.publish(0.08720)
        time.sleep(0.1) 
        ######################################
        # Pose 6
        Right_Hip_Joint.publish(0.00000)
        Right_Thigh_Joint.publish(0.52350)
        Right_Calf_Joint.publish(0.52350)
        Right_Foot_Joint.publish(0.00000)
        Left_Hip_Joint.publish(0.00000)
        Left_Thigh_joint.publish(0.0000)
        Left_Calf_Joint.publish(-0.08720)
        Left_foot_Joint.publish(-0.08720)
        time.sleep(0.1)
        # Pose 7
        Right_Hip_Joint.publish(0.00000)
        Right_Thigh_Joint.publish(0.52350)
        Right_Calf_Joint.publish(0.0000)
        Right_Foot_Joint.publish(0.0000)
        Left_Hip_Joint.publish(0.00000)
        Left_Thigh_joint.publish(-0.26170)
        Left_Calf_Joint.publish(0.0000)
        Left_foot_Joint.publish(0.0000)
        time.sleep(0.1)   
   


if __name__ == '__main__':
    main()
