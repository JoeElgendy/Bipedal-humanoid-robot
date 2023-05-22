#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory
import PyKDL as KDL
import PyKDLParser as KDLParser
import geometry_msgs.msg

class KinematicsFull:
    def __init__(self, n, urdf, base, Right_Leg_End_Effector, Left_Leg_End_Effector, Right_Arm_End_Effector, Left_Arm_End_Effector):
        self.pb_Right_Upper_Shoulder_Link = rospy.Publisher("/humanoid/Right_Upper_Shoulder_Joint_position/command", Float64, queue_size=10)
        self.pb_Right_Mid_Shoulder_Link = rospy.Publisher("/humanoid/Right_Mid_Shoulder_Joint_position/command", Float64, queue_size=10)
        self.pb_Right_Lower_Shoulder_Link = rospy.Publisher("/humanoid/Right_Lower_Shoulder_Joint_position/command", Float64, queue_size=10)
        self.pb_Right_Hip_Link = rospy.Publisher("/humanoid/Right_Hip_Joint_position/command", Float64, queue_size=10)
        self.pb_Right_Thigh_Link = rospy.Publisher("/humanoid/Right_Thigh_Joint_position/command", Float64, queue_size=10)
        self.pb_Right_Calf_Link = rospy.Publisher("/humanoid/Right_Calf_Joint_position/command", Float64, queue_size=10)
        self.pb_Right_Foot_Link = rospy.Publisher("/humanoid/Right_Foot_Joint_position/command", Float64, queue_size=10)
        self.pb_Left_Upper_Shoulder_Link = rospy.Publisher("/humanoid/Left_Upper_Shoulder_Joint_position/command", Float64, queue_size=10)
        self.pb_Left_Mid_Shoulder_Link = rospy.Publisher("/humanoid/Left_Mid_Shoulder_Joint_position/command", Float64, queue_size=10)
        self.pb_Left_Lower_Shoulder_Link = rospy.Publisher("/humanoid/Left_Lower_Shoulder_Joint_position/command", Float64, queue_size=10)
        self.pb_Left_Hip_Link = rospy.Publisher("/humanoid/Left_Hip_Joint_position/command", Float64, queue_size=10)
        self.pb_Left_Thigh_Link = rospy.Publisher("/humanoid/Left_Thigh_joint_position/command", Float64, queue_size=10)
        self.pb_Left_Calf_Link = rospy.Publisher("/humanoid/Left_Calf_Joint_position/command", Float64, queue_size=10)
        self.pb_Left_Foot_Link = rospy.Publisher("/humanoid/Left_foot_Joint_position/command", Float64, queue_size=10)
        self.pb_com = rospy.Publisher("humanoid/com", geometry_msgs.msg.PointStamped, queue_size=10)
        self.pb_centroid = rospy.Publisher("humanoid/com_target", geometry_msgs.msg.PointStamped, queue_size=10)
        self.pb_com_x = rospy.Publisher("humanoid/com/x", Float64, queue_size=10)
        self.pb_centroid_x = rospy.Publisher("humanoid/centroid/x", Float64, queue_size=10)
        self.pb_com_y = rospy.Publisher("humanoid/com/y", Float64, queue_size=10)
        self.pb_centroid_y = rospy.Publisher("humanoid/centroid/y", Float64, queue_size=10)
        self.pb_traj = rospy.Publisher("humanoid/joint_command", JointTrajectory, queue_size=10)
        
        self.read_Right_Upper_Shoulder_Link = rospy.Subscriber("/humanoid/Right_Upper_Shoulder_Joint_position/state", Float64, self.fread_Right_Upper_Shoulder_Link, queue_size=1000)
        self.read_Right_Mid_Shoulder_Link = rospy.Subscriber("/humanoid/Right_Mid_Shoulder_Joint_position/state", Float64, self.fread_Right_Mid_Shoulder_Link, queue_size=1000)
        self.read_Right_Lower_Shoulder_Link = rospy.Subscriber("/humanoid/Right_Lower_Shoulder_Joint_position/state", Float64, self.fread_Right_Lower_Shoulder_Link, queue_size=1000)
        self.read_Right_Hip_Link = rospy.Subscriber("/humanoid/Right_Hip_Joint_position/state", Float64, self.fread_Right_Hip_Link, queue_size=1000)
        self.read_Right_Thigh_Link = rospy.Subscriber("/humanoid/Right_Thigh_Joint_position/state", Float64, self.fread_Right_Thigh_Link, queue_size=1000)
        self.read_Right_Calf_Link = rospy.Subscriber("/humanoid/Right_Calf_Joint_position/state", Float64, self.fread_Right_Calf_Link, queue_size=1000)
        self.read_Right_Foot_Link = rospy.Subscriber("/humanoid/Right_Foot_Joint_position/state", Float64, self.fread_Right_Foot_Link, queue_size=1000)
        self.read_Left_Upper_Shoulder_Link = rospy.Subscriber("/humanoid/Left_Upper_Shoulder_Joint_position/state", Float64, self.fread_Left_Upper_Shoulder_Link, queue_size=1000)
        self.read_Left_Mid_Shoulder_Link = rospy.Subscriber("/humanoid/Left_Mid_Shoulder_Joint_position/state", Float64, self.fread_Left_Mid_Shoulder_Link, queue_size=1000)
        self.read_Left_Lower_Shoulder_Link = rospy.Subscriber("/humanoid/Left_Lower_Shoulder_Joint_postion/state", Float64, self.fread_Left_Lower_Shoulder_Link, queue_size=1000)
        self.read_Left_Hip_Link = rospy.Subscriber("/humanoid/Left_Hip_Joint_position/state", Float64, self.fread_Left_Hip_Link, queue_size=1000)
        self.read_Left_Thigh_Link = rospy.Subscriber("/humanoid/Left_Thigh_joint_position/state", Float64, self.fread_Left_Thigh_Link, queue_size=1000)
        self.read_Left_Calf_Link = rospy.Subscriber("/humanoid/Left_Calf_Joint_position/state", Float64, self.fread_Left_Calf_Link, queue_size=1000)
        self.read_Left_Foot_Link = rospy.Subscriber("/humanoid/Left_foot_Joint_position/state", Float64, self.fread_Left_Foot_Link, queue_size=1000)
        
        self.humanoid_tree = KDLParser.Tree()
        
        if not self.humanoid_tree.initFromFile(urdf):
            print("Failed to construct KDL tree")
        else:
            print("Succeeded to construct KDL tree")
        
        root_link = self.humanoid_tree.getRootSegment().first
        self.humanoid_chains = []
        
        Right_Arm_Chain = KDL.Chain()
        Left_Arm_Chain = KDL.Chain()
        Right_Leg_Chain = KDL.Chain()
        Left_Leg_Chain = KDL.Chain()
        
        if not self.humanoid_tree.getChain(root_link, Right_Leg_End_Effector, Right_Leg_Chain):
            print("Failed to construct Right Leg Chain")
        else:
            print("Succeeded to construct Right Leg Chain")
            self.humanoid_chains.append(Right_Leg_Chain)
        
        if not self.humanoid_tree.getChain(root_link, Left_Leg_End_Effector, Left_Leg_Chain):
            print("Failed to construct Left Leg Chain")
        else:
            print("Succeeded to construct Left Leg Chain")
            self.humanoid_chains.append(Left_Leg_Chain)
            self.Right_Leg_jntarray = KDL.JntArray(Left_Leg_Chain.getNrOfJoints())
            self.Left_Leg_jntarray = KDL.JntArray(Left_Leg_Chain.getNrOfJoints())
        
        if not self.humanoid_tree.getChain(root_link, Right_Arm_End_Effector, Right_Arm_Chain):
            print("Failed to construct Right Arm Chain")
        else:
            print("Succeeded to construct Right Arm Chain")
            self.humanoid_chains.append(Right_Arm_Chain)
        
        if not self.humanoid_tree.getChain(root_link, Left_Arm_End_Effector, Left_Arm_Chain):
            print("Failed to construct Left Arm Chain")
        else:
            print("Succeeded to construct Left Arm Chain")
            self.humanoid_chains.append(Left_Arm_Chain)
            self.Right_Arm_jntarray = KDL.JntArray(Left_Arm_Chain.getNrOfJoints())
            self.Left_Arm_jntarray = KDL.JntArray(Left_Arm_Chain.getNrOfJoints())
