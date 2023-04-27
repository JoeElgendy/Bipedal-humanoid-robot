import rospy 
import numpy as np 
import tf 
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped #represent a point in 3D within specific time 

class DynamicsNode:
    def __init__(self):
        #initialize the ros node 
        rospy.init_node('kinematics_and_dynamics_node')
        #mn URDF parser bn5od kol el robot description ely fil URDF w n-load it ll robot 
        self.robot=URDF.from_parameter_server()
        #empty list 34n n-store fyha el DH parameters
        self.dh_params=[]
        #empty list 34n n-store fyha el joint limits 
        self.joint_limits=[]
        #7y-loop 3la kol joint mn dool
        for joint_name in ['Right_Hip_Joint','Right_Thigh_Joint','Right_Calf_Joint','Right_Foot_Joint','Left_Hip_Joint','Left_Thigh_Joint','Left_Calf_Joint','Left_Foot_Joint']:
            #joint map deh function bnst5dmha 34n el URDF parser by-store el data fy soret map fa2 e7na 34n n-access 7ga bn-accessha menha
            joint= self.robot.joint_map[joint_name]
            #bnstore el limit locally
            limit=joint.limit
            #lw el joint lyh limits fil URDF 7n-store it fy el tuple bt3 el joint_limits
            if limit is not None:
                self.joint_limits.append((limit.lower,limit.upper))
            #hena el joint.origin.xyz ely hya el translation relative to el frame ely ablo 
            #joint.origin.rpy bt-descripe el orientation 
            #joint.axis bt-describe el directions
            self.dh_params.append((joint.origin.xyz,joint.origin.rpy,joint.axis))
        self.joint_state_publisher = rospy.Publisher('/joint_states',JointState,queue_size=10)
        self.zmp_publisher=rospy.Publisher('zmp',PointStamped,queue_size=10)

    def compute_forward_kinematics(self,joint_angles):
        #hena 7n-initate object mn TransformListener --> dah bysm3 en fyh 7ga et8yrt mn mknha 
        tf_listener=tf.TransformListener()
        #hena 7nfdl mstnyen l7d ma2 el transformation y7sal 
        tf_listener.waitForTransform('base_link','left_foot',rospy.Time(0),rospy.Duration(5.0))
        #hena bgyb el translation w el quaternion matrix
        T_base_lfoot=tf_listener.lookupTransform('base_link','left_foot',rospy.Time(0))
        #adrab el etnyn fy ba3d aagyb el Transform
        T_base_lfoot=tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(T_base_lfoot[0]),
                                                            tf.transformations.quaternion_matrix(T_base_lfoot[1])) 
        tf_listener.waitForTransform('base_link','right_foot',rospy.Time(0),rospy.Duration(5.0))
        T_base_rfoot=tf_listener.lookupTransform('base_link','right_foot',rospy.Time(0))
        T_base_rfoot=tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(T_base_rfoot[0]),
                                                            tf.transformations.quaternion_matrix(T_base_rfoot[1])) 
        #nft7 empty list ll transformation bt3 el joints
        T_joints=[np.identity(4),np.identity(4),np.identity(4),np.identity(4)]
        # nft7 empty list ll transformation bt3 el end effectors
        T_ends=[np.identity(4),np.identity(4)]
        # n7seb el transformation matrix
        for i,(xyz,rpy,axis) in enumerate(self.dh_params):
            T_prev_joints=T_joints.copy()
            for j in range(4):
                T_joints[j]=tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(xyz),
                                                                    tf.transformations.euler_matrix(rpy[0],rpy[1],rpy[2]))
                joint_name=f'{["left","right"][j]}_{"hip","thigh","calf","foot"[i]}_joint'
                joint_index=self.robot.joint_map[joint_name].joint_index
                T_joints[j]=tf.transformations.concatenate_matrices(T_joints[j],
                                                                    tf.transformations.rotation_matrix(joint_angles[joint_index],axis))
                T_ends[j]=np.dot(T_ends[j],np.dot(T_prev_joints[j],T_joints[j]))
        
        pos_l=T_ends[0][:3, 3]
        rot_l=T_ends[0][:3,:3]
        pos_r=T_ends[1][:3, 3]
        rot_r=T_ends[1][:3,:3]
        #assuming en el center of mass fil nos bzbt
        com=np.array([0.0,0.0,0.5])

        # ZMP = CoM - (total horizontal ground reaction force / total vertical ground reaction force) * CoM height
        zmp_l = np.array([pos_l[0] - com[0] * (pos_l[2] / 9.81), pos_l[1] - com[1] * (pos_l[2] / 9.81), 0.0])
        zmp_r = np.array([pos_r[0] - com[0] * (pos_r[2] / 9.81), pos_r[1] - com[1] * (pos_r[2] / 9.81), 0.0])
        return pos_l,rot_l,pos_r,rot_r,zmp_l,zmp_r       

    def run(self):
        joint_angles={
            'left_hip_joint': 0.0,
            'left_thigh_joint': 0.0,
            'left_calf_joint': 0.0,
            'left_foot_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_thigh_joint': 0.0,
            'right_calf_joint': 0.0,
            'right_foot_joint': 0.0,
        }
        rate=rospy.rate(10)

        while not rospy.is_shutdown():
            left_position,left_orientation,left_zmp=self.compute_forward_kinematics(joint_angles,leg='left')
            right_position, right_orientation, right_zmp = self.compute_forward_kinematics(joint_angles,leg='right')

            joint_state_msg=JointState()
            joint_state_msg.header.stamp=rospy.Time.now()
            joint_state_msg.name=['left_hip_joint', 'left_thigh_joint', 'left_calf_joint', 'left_foot_joint',
                                'right_hip_joint', 'right_thigh_joint', 'right_calf_joint', 'right_foot_joint']
             joint_state_msg.position = [joint_angles['left_hip_joint'], joint_angles['left_thigh_joint'],
                                    joint_angles['left_calf_joint'], joint_angles['left_foot_joint'],
                                    joint_angles['right_hip_joint'], joint_angles['right_thigh_joint'],
                                    joint_angles['right_calf_joint'], joint_angles['right_foot_joint']]
            joint_state_msg.position = [joint_angles['left_hip_joint'], joint_angles['left_thigh_joint'],
                                    joint_angles['left_calf_joint'], joint_angles['left_foot_joint'],
                                    joint_angles['right_hip_joint'], joint_angles['right_thigh_joint'],
                                    joint_angles['right_calf_joint'], joint_angles['right_foot_joint']]
            self.joint_state_publisher.publish(joint_state_msg)

            zmp_msg=PointStamped()
            zmp_msg.header.stamp=rospy.Time.now()
            zmp_msg.header.frame_id='base_link'
            zmp_msg.point.x=(left_zmp[0] + right_zmp[0]) / 2
            zmp_msg.point.y=(left_zmp[1] + right_zmp[1]) / 2
            zmp_msg.point.z=(left_zmp[2] + right_zmp[2]) / 2

            self.zmp_publisher(zmp_msg)

            rate.sleep()

if __name__ == '__main__':
    kinematics_Dynamics = DynamicsNode()
    kinematics_Dynamics.run()

