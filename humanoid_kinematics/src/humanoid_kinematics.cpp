/****************************************************************/
/*				Author:Humanoid Graduation Project              */	
/*				Date: 1-5-2023					                */
/*				Version: 1.5					                */
/*				Module:Kinematics				                */
/****************************************************************/
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PointStamped.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "control_msgs/JointControllerState.h"
#include "nav_msgs/Path.h"
#include "tf/transform_datatypes.h"
#include <sstream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <ctime>

using namespace KDL;
// Hany :  find these values -->
#define foot_x_size 0.03
#define foot_x_size2 -0.04
#define foot_y_size 0.018
#define foot_y_size2 -0.018

// #define stance_value 0.001
// Initializing mass point
class point_mass
{
public:
    double x = 0, y = 0, z = 0, mass = 0;
    char state;
};
// Initializing Stability point
class stability
{
public:
    bool x = 0, y = 0;
    bool check()
    {
        return x && y;
    }
};
// hena 3amal class kbeer hy7ot feeh kol elfunctions elly m7tagha
class humanoid_kd
{
    // el topics elly hy3mlaha publish mn elnode di
    ros::Publisher pb_Right_Hip_Link,
        pb_Right_Thigh_Link,
        pb_Right_Calf_Link,
        pb_Right_Foot_Link,
        pb_Left_Hip_Link,
        pb_Left_Thigh_Link,
        pb_Left_Calf_Link,
        pb_Left_Foot_Link,
        pb_Right_Upper_Shoulder_Link,
        pb_Right_Mid_Shoulder_Link,
        pb_Right_Lower_Shoulder_Link,
        pb_Right_Hand_Link,
        pb_Left_Upper_Shoulder_Link,
        pb_Left_Mid_Shoulder_Link,
        pb_Left_Lower_Shoulder_Link,
        pb_Left_Hand_Link,
        pb_chest,
        pb_com,
        pb_centroid,
        pb_com_x,
        pb_centroid_x,
        pb_com_y,
        pb_centroid_y,
        pb_traj; 

    trajectory_msgs::JointTrajectoryPoint traj_point;
    // Pointstamped: This represents a Point with reference coordinate frame and timestamp
    geometry_msgs::PointStamped ps, ps_centroid, ps_l_foot, ps_r_foot, ps_r_hand, ps_l_hand;
    // initializing humanoid tree we right joint array we left joint array
    KDL::Tree humanoid_tree;
    KDL::JntArray r_leg_jntarray;
    KDL::JntArray l_leg_jntarray;
    KDL::JntArray r_arm_jntarray;
    KDL::JntArray l_arm_jntarray;
    // initializing array of home
    double home[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // initializing 2 point masses(humanoid CoM and Centroid)
    point_mass humanoid_CoM, centroid;
    // initializing variables of both (legs, imu and base) roll pitch yaw
    double r_roll, r_pitch, r_yaw,
        l_roll, l_pitch, l_yaw,
        imu_roll, imu_pitch, imu_yaw,
        base_roll, base_pitch, base_yaw;
    // intializing subscribers
    ros::Subscriber sub_imu;
    ros::Subscriber
        read_Right_Hip_Link,
        read_Right_Thigh_Link,
        read_Right_Calf_Link,
        read_Right_Foot_Link,
        read_Left_Hip_Link,
        read_Left_Thigh_Link,
        read_Left_Calf_Link,
        read_Left_Foot_Link,
        read_Right_Upper_Shoulder_Link,
        read_Right_Mid_Shoulder_Link,
        read_Right_Lower_Shoulder_Link,
        read_Left_Upper_Shoulder_Link,
        read_Left_Mid_Shoulder_Link,
        read_Left_Lower_Shoulder_Link,
        read_chest;

    void fread_Right_Upper_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        //joe : check the numbers of the data
        r_arm_jntarray.data[2] = msg->process_value;
        l_arm_jntarray.data[5] = msg->process_value;
    }
    void fread_Right_Mid_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_arm_jntarray.data[1] = msg->process_value;
        l_arm_jntarray.data[4] = msg->process_value;
    }
    void fread_Right_Lower_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_arm_jntarray.data[0] = msg->process_value;
        l_arm_jntarray.data[3] = msg->process_value;
    }
    void fread_Left_Upper_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        //joe : check the numbers of the data
        r_arm_jntarray.data[5] = msg->process_value;
        l_arm_jntarray.data[2] = msg->process_value;
    }
    void fread_Left_Mid_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_arm_jntarray.data[4] = msg->process_value;
        l_arm_jntarray.data[1] = msg->process_value;
    }
    void fread_Left_Lower_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_arm_jntarray.data[3] = msg->process_value;
        l_arm_jntarray.data[0] = msg->process_value;
    }
    void fread_Right_Hip_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        /**
         * The function updates the joint angles of the right and left hip based on a received message.
         *
         * @param msg msg is a pointer to a constant object of type `control_msgs::JointControllerState`.
         * This object contains information about the state of a joint controller, such as the current
         * position, velocity, and effort of the joint. The `ConstPtr` type indicates that the pointer is a
         * shared pointer with const
         */
        r_leg_jntarray.data[5] = msg->process_value;
        l_leg_jntarray.data[6] = msg->process_value;
    }
    void fread_Right_Thigh_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_leg_jntarray.data[4] = msg->process_value;
        l_leg_jntarray.data[7] = msg->process_value;
    }
    void fread_Right_Calf_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_leg_jntarray.data[3] = msg->process_value;
        l_leg_jntarray.data[2] = msg->process_value;
    }
    void fread_Right_Foot_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_leg_jntarray.data[1] = msg->process_value;
        l_leg_jntarray.data[0] = msg->process_value;
    }
    void fread_Left_Hip_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_leg_jntarray.data[6] = msg->process_value;
        l_leg_jntarray.data[5] = msg->process_value;
    }
    void fread_Left_Thigh_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_leg_jntarray.data[7] = msg->process_value;
        l_leg_jntarray.data[4] = msg->process_value;
    }
    void fread_Left_Calf_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_leg_jntarray.data[2] = msg->process_value;
        l_leg_jntarray.data[3] = msg->process_value;
    }
    void fread_Left_Foot_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_leg_jntarray.data[0] = msg->process_value;
        l_leg_jntarray.data[1] = msg->process_value;
    }

public:
    bool gazebo = 1,
         real = 0;
    // ana fo2 3araft tree we joint array ! dlwa2ty ba3araf chain we frame
    KDL::Chain r_leg, l_leg, r_arm,l_arm;
    KDL::Frame r_foot, l_foot,r_hand,l_hand;
    // Right Upper Arm
    double Right_Upper_Shoulder_Link,Right_Mid_Shoulder_Link,Right_Lower_Shoulder_Link,Right_Hand_Link;
    // Left Upper Arm
    double Left_Upper_Shoulder_Link,Left_Mid_Shoulder_Link,Left_Lower_Shoulder_Link,Left_Hand_Link;
    // Right Leg
    double Right_Hip_Link, Right_Thigh_Link, Right_Calf_Link, Right_Foot_Link;
    // Left Leg
    double Left_Hip_Link, Left_Thigh_Link, Left_Calf_Link, Left_Foot_Link;
    // Transformation of Right upper Arm
    double T_Right_Upper_Shoulder_Link, T_Right_Mid_Shoulder_Link, T_Right_Lower_Shoulder_Link,T_Right_Hand_Link;
    // Transformation of Left Upper Arm
    double T_Left_Upper_Shoulder_Link,  T_Left_Mid_Shoulder_Link,  T_Left_Lower_Shoulder_Link,T_Left_Hand_Link;
    // Transformation of Right Legn
    double T_Right_Hip_Link, T_Right_Thigh_Link, T_Right_Calf_Link,T_Right_Foot_Link;
    // Transformation of Left Leg
    double T_Left_Hip_Link,  T_Left_Thigh_Link,  T_Left_Calf_Link, T_Left_Foot_Link;

    // hena ana bady lclass humanoid_kd 5 parameters
    // 1.rosnode handle, 2.file path llurdf file, 3. esm elbase link fl urdf, 4. esm e end effectors elly fl urdf
    // ba3dein bat2aked en elcode 3aref y integrate m3 el urdf file

    humanoid_kd(ros::NodeHandle *n, std::string urdf, std::string base, std::string r_endf, std::string l_endf,std::string r_endh, std::string l_endh)
    {
        /**
         * This function constructs a KDL tree and kinematics chains for a humanoid robot, sets up publishers
         * and subscribers for joint positions and trajectory commands.
         *
         * @param n A pointer to a ROS node handle, used for communication with the ROS system.
         * @param urdf The file path to the URDF (Unified Robot Description Format) file that describes the
         * robot's kinematic and dynamic properties.
         * @param base The name of the base link of the robot in the URDF file.
         * @param r_endf The end effector of the right leg in the kinematics chain.
         * @param l_endf The end effector of the left leg in the kinematics chain.
         * @param r_endh The end effector of the right arm in the kinematics chain.
         * @param l_endh The end effector of the left arm in the kinematics chain.
         */


        if (!kdl_parser::treeFromFile(urdf, humanoid_tree))
        {
            /*
             *if (!kdl_parser::treeFromFile(urdf, humanoid_tree))
             *This function constructs a KDL tree from an URDF file. The tree is a tree of rigid bodies connected by joints.
             *The tree is constructed from the robot description on the ROS parameter server, which is parsed as an URDF document.
             */
            std::cout << "Failed to construct kdl tree\n";
        }
        else
        {
            std::cout << "Success to construct kdl tree\n";
            //std::cout<< humanoid_tree.getSegments(); //Trying to get the name of segments LAST EDIT
            //std::map<std::string,TreeElement>::const_iterator root=humanoid_tree.getRootSegment();
        }

        std::string root_link= humanoid_tree.getRootSegment()->first;
        std::vector<Chain> humanoid_chains;

        KDL::Chain Right_Arm_Chain, Left_Arm_Chain, Right_Leg_Chain, Left_Leg_Chain;
        //Right Arm Chain
        if(!humanoid_tree.getChain(root_link,r_endh,Right_Arm_Chain)){
           std::cout << "Failed to get r_arm kinematics chain\n";
        }

        else{
            humanoid_chains.push_back(Right_Arm_Chain);
        }

        if(!humanoid_tree.getChain(root_link,l_endh,Left_Arm_Chain)){
             std::cout << "Failed to get l_arm kinematics chain\n";
        }
        else
        {
            humanoid_chains.push_back(Left_Arm_Chain);
            std::cout << "Success to get kinematics chain of arm \n";
            r_arm_jntarray=KDL::JntArray(Right_Arm_Chain.getNrOfJoints());
            l_arm_jntarray=KDL::JntArray(Left_Arm_Chain.getNrOfJoints());
            std::cout << Right_Arm_Chain.getNrOfJoints();
            std::cout << Right_Arm_Chain.getNrOfSegments();
        }
        //Leg Chain
        if (!humanoid_tree.getChain(root_link, r_endf, Right_Leg_Chain))
        {
            /*
             * This function gets the chain from the tree. A chain is a subset of a tree. The chain is specified by the base link
             * and the tip link. The chain is constructed by tracing the path in the tree from base to tip. The chain contains
             * all the segments and joints along this path. If no path is found, the chain is not modified and false is returned.
             */
            std::cout << "Failed to get r_leg kinematics chain\n";
        }
        else{
            humanoid_chains.push_back(Right_Leg_Chain);
        }
        if (!humanoid_tree.getChain(root_link, l_endf, Left_Leg_Chain))
        {
            std::cout << "Failed to get l_leg kinematics chain\n";
        }
        else
        {
            humanoid_chains.push_back(Left_Leg_Chain);
            std::cout << "Success to get kinematics chain of leg\n";
            r_leg_jntarray = KDL::JntArray(Right_Leg_Chain.getNrOfJoints()); //(r_leg.getNrOfJoints()):
                                                               // This function gets the number of joints of the right leg chain and then creates a joint array with the same number of joints.
                                                               // The joint array is used to store the joint positions of the right leg.
            l_leg_jntarray = KDL::JntArray(Left_Leg_Chain.getNrOfJoints());
        }

        // publishers
        pb_Right_Upper_Shoulder_Link=n->advertise<std_msgs::Float64>("/humanoid/Right_Upper_Shoulder_Joint_position/command",10);
        pb_Right_Mid_Shoulder_Link=n->advertise<std_msgs::Float64>("/humanoid/Right_Mid_Shoulder_Joint_position/command",10);
        pb_Right_Lower_Shoulder_Link=n->advertise<std_msgs::Float64>("/humanoid/Right_Lower_Shoulder_Joint_position/command",10);
        pb_Right_Hip_Link = n->advertise<std_msgs::Float64>("/humanoid/Right_Hip_Joint_position/command", 10);
        pb_Right_Thigh_Link = n->advertise<std_msgs::Float64>("/humanoid/Right_Thigh_Joint_position/command", 10);
        pb_Right_Calf_Link = n->advertise<std_msgs::Float64>("/humanoid/Right_Calf_Joint_position/command", 10);
        pb_Right_Foot_Link = n->advertise<std_msgs::Float64>("/humanoid/Right_Foot_Joint_position/command", 10);
        pb_Left_Upper_Shoulder_Link=n->advertise<std_msgs::Float64>("/humanoid/Left_Upper_Shoulder_Joint_position/command",10);
        pb_Left_Mid_Shoulder_Link=n->advertise<std_msgs::Float64>("/humanoid/Left_Mid_Shoulder_Joint_position/command",10);
        pb_Left_Lower_Shoulder_Link=n->advertise<std_msgs::Float64>("/humanoid/Left_Lower_Shoulder_Joint_position/command",10);
        pb_Left_Hip_Link = n->advertise<std_msgs::Float64>("/humanoid/Left_Hip_Joint_position/command", 10);
        pb_Left_Thigh_Link = n->advertise<std_msgs::Float64>("/humanoid/Left_Thigh_joint_position/command", 10);
        pb_Left_Calf_Link = n->advertise<std_msgs::Float64>("/humanoid/Left_Calf_Joint_position/command", 10);
        pb_Left_Foot_Link = n->advertise<std_msgs::Float64>("/humanoid/Left_foot_Joint_position/command", 10);
        pb_com = n->advertise<geometry_msgs::PointStamped>("humanoid/com", 10);
        pb_centroid = n->advertise<geometry_msgs::PointStamped>("humanoid/com_target", 10);
        pb_com_x = n->advertise<std_msgs::Float64>("com/x", 10);
        pb_centroid_x = n->advertise<std_msgs::Float64>("centroid/x", 10);
        pb_com_y = n->advertise<std_msgs::Float64>("com/y", 10);
        pb_centroid_y = n->advertise<std_msgs::Float64>("centroid/y", 10);
        pb_traj = n->advertise<trajectory_msgs::JointTrajectory>("humanoid/joint_command", 10);
        // subscribe from joint position states with a buffer size of 1000 we bspecify callback function fread mn class humanoid 3shan tshta8al
        read_Right_Upper_Shoulder_Link=n->subscribe("/humanoid/Right_Upper_Shoulder_Joint_position/state",1000, &humanoid_kd::fread_Right_Upper_Shoulder_Link,this);
        read_Right_Mid_Shoulder_Link=n->subscribe("/humanoid/Right_Mid_Shoulder_Joint_position/state",1000,& humanoid_kd::fread_Right_Mid_Shoulder_Link,this);
        read_Right_Lower_Shoulder_Link=n->subscribe("/humanoid/Right_Lower_Shoulder_Joint_position/state",1000,&humanoid_kd::fread_Right_Lower_Shoulder_Link,this);
        read_Right_Hip_Link = n->subscribe("/humanoid/Right_Hip_Joint_position/state", 1000, &humanoid_kd::fread_Right_Hip_Link, this);
        read_Right_Thigh_Link = n->subscribe("/humanoid/Right_Thigh_Joint_position/state", 1000, &humanoid_kd::fread_Right_Thigh_Link, this);
        read_Right_Calf_Link = n->subscribe("/humanoid/Right_Calf_Joint_position/state", 1000, &humanoid_kd::fread_Right_Calf_Link, this);
        read_Right_Foot_Link = n->subscribe("/humanoid/Right_Foot_Joint_position/state", 1000, &humanoid_kd::fread_Right_Foot_Link, this);
        read_Left_Upper_Shoulder_Link=n->subscribe("/humanoid/Left_Upper_Shoulder_Joint_position/state",1000,&humanoid_kd:: fread_Left_Upper_Shoulder_Link,this);
        read_Left_Mid_Shoulder_Link=n->subscribe("/humanoid/Left_Mid_Shoulder_Joint_position/state",1000,&humanoid_kd::fread_Left_Mid_Shoulder_Link,this);
        read_Left_Lower_Shoulder_Link=n->subscribe("/humanoid/Left_Lower_Shoulder_Joint_postion/state",1000,&humanoid_kd::fread_Left_Lower_Shoulder_Link,this);
        read_Left_Hip_Link = n->subscribe("/humanoid/Left_Hip_Joint_position/state", 1000, &humanoid_kd::fread_Left_Hip_Link, this);
        read_Left_Thigh_Link = n->subscribe("/humanoid/Left_Thigh_joint_position/state", 1000, &humanoid_kd::fread_Left_Thigh_Link, this);
        read_Left_Calf_Link = n->subscribe("/humanoid/Left_Calf_Joint_position/state", 1000, &humanoid_kd::fread_Left_Calf_Link, this);
        read_Left_Foot_Link = n->subscribe("/humanoid/Left_foot_Joint_position/state", 1000, &humanoid_kd::fread_Left_Foot_Link, this);
    }

    void set_T_jointpose(double *jnt)
    {
        /**
         * The function sets the values of several variables based on an input array of joint angles.
         *
         * @param jnt a pointer to an array of 6 doubles representing the joint angles of a humanoid robot's
         * legs. The first three elements represent the right leg joint angles (hip, thigh, and calf), and the
         * last three elements represent the left leg joint angles (hip, thigh, and calf).
         */
        // Joe Check the numbers of the jnts;
        T_Right_Upper_Shoulder_Link=jnt[0];
        T_Right_Mid_Shoulder_Link=jnt[1];
        T_Right_Lower_Shoulder_Link=jnt[2];
        T_Right_Hand_Link= jnt[3];
       // T_Right_Hand_Link= 0 - (jnt[2] + jnt[1]); //revise

        T_Right_Hip_Link = jnt[4];
        T_Right_Thigh_Link = jnt[5];
        T_Right_Calf_Link = jnt[6];
        T_Right_Foot_Link = jnt[7];
        //T_Right_Foot_Link = 0 - (jnt[5] + jnt[6]);

        T_Left_Upper_Shoulder_Link=jnt[8];
        T_Left_Mid_Shoulder_Link=jnt[9];
        T_Left_Lower_Shoulder_Link=jnt[10];
        T_Left_Hand_Link= jnt[11];
        //T_Left_Hand_Link= 0 - (jnt[9] + jnt[10]);

        T_Left_Hip_Link = jnt[12];
        T_Left_Thigh_Link = jnt[13];
        T_Left_Calf_Link = jnt[14];
        T_Left_Foot_Link = jnt[15];
        //T_Left_Foot_Link = 0 - (jnt[13] + jnt[14]);
    }


    void set_T_equal_jointpose()
    {
        /**
         * The function sets the values of several variables equal to their corresponding joint pose
         * values.
         */
        T_Right_Upper_Shoulder_Link=Right_Upper_Shoulder_Link;
        T_Right_Mid_Shoulder_Link=Right_Mid_Shoulder_Link;
        T_Right_Lower_Shoulder_Link=Right_Lower_Shoulder_Link;
        T_Right_Hand_Link=Right_Hand_Link;

        T_Right_Hip_Link = Right_Hip_Link;
        T_Right_Thigh_Link = Right_Thigh_Link;
        T_Right_Calf_Link = Right_Calf_Link;
        T_Right_Foot_Link = Right_Foot_Link;

        T_Left_Upper_Shoulder_Link=Left_Upper_Shoulder_Link;
        T_Left_Mid_Shoulder_Link=Left_Mid_Shoulder_Link;
        T_Left_Lower_Shoulder_Link=Left_Lower_Shoulder_Link;
        T_Left_Hand_Link=Left_Hand_Link;

        T_Left_Hip_Link = Left_Hip_Link;
        T_Left_Thigh_Link = Left_Thigh_Link;
        T_Left_Calf_Link = Left_Calf_Link;
        T_Left_Foot_Link = Left_Foot_Link;
    }

    void set_jointpose(double *jnt)
    {
        /**
         * The function sets the joint pose of a robot's legs based on an array of joint angles.
         *
         * @param jnt a pointer to an array of 6 doubles representing joint angles in radians for a humanoid
         * robot's right and left legs.
         */
        Right_Upper_Shoulder_Link=jnt[0];
        Right_Mid_Shoulder_Link=jnt[1];
        Right_Lower_Shoulder_Link=jnt[2];
        Right_Hand_Link= jnt[3];
        //Right_Hand_Link= 0 - (jnt_h[2] + jnt_h[1]);

        Right_Hip_Link = jnt[4];
        Right_Thigh_Link = jnt[5];
        Right_Calf_Link = jnt[6];
        Right_Foot_Link = jnt[7];
        //Right_Foot_Link = 0 - (jnt_l[2] + jnt_l[1]);

        Left_Upper_Shoulder_Link=jnt[8];
        Left_Mid_Shoulder_Link=jnt[9];
        Left_Lower_Shoulder_Link=jnt[10];
        Left_Hand_Link= jnt[11];
        //Left_Hand_Link= 0 - (jnt_h[5] + jnt_h[4]);

        Left_Hip_Link = jnt[12];
        Left_Thigh_Link = jnt[13];
        Left_Calf_Link = jnt[14];
        Left_Foot_Link = jnt[15];
        //Left_Foot_Link = 0 - (jnt_l[4] + jnt_l[5]);
    }

    void add_jointpose(double *jnt)
    {
        /**
         * The function adds joint positions to various body links in a humanoid model.
         *
         * @param jnt a pointer to an array of 6 doubles representing joint angles in the following order:
         */
        Right_Upper_Shoulder_Link +=jnt[0];
        Right_Mid_Shoulder_Link +=jnt[1];
        Right_Lower_Shoulder_Link +=jnt[2];
        Right_Hand_Link += jnt[3];
        //Right_Hand_Link += 0 - (jnt[2] + jnt[1]);

        Right_Hip_Link += jnt[4];
        Right_Thigh_Link += jnt[5];
        Right_Calf_Link += jnt[6];
        Right_Foot_Link += jnt[7];
        //Right_Foot_Link += 0 - (jnt[2] + jnt[1]);

        Left_Upper_Shoulder_Link +=jnt[8];
        Left_Mid_Shoulder_Link +=jnt[9];
        Left_Lower_Shoulder_Link +=jnt[10];
        Left_Hand_Link += jnt[11];
        //Left_Hand_Link += 0 - (jnt[2] + jnt[1]);

        Left_Hip_Link += jnt[12];
        Left_Thigh_Link += jnt[13];
        Left_Calf_Link += jnt[14];
        Left_Foot_Link += jnt[15];
        //Left_Foot_Link += 0 - (jnt[4] + jnt[5]);
    }
    // void auto_ankle(char state)
    //{

    //  if (state == 'r')
    //    lhip_r = rhip_r;
    // else if (state == 'l')
    //    rhip_r = lhip_r;
    // rankle_r = -rhip_r;
    // lankle_r = -lhip_r;

    // rankle_p = 0 - (rhip_p + rknee_p);
    // lankle_p = 0 - (lhip_p + lknee_p);
    //}

    void set_kdjointpose()
    {
        /**
         * The function sets the joint positions for a humanoid robot's left and right legs. and sets them in a single array of type jntarray
         */
        r_arm_jntarray.data[11]  = Left_Hand_Link;
        r_arm_jntarray.data[3]  = Right_Hand_Link;

        r_arm_jntarray.data[8]  = Left_Upper_Shoulder_Link;
        r_arm_jntarray.data[0]  = Right_Upper_Shoulder_Link;

        r_arm_jntarray.data[9]  = Left_Mid_Shoulder_Link;
        r_arm_jntarray.data[1]  = Right_Mid_Shoulder_Link;

        r_arm_jntarray.data[10]  = Left_Lower_Shoulder_Link;
        r_arm_jntarray.data[2]  = Right_Lower_Shoulder_Link;

        r_leg_jntarray.data[15] = Left_Foot_Link;   // r_hip yaw
        r_leg_jntarray.data[7] = Right_Foot_Link;  // r_hip rol

        r_leg_jntarray.data[14] = Left_Calf_Link;   // r_hip pitch
        r_leg_jntarray.data[6] = Right_Calf_Link;  // r_knee pitch

        r_leg_jntarray.data[5] = Right_Thigh_Link; // r_ankle pitch
        r_leg_jntarray.data[4] = Right_Hip_Link;   // r_ankle roll

        r_leg_jntarray.data[12] = Left_Hip_Link;    // r_hip yaw
        r_leg_jntarray.data[13] = Left_Thigh_Link;  // r_hip rol
 


        l_arm_jntarray.data[3]  = Right_Hand_Link;
        l_arm_jntarray.data[11]  = Left_Hand_Link; 

        l_arm_jntarray.data[0]  = Right_Upper_Shoulder_Link; 
        l_arm_jntarray.data[8]  = Left_Upper_Shoulder_Link;

        l_arm_jntarray.data[1]  = Right_Mid_Shoulder_Link;
        l_arm_jntarray.data[9]  = Left_Mid_Shoulder_Link;

        l_arm_jntarray.data[2]  = Right_Lower_Shoulder_Link;
        l_arm_jntarray.data[10]  = Left_Lower_Shoulder_Link;

        l_leg_jntarray.data[7] = Right_Foot_Link;  // r_hip yaw
        l_leg_jntarray.data[15] = Left_Foot_Link;   // r_hip rol

        l_leg_jntarray.data[6] = Right_Calf_Link;  // r_hip pitch
        l_leg_jntarray.data[14] = Left_Calf_Link;   // r_knee pitch

        l_leg_jntarray.data[13] = Left_Thigh_Link;  // r_ankle pitch
        l_leg_jntarray.data[12] = Left_Hip_Link;    // r_ankle roll

        l_leg_jntarray.data[4] = Right_Hip_Link;   // r_hip yaw
        l_leg_jntarray.data[5] = Right_Thigh_Link; // r_hip rol
    }

    void joint_publish(ros::Rate *r)
    {
        /**
         * The function publishes joint angles to either a Gazebo simulation or a real robot.
         *
         * @param r -> A pointer to a ROS rate object used to control the frequency of publishing joint positions.
         */

        if (gazebo)
        {
            std_msgs::Float64 radi;
            radi.data = Right_Upper_Shoulder_Link;
            pb_Right_Upper_Shoulder_Link.publish(radi);
            radi.data = Right_Mid_Shoulder_Link;
            pb_Right_Mid_Shoulder_Link.publish(radi);
            radi.data = Right_Lower_Shoulder_Link;
            pb_Right_Lower_Shoulder_Link.publish(radi);
            radi.data = Right_Hand_Link;
            pb_Right_Hand_Link.publish(radi);

            radi.data = Right_Hip_Link;
            pb_Right_Hip_Link.publish(radi);
            radi.data = Right_Thigh_Link;
            pb_Right_Thigh_Link.publish(radi);
            radi.data = Right_Calf_Link;
            pb_Right_Calf_Link.publish(radi);
            radi.data = Right_Foot_Link;
            pb_Right_Foot_Link.publish(radi);

            radi.data = Left_Upper_Shoulder_Link;
            pb_Left_Upper_Shoulder_Link.publish(radi);
            radi.data = Left_Mid_Shoulder_Link;
            pb_Left_Mid_Shoulder_Link.publish(radi);
            radi.data = Left_Lower_Shoulder_Link;
            pb_Left_Lower_Shoulder_Link.publish(radi);
            radi.data = Left_Hand_Link;
            pb_Left_Hand_Link.publish(radi);

            radi.data = Left_Hip_Link;
            pb_Left_Hip_Link.publish(radi);
            radi.data = Left_Thigh_Link;
            pb_Left_Thigh_Link.publish(radi);
            radi.data = Left_Calf_Link;
            pb_Left_Calf_Link.publish(radi);
            radi.data = Left_Foot_Link;
            pb_Left_Foot_Link.publish(radi);
        }
        if (real)
        {
            std::vector<double> temp;
            trajectory_msgs::JointTrajectory joint_traj;
            std_msgs::Float64 radi;
            radi.data = Right_Upper_Shoulder_Link;
            temp.push_back(radi.data);
            radi.data = Right_Mid_Shoulder_Link;
            temp.push_back(radi.data);
            radi.data = Right_Lower_Shoulder_Link;
            temp.push_back(radi.data);

            radi.data = Right_Hip_Link;
            temp.push_back(radi.data);
            radi.data = Right_Thigh_Link;
            temp.push_back(radi.data);
            radi.data = Right_Calf_Link;
            temp.push_back(radi.data);
            radi.data = Right_Foot_Link;
            temp.push_back(radi.data);

            radi.data = Left_Upper_Shoulder_Link;
            temp.push_back(radi.data);
            radi.data = Left_Mid_Shoulder_Link;
            temp.push_back(radi.data);
            radi.data = Left_Lower_Shoulder_Link;
            temp.push_back(radi.data);

            radi.data = Left_Hip_Link;
            temp.push_back(radi.data);
            radi.data = Left_Thigh_Link;
            temp.push_back(radi.data);
            radi.data = Left_Calf_Link;
            temp.push_back(radi.data);
            radi.data = Left_Foot_Link;
            traj_point.positions = temp;
            traj_point.time_from_start = ros::Duration(0.03333);
            joint_traj.points.push_back(traj_point);
            pb_traj.publish(joint_traj);
        }
        r->sleep();
    }
// joe: check this fun.
    void com_publish(char state, ros::Rate *r)
    {
        /**
         * This function publishes the position of the center of mass and other related points in a ROS
         * environment.
         *
         * @param state -> a character variable that indicates the state of the robot (either 'right', 'left', or
         * 'double support')
         * @param r> The parameter "r" is a pointer to a ROS Rate object, which is used to control the
         * frequency at which the com_publish function is executed.
         */
        ps.point.x = humanoid_CoM.x;
        ps.point.y = humanoid_CoM.y;
        ps.point.z = 0;

        if (state == 'r' || state == 'd')
        {
            // setting the frame of reference for the points to be published
            // the point will be published in the reference frame of the foot that is in contact with the ground
            // as it's the one being used to calculate the com and the centroid of support polygon
            ps.header.frame_id = "r_foot_ft_link";
            ps_centroid.header.frame_id = "r_foot_ft_link";
        }
        else if (state == 'l')
        {
            ps.header.frame_id = "l_foot_ft_link";
            ps_centroid.header.frame_id = "l_foot_ft_link";
        }
        ps_centroid.point.x = centroid.x;
        ps_centroid.point.y = centroid.y;
        ps_centroid.point.z = ps.point.z;
        ps_l_foot.header.frame_id = "r_foot_ft_link";
        ps_l_foot.point.x = l_foot.p.data[0];
        ps_l_foot.point.y = l_foot.p.data[1];
        ps_l_foot.point.z = l_foot.p.data[2];
        ps_r_foot.header.frame_id = "l_foot_ft_link";
        ps_r_foot.point.x = r_foot.p.data[0];
        ps_r_foot.point.y = r_foot.p.data[1];
        ps_r_foot.point.z = r_foot.p.data[2];
        pb_com.publish(ps);
        pb_centroid.publish(ps_centroid);
        pb_Left_Foot_Link.publish(ps_l_foot);
        pb_Right_Foot_Link.publish(ps_r_foot);

        std_msgs::Float64 centroid_x, com_x, centroid_y, com_y;
        centroid_x.data = centroid.x;
        com_x.data = humanoid_CoM.x;
        centroid_y.data = centroid.y;
        com_y.data = humanoid_CoM.y;
        pb_com_x.publish(com_x);
        pb_centroid_x.publish(centroid_x);
        pb_com_y.publish(com_y);
        pb_centroid_y.publish(centroid_y);
        r->sleep();
    }

    point_mass compute_com(char state, KDL::ChainFkSolverPos_recursive *r_leg_fk_solver, KDL::ChainFkSolverPos_recursive *l_leg_fk_solver,KDL::ChainFkSolverPos_recursive *r_arm_fk_solver,KDL::ChainFkSolverPos_recursive *l_arm_fk_solver, bool verbose = 0)
    {
        /**
         * The function computes the center of mass of a humanoid robot given its state and kinematic chain
         * solvers for the right and left legs.
         *
         * @param state -> The state parameter is a character that specifies which leg to compute the center
         * of mass for. It can take the values 'r' for the right leg, 'l' for the left leg, or 'd' for both
         * legs.
         * @param r_leg_fk_solver -> r_leg_fk_solver is a pointer to a KDL::ChainFkSolverPos_recursive object,
         * which is used to compute the forward kinematics of the right leg of a humanoid robot.
         * @param l_leg_fk_solver -> l_leg_fk_solver is a pointer to a KDL::ChainFkSolverPos_recursive object
         * that is used to compute the forward kinematics of the left leg of a humanoid robot. It takes as
         * input the joint angles of the left leg and returns the position and orientation of the end
         * effector (
         * @param verbose -> A boolean variable that determines whether or not to print out additional
         * information during the computation of the center of mass. If set to true, it will print out
         * information such as the segment name, mass, rotation, and translation. If set to false, it will
         * only return the computed center of mass.
         *
         * @return a point_mass object, which contains the x, y, and z coordinates of the center of mass of
         * the humanoid.
         */
        point_mass r_CoM, l_CoM;
        for(int i=0;i<r_arm.getNrOfSegments();i++){

            r_arm_fk_solver->JntToCart(r_arm_jntarray,r_hand,i+1); // calculate the forward kinematics and stores it in l_hand
            double r_roll,r_pitch,r_yaw;
            r_hand.M.GetRPY(r_roll,r_pitch,r_yaw);
            KDL::Rotation rot_inv=r_hand.M.Inverse();
            KDL::Vector link_cog=r_arm.getSegment(i).getInertia().getCOG();
            KDL::Vector link_cog_refbase;
            KDL::Vector link_temp=rot_inv.operator*(r_hand.p);
            // the COG of the link is computed with respect to the base frame and added to the total COG of the robot
            link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
            link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
            link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
            link_cog_refbase.operator=(r_hand.M.operator*(link_cog_refbase));
            r_CoM.x += link_cog_refbase.data[0] * r_arm.getSegment(i).getInertia().getMass();
            r_CoM.y += link_cog_refbase.data[1] * r_arm.getSegment(i).getInertia().getMass();
            r_CoM.z += link_cog_refbase.data[2] * r_arm.getSegment(i).getInertia().getMass();

            // i changed the if condition to 4 to match the no of links in the Arm
            if(i==4){
                base_roll=r_roll;
                base_pitch=r_pitch;
                base_yaw=r_yaw;
            }
            if(verbose){
                std::cout << std::setprecision(5)<< i<<".) state ="
                          <<" " <<r_arm.getSegment(i).getName()<<" mass =" << r_arm.getSegment(i).getInertia().getMass()
                          <<"     Rotation :" <<r_roll<< " " <<r_pitch<<" "<<r_yaw
                          << "    Mass Trans : " << r_CoM.x << "  " << r_CoM.y << "  " << r_CoM.z << ""
                          <<"     Translation main : "<<r_hand.p[0]<<" "<<r_hand.p[1]<<" "<<r_hand.p[2]<<"\n";

            }
            humanoid_CoM.x = r_CoM.x / r_CoM.mass;
            humanoid_CoM.y = r_CoM.y / r_CoM.mass;

        }
        for(int i=0; i<l_arm.getNrOfSegments();i++){
            l_arm_fk_solver->JntToCart(l_arm_jntarray,l_hand,i+1); // calculate the forward kinematics and stores it in r_hand
            double l_roll,l_pitch,l_yaw;
            l_hand.M.GetRPY(l_roll,l_pitch,l_yaw);
            KDL::Rotation rot_inv=l_hand.M.Inverse();
            KDL::Vector link_cog=l_arm.getSegment(i).getInertia().getCOG();
            KDL::Vector link_cog_refbase;
            KDL::Vector link_temp=rot_inv.operator*(l_hand.p);
            // the COG of the link is computed with respect to the base frame and added to the total COG of the robot
            link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
            link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
            link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
            link_cog_refbase.operator=(l_hand.M.operator*(link_cog_refbase));
            l_CoM.x += link_cog_refbase.data[0] * l_arm.getSegment(i).getInertia().getMass();
            l_CoM.y += link_cog_refbase.data[1] * l_arm.getSegment(i).getInertia().getMass();
            l_CoM.z += link_cog_refbase.data[2] * l_arm.getSegment(i).getInertia().getMass();

            // i changed the if condition to 4 to match the no of links in the Arm

            if(i==4){
                base_roll=l_roll;
                base_pitch=l_pitch;
                base_yaw=l_yaw;
            }
            if(verbose){
                std::cout << std::setprecision(5)<< i<<".) state ="
                          <<" " <<l_arm.getSegment(i).getName()<<" mass =" << l_arm.getSegment(i).getInertia().getMass()
                          <<"     Rotation :" <<l_roll<< " " <<l_pitch<<" "<<l_yaw
                          << "    Mass Trans : " << l_CoM.x << "  " << l_CoM.y << "  " << l_CoM.z << ""
                          <<"     Translation main : "<<l_hand.p[0]<<" "<<l_hand.p[1]<<" "<<l_hand.p[2]<<"\n";

            }
            humanoid_CoM.x += l_CoM.x / l_CoM.mass;
            humanoid_CoM.y += l_CoM.y / l_CoM.mass;
        }
        if (state == 'r' || state == 'd')
        {
            for (int i = 0; i < r_leg.getNrOfSegments(); i++)
            {
                r_leg_fk_solver->JntToCart(r_leg_jntarray, l_foot, i + 1); // calculate the forward kinematics and stores it in l_foot
                double r_roll, r_pitch, r_yaw;
                l_foot.M.GetRPY(r_roll, r_pitch, r_yaw);
                KDL::Rotation rot_inv = l_foot.M.Inverse();                       // calculate the orientation with respect frame
                KDL::Vector link_cog = r_leg.getSegment(i).getInertia().getCOG(); // the center of mass of the link with respect to the link frame
                KDL::Vector link_cog_refbase;                                     // the center of mass of the link with respect to the base frame
                KDL::Vector link_temp = rot_inv.operator*(l_foot.p);
                // the COG of the link is computed with respect to the base frame and added to the total COG of the robot
                link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
                link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
                link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
                link_cog_refbase.operator=(l_foot.M.operator*(link_cog_refbase));
                r_CoM.x += link_cog_refbase.data[0] * r_leg.getSegment(i).getInertia().getMass();
                r_CoM.y += link_cog_refbase.data[1] * r_leg.getSegment(i).getInertia().getMass();
                r_CoM.z += link_cog_refbase.data[2] * r_leg.getSegment(i).getInertia().getMass();

                // i changed the if condition to 4 to match the no of links in the leg
                if (i == 4)
                {
                    base_roll = r_roll;
                    base_pitch = r_pitch;
                    base_yaw = r_yaw;
                }
                if (verbose)
                {
                    std::cout << std::setprecision(5) << i << ".) state = "
                              << "  " << r_leg.getSegment(i).getName() << " mass = " << r_leg.getSegment(i).getInertia().getMass()
                              << "        Rotation : " << r_roll << "  " << r_pitch << "  " << r_yaw
                              << "           Mass Trans : " << r_CoM.x << "  " << r_CoM.y << "  " << r_CoM.z << ""
                              << "       Translation main : " << l_foot.p[0] << "  " << l_foot.p[1] << "  " << l_foot.p[2] << "\n";
                }
                humanoid_CoM.x += r_CoM.x / r_CoM.mass;
                humanoid_CoM.y += r_CoM.y / r_CoM.mass;
            }
        }
        if (state == 'l')
        {
            for (int i = 0; i < l_leg.getNrOfSegments(); i++)
            {
                l_leg_fk_solver->JntToCart(l_leg_jntarray, r_foot, i + 1);
                double l_roll, l_pitch, l_yaw;
                r_foot.M.GetRPY(l_roll, l_pitch, l_yaw);

                KDL::Rotation rot_inv = r_foot.M.Inverse();
                KDL::Vector link_cog = l_leg.getSegment(i).getInertia().getCOG();
                KDL::Vector link_cog_refbase;
                KDL::Vector link_temp = rot_inv.operator*(r_foot.p);
                link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
                link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
                link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
                link_cog_refbase.operator=(r_foot.M.operator*(link_cog_refbase));
                l_CoM.x += link_cog_refbase.data[0] * l_leg.getSegment(i).getInertia().getMass();
                l_CoM.y += link_cog_refbase.data[1] * l_leg.getSegment(i).getInertia().getMass();
                l_CoM.z += link_cog_refbase.data[2] * l_leg.getSegment(i).getInertia().getMass();
                l_CoM.mass += l_leg.getSegment(i).getInertia().getMass();
                if (verbose)
                {
                    std::cout
                        << std::setprecision(5) << i << ".) state = "
                        << "  " << l_leg.getSegment(i).getName() << " mass = " << l_leg.getSegment(i).getInertia().getMass()
                        << "        Rotation : " << l_roll << "  " << l_pitch << "  " << l_yaw
                        << "           Mass Trans : " << l_CoM.x << "  " << l_CoM.y << "  " << l_CoM.z << ""
                        << "       Translation main : " << r_foot.p[0] << "  " << r_foot.p[1] << "  " << r_foot.p[2] << "\n";
                }
                humanoid_CoM.x += l_CoM.x / l_CoM.mass;
                humanoid_CoM.y += l_CoM.y / l_CoM.mass;
            }
        }
        return humanoid_CoM;
    }

    point_mass compute_centroid(char state, bool verbose = 0)
    {
        /**
         * This function computes the centroid of a point mass based on the state of the leg and returns the
         * centroid.
         *
         * @param state The state of the robot, which can be 'r' for right stance, 'l' for left stance, or 'd'
         * for double support.
         * @param verbose A boolean variable that determines whether or not to print out the centroid
         * coordinates. If set to 1, the centroid coordinates will be printed out. If set to 0, nothing will be
         * printed out.
         *
         * @return a variable of type `point_mass`.
         */

        if (state == 'r' || state == 'l') // R leg is the stance
        {
            centroid.x = 0;
            centroid.y = 0;
            centroid.z = 0;
        } 
        else if (state == 'd') // R is main leg for double support
        {
            centroid.x = l_foot.p.data[0] / 2.0;
            centroid.y = l_foot.p.data[1] / 2.0;
            centroid.z = l_foot.p.data[2] / 2.0;
        }
        centroid.state = state;
        if (verbose)
        {
            std::cout << centroid.state << "centroid :(" << centroid.x << "," << centroid.y << "," << centroid.z << ")\n"
                      << std::endl;
        }
        return centroid;
    }

    stability quick_is_stable(bool verbose = 0)
    {
        /**
         * The function checks if the humanoid center of mass is within a stable range of the centroid of the
         * foot.
         * @param verbose A boolean parameter that determines whether or not to print out additional
         * information during the function's execution. If set to true, the function will print out messages
         * indicating whether the humanoid's position is stable in the x and y directions. If set to false, the
         * function will not print out any additional information.
         */
        stability out;
        if (verbose)
        {
            std::cout << "Humanoid_CoM :" << humanoid_CoM.x << "," << humanoid_CoM.y << "\n";
        }
        // if the com is in between the centroid and the centroid + foot size then the humanoid is stable in x direction
        // because the center of mass is in between the front and back of the fot, so the humanoid will not fall forward or backward
        if (centroid.x + foot_x_size2 < humanoid_CoM.x && humanoid_CoM.x < centroid.x + foot_x_size)
        {
            out.x = true;
            if (verbose)
            {
                std::cout << "x is stable\n";
            }

            // if the com is in between the centroid and the centroid + foot size then the humanoid is stable in y direction
            // because the center of mass is in between the left and right of the fot, so the humanoid will not fall left or right
            if (centroid.y + foot_y_size2 < humanoid_CoM.y && humanoid_CoM.y < centroid.y + foot_y_size)
            {
                out.y = true;
                if (verbose)
                {
                    std::cout << "y is stable\n";
                }
            }
        }
    }

    bool humanoid_will_go_on(char state, KDL::ChainFkSolverPos_recursive *r_leg_fk_solver, KDL::ChainFkSolverPos_recursive *l_leg_fk_solver,KDL::ChainFkSolverPos_recursive *r_arm_fk_solver,KDL::ChainFkSolverPos_recursive *l_arm_fk_solver ,ros::Rate *rate, int freq = 200)
    {
        /**
         * The function uses a PID controller to adjust the joint angles of a humanoid robot to maintain
         * stability while walking.
         *
         * @param state a character indicating which foot is on the ground ('r' for right, 'l' for left, 'd'
         * for double support)
         * @param r_lrg_fk_solver A pointer to a KDL::ChainFkSolverPos_recursive object for the right leg of the
         * humanoid robot.
         * @param l_leg_fk_solver A pointer to a KDL::ChainFkSolverPos_recursive object for the left leg of the
         * humanoid robot.
         * @param rate The rate at which the loop runs, in Hz. It is used to calculate the time interval
         * between each iteration of the loop.
         * @param freq The frequency at which the loop runs, in Hz.
         */

        double jntstate[] = {(T_Right_Upper_Shoulder_Link- Right_Upper_Shoulder_Link)/freq,//0
                             (T_Right_Mid_Shoulder_Link-Right_Mid_Shoulder_Link)/freq,//1
                             (T_Right_Lower_Shoulder_Link-Right_Lower_Shoulder_Link)/freq,//2
                             (T_Right_Hand_Link-Right_Hand_Link)/freq,//3

                             (T_Right_Hip_Link - Right_Hip_Link) / freq,//4
                             (T_Right_Thigh_Link - Right_Thigh_Link) / freq,//5
                             (T_Right_Calf_Link - Right_Calf_Link) / freq,//6
                             (T_Right_Foot_Link - Right_Foot_Link) / freq,//7

                             (T_Left_Upper_Shoulder_Link- Left_Upper_Shoulder_Link)/freq,//8
                             (T_Left_Mid_Shoulder_Link-Left_Mid_Shoulder_Link)/freq,//9
                             (T_Left_Lower_Shoulder_Link-Left_Lower_Shoulder_Link)/freq,//10
                             (T_Left_Hand_Link-Left_Hand_Link)/freq,//11

                             (T_Left_Hip_Link - Left_Hip_Link) / freq,//12
                             (T_Left_Thigh_Link - Left_Thigh_Link) / freq,//13
                             (T_Left_Calf_Link - Left_Calf_Link) / freq,//14
                             (T_Left_Foot_Link - Left_Foot_Link) / freq//15
                             };
        stability humanoid_stability;
        // define a boolean variable to check if the humanoid is stable or not
        bool stable = false;
        // define the PID controller parameters
        double Kp = 20.0 / freq,
               Ki = 1.5 / freq,
               Kd = 0.0 / freq,
               error_x = 0.0,
               error_y = 0.0,
               derivative_error_x = 0.0,
               derivative_error_y = 0.0,
               integral_error_x = 0.0,
               integral_error_y = 0.0,
               P_limit = 0.2 / freq,
               I_limit = 4.0 / freq,
               D_limit = 200000.0 / freq;
        for (int k = 0; k < freq; k++)
        {
            // add the step size to the joint angles
            add_jointpose(jntstate);
            // move the ankle joints to maintain the humanoid's balance
            //  auto_ankle(state);
            // set the kdjointpose variable to the new joint angles
            set_kdjointpose();
            // compute the new center of mass and centroid
            humanoid_CoM = compute_com(state, r_leg_fk_solver,l_leg_fk_solver,r_arm_fk_solver,l_arm_fk_solver, 0);
            centroid = compute_centroid(state, 0);
            // check if the humanoid is stable
            humanoid_stability = quick_is_stable();
            stable = humanoid_stability.check();
            // caclulate the error in the x and y directions
            error_x = humanoid_CoM.x - centroid.x;
            error_y = humanoid_CoM.y - centroid.y;
            // calculate the derivative error in the x and y directions
            derivative_error_x = (humanoid_CoM.x - centroid.x) - error_x;
            derivative_error_y = (humanoid_CoM.y - centroid.y) - error_y;
            // calculate the integral error in the x and y directions
            integral_error_x += error_x;
            integral_error_y += error_y;

            double P_x = (Kp * error_x),
                   P_y = (Kp * error_y),
                   I_x = (Ki * integral_error_x),
                   I_y = (Ki * integral_error_y),
                   D_x = (Kd * derivative_error_x),
                   D_y = (Kd * derivative_error_y);
            // Setting the maximum limits for the Propotional controller on x
            if (P_x > P_limit)
            {
                P_x = P_limit;
            }
            else if (P_x < -P_limit)
            {
                P_x = -P_limit;
            }
            // Setting the maximum limits for the Propotional controller on y
            if (P_y > P_limit)
            {
                P_y = P_limit;
            }
            else if (P_y < -P_limit)
            {
                P_y = -P_limit;
            }
            // Setting the maximum limits for the Integral controller on x
            if (I_x > I_limit)
            {
                I_x = I_limit;
            }
            else if (I_x < -I_limit)
            {
                I_x = -I_limit;
            }
            // Setting the maximum limits for the Integral controller on y
            if (I_y > I_limit)
            {
                I_y = I_limit;
            }
            else if (I_y < -I_limit)
            {
                I_y = -I_limit;
            }
            // Setting the maximum limits for the Derivative controller on x
            if (D_x > D_limit)
            {
                D_x = D_limit;
            }
            else if (D_x < -D_limit)
            {
                D_x = -D_limit;
            }
            // Setting the maximum limits for the Derivative controller on y
            if (D_y > D_limit)
            {
                D_y = D_limit;
            }
            else if (D_y < -D_limit)
            {
                D_y = -D_limit;
            }
            // calculate the new joint angles for the PID controller
            if (state == 'r')
            {
                Right_Hip_Link += P_y + I_y + D_y;   // 
                Right_Thigh_Link -= P_x + I_x + D_x; // 

               /* if (l_foot.p.data[1] < 0.16)
                {
                    Left_Hip_Link += Kp / 3 * (0.16 - l_foot.p.data[1]);
                }*/
            }
            else if (state == 'l')
            {
                Left_Hip_Link += P_y + I_y + D_y;   //
                Left_Thigh_Link -= P_x + I_x + D_x; // 

               /* if (r_foot.p.data[1] > -0.16)
                {
                    Right_Hip_Link -= Kp / 3 * (-r_foot.p.data[1] + 0.16);
                }*/
            }

            else if (state == 'd')
            {
                Right_Hip_Link += (P_y + I_y + D_y);
                Right_Thigh_Link -= (P_x + I_x + D_x);
                Left_Hip_Link += (P_y + I_y + D_y);
                Left_Thigh_Link -= (P_x + I_x + D_x);
            }
            joint_publish(rate);
            com_publish(state, rate);
        }
        set_T_equal_jointpose();
    }

    bool moveleg(double *target, char state, KDL::ChainFkSolverPos_recursive *r_leg_fk_solver, KDL::ChainFkSolverPos_recursive *l_leg_fk_solver, KDL::ChainFkSolverPos_recursive *r_arm_fk_solver,KDL::ChainFkSolverPos_recursive *l_arm_fk_solver,ros::Rate *rate, int freq = 200)
    {
        /**
         * The function implements a PID controller to move the legs of a humanoid robot towards a target
         * position while maintaining stability.
         *
         * @param target A pointer to an array of doubles representing the desired position of the foot in
         * Cartesian coordinates (x, y, z).
         * @param state A character representing the current state of the humanoid robot ('r' for right
         * foot support, 'l' for left foot support, 'd' for double support).
         * @param r_leg_fk_solver A pointer to a KDL::ChainFkSolverPos_recursive object for the right leg.
         * @param l_leg_fk_solver A pointer to a KDL::ChainFkSolverPos_recursive object for the left leg.
         * @param rate The rate at which the loop runs, in Hz (Hertz). It determines how often the loop
         * updates and the robot moves.
         * @param freq The frequency of the loop in Hz. It determines how many times the loop will run per
         * second.
         */
        double jntstate[] = {(T_Right_Upper_Shoulder_Link-Right_Upper_Shoulder_Link)/freq,
                             (T_Right_Mid_Shoulder_Link-Right_Mid_Shoulder_Link)/freq,
                             (T_Right_Lower_Shoulder_Link-Right_Lower_Shoulder_Link)/freq,
                             (T_Right_Hip_Link - Right_Hip_Link) / freq,
                             (T_Right_Thigh_Link - Right_Thigh_Link) / freq,
                             (T_Right_Calf_Link - Right_Calf_Link) / freq,
                             (T_Right_Foot_Link - Right_Foot_Link) / freq,
                             (T_Left_Upper_Shoulder_Link-Left_Upper_Shoulder_Link)/freq,
                             (T_Left_Mid_Shoulder_Link-Left_Mid_Shoulder_Link)/freq,
                             (T_Left_Lower_Shoulder_Link-Left_Lower_Shoulder_Link)/freq,
                             (T_Left_Hip_Link - Left_Hip_Link) / freq,
                             (T_Left_Thigh_Link - Left_Thigh_Link) / freq,
                             (T_Left_Calf_Link - Left_Calf_Link) / freq,
                             (T_Left_Foot_Link - Left_Foot_Link) / freq};
        stability humanoid_stability;
        bool stable = false;
        double Kp = 30.0 / freq,
               Ki = 1.5 / freq,
               Kd = 0.0 / freq,
               error_x = 0.0,
               error_y = 0.0,
               derivative_error_x = 0.0,
               derivative_error_y = 0.0,
               integral_error_x = 0.0,
               integral_error_y = 0.0,
               te_x = 0.0,
               te_y = 0.0,
               te_z = 0.0,
               P_limit = 0.2 / freq,
               I_limit = 4.0 / freq,
               D_limit = 200000.0 / freq;
        for (int k = 0; k < freq; k++)
        {
            add_jointpose(jntstate);
            // 7arketha el mfrod tba 3ks 7arket el thigh w el calf 34n t-compensate
            Right_Foot_Link = 0 - (Right_Thigh_Link + Right_Calf_Link);
            Left_Foot_Link = 0 - (Left_Thigh_Link + Left_Calf_Link);

            set_kdjointpose();
            humanoid_CoM = compute_com(state, r_leg_fk_solver, l_leg_fk_solver,r_arm_fk_solver,l_arm_fk_solver, 0);
            centroid = compute_centroid(state, 0);
            if (state == 'l')
            {
                centroid.y -= 0.02;
            }
            if (state == 'r')
            {
                centroid.y += 0.02;
            }
            humanoid_stability = quick_is_stable();
            stable = humanoid_stability.check();
            derivative_error_x = (humanoid_CoM.x - centroid.x) - error_x;
            derivative_error_y = (humanoid_CoM.y - centroid.y) - error_y;
            error_x = humanoid_CoM.x - centroid.x;
            error_y = humanoid_CoM.y - centroid.y;
            integral_error_x += error_x;
            integral_error_y += error_y;
            double P_x = (Kp * error_x),
                   P_y = (Kp * error_y),
                   I_x = (Ki * integral_error_x),
                   I_y = (Ki * integral_error_y),
                   D_x = (Kd * derivative_error_x),
                   D_y = (Kd * derivative_error_y);
            // Setting the maximum limits for the Proportional controller on x
            if (P_x > P_limit)
            {
                P_x = P_limit;
            }
            else if (P_x < -P_limit)
            {
                P_x = -P_limit;
            }
            // Setting the maximum limits for the Proportional controller on y
            if (P_y > P_limit)
            {
                P_y = P_limit;
            }
            else if (P_y < -P_limit)
            {
                P_y = -P_limit;
            }
            // Setting the maximum limits for the Integral controller on x
            if (I_x > I_limit)
            {
                I_x = I_limit;
            }
            else if (I_x < -I_limit)
            {
                I_x = -I_limit;
            }
            // Setting the maximum limits for the Integral controller on y
            if (I_y > I_limit)
            {
                I_y = I_limit;
            }
            else if (I_y < -I_limit)
            {
                I_y = -I_limit;
            }
            // Setting the maximum limits for the Derivative controller on x
            if (D_x > D_limit)
            {
                D_x = D_limit;
            }
            else if (D_x < -D_limit)
            {
                D_x = -D_limit;
            }
            // Setting the maximum limits for the Derivative controller on y
            if (D_y > D_limit)
            {
                D_y = D_limit;
            }
            else if (D_y < -D_limit)
            {
                D_y = -D_limit;
            }
            if (state == 'r')
            {
                te_x = l_foot.p.data[0] - target[0];
                te_y = l_foot.p.data[1] - target[1];
                te_z = l_foot.p.data[2] - target[2];
                Right_Hip_Link += (P_y + I_y + D_y);
                Right_Thigh_Link -= (P_x + I_x + D_x);

                if (l_foot.p.data[1] < 0.16)
                {
                    Left_Hip_Link += Kp / 3 * (0.16 - l_foot.p.data[1]);
                }
                Left_Hip_Link -= Kp * te_y;
                Left_Thigh_Link += Kp * te_x;
                Left_Calf_Link -= Kp * te_z;
            }
            else if (state == 'l')
            {
                te_x = r_foot.p.data[0] - target[0];
                te_y = r_foot.p.data[1] - target[1];
                te_z = r_foot.p.data[2] - target[2];
                Left_Hip_Link += (P_y + I_y + D_y);
                Left_Thigh_Link -= (P_x + I_x + D_x);
                if (r_foot.p.data[1] > -0.16)
                {
                    Right_Hip_Link += Kp / 3 * (-0.16 - r_foot.p.data[1]);
                }
                Right_Hip_Link -= Kp * te_y;
                Right_Thigh_Link += Kp * te_x;
                Right_Calf_Link -= Kp * te_z;
            }
            else if (state == 'd')
            {
                Right_Hip_Link += (P_y + I_y + D_y);
                Right_Thigh_Link -= (P_x + I_x + D_x);
                Left_Hip_Link += (P_y + I_y + D_y);
                Left_Thigh_Link -= (P_x + I_x + D_x);
            }
            joint_publish(rate);
            com_publish(state, rate);
        }
        set_T_equal_jointpose();
    }
};

void get_path(const nav_msgs::Path::ConstPtr &msg, int sampling, ros::Rate *rate, humanoid_kd *humanoid, KDL::ChainFkSolverPos_recursive *r_leg_fk_solver, KDL::ChainFkSolverPos_recursive *l_leg_fk_solver,KDL::ChainFkSolverPos_recursive *r_arm_fk_solver,KDL::ChainFkSolverPos_recursive *l_arm_fk_solver)
{
    /**
     * This function takes in a path message and uses it to control a humanoid robot's movements.
     *
     * @param msg A pointer to a constant object of type nav_msgs::Path, which contains a sequence of poses
     * representing a path for the humanoid robot to follow.
     * @param sampling The sampling parameter is an integer value that determines the frequency at which
     * the robot's movements are updated. It is used to control the rate at which the robot moves and to
     * ensure that the robot's movements are smooth and consistent.
     * @param rate A pointer to a ROS rate object, which controls the frequency at which the loop runs.
     * @param humanoid It is an object of the class humanoid_kd, which contains the methods and variables
     * necessary to control the humanoid robot's movements.
     * @param r_leg_fk_solver A pointer to a KDL::ChainFkSolverPos_recursive object for the right leg of the
     * humanoid robot.
     * @param l_leg_fk_solver KDL::ChainFkSolverPos_recursive object for the left leg of the humanoid robot
     */

    for (int idx = 2; idx < msg->poses.size(); idx++)
    {
        // getting the position and orientation of the current pose
        double x = msg->poses[idx].pose.position.x,
               y = msg->poses[idx].pose.position.y,
               roll, pitch, yaw;
        tf::Quaternion q;
        q.setX(msg->poses[idx].pose.orientation.x);
        q.setY(msg->poses[idx].pose.orientation.y);
        q.setZ(msg->poses[idx].pose.orientation.z);
        q.setW(msg->poses[idx].pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        // getting the frame_id of the current pose
        char foot = msg->poses[idx].header.frame_id[0];
        char foot_stance;
        if (foot == 'r')
        {
            foot_stance = 'l';
        }
        if (foot == 'l')
        {
            foot_stance = 'r';
        }
        std::cout << idx << ".)  x = " << x << "   , y = " << y << "   , yaw = " << yaw << "   , foot = " << foot << "\n";
        // calling the humanoid_will_go_on function to move the humanoid robot
        humanoid->humanoid_will_go_on(foot_stance, r_leg_fk_solver, l_leg_fk_solver,r_arm_fk_solver,l_arm_fk_solver, rate, sampling * 1.5);
        if (foot_stance == 'r')
        {
            humanoid->T_Right_Thigh_Link = -0.3;
            humanoid->T_Right_Calf_Link = 0.55;

            humanoid->T_Left_Thigh_Link = -0.2;
            humanoid->T_Left_Calf_Link = 0.5;

            humanoid->T_Left_Hip_Link = yaw;
            humanoid->T_Right_Hip_Link = 0;
        }
        else if (foot_stance == 'l')
        {
            humanoid->T_Left_Thigh_Link = -0.3;
            humanoid->T_Left_Calf_Link = 0.55;

            humanoid->T_Right_Thigh_Link = -0.2;
            humanoid->T_Right_Calf_Link = 0.5;

            humanoid->T_Right_Hip_Link = yaw;
            humanoid->T_Left_Hip_Link = 0;
        }
        humanoid->humanoid_will_go_on(foot_stance, r_leg_fk_solver, l_leg_fk_solver,r_arm_fk_solver,l_arm_fk_solver, rate, sampling);
        double tfoot[] = {x, y, 0.0};
        humanoid->moveleg(tfoot, foot_stance, r_leg_fk_solver, l_leg_fk_solver,r_arm_fk_solver,l_arm_fk_solver, rate, sampling * 2);
        humanoid->humanoid_will_go_on('d', r_leg_fk_solver, l_leg_fk_solver,r_arm_fk_solver,l_arm_fk_solver, rate, sampling);
    }
    humanoid->T_Right_Thigh_Link = -0.3;
    humanoid->T_Right_Calf_Link = 0.55;
    humanoid->T_Left_Thigh_Link = -0.3;
    humanoid->T_Left_Calf_Link = 0.55;
    humanoid->humanoid_will_go_on('d', r_leg_fk_solver,l_leg_fk_solver, r_arm_fk_solver,l_arm_fk_solver, rate, 100);
    std::cout << "Finish! \n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "humanoid_kd");
    ros::NodeHandle nh;
    ros::Rate rate(100); // try 30,100,80
    std::string urdf_file;
    nh.getParam("urdf_file", urdf_file);
    // el function el gya 7tdrb error 34n m3nda4 7ga esmha base link
    humanoid_kd humanoid(&nh, urdf_file, "base_footprint", "Right_Foot_Link", "Left_foot_Link","Right_Lower_Shoulder_link","Left_Lower_Shoulder_Link");
    KDL::ChainFkSolverPos_recursive r_leg_fk_solver(humanoid.r_leg);
    KDL::ChainFkSolverPos_recursive l_leg_fk_solver(humanoid.l_leg);
    KDL::ChainFkSolverPos_recursive r_arm_fk_solver(humanoid.r_arm);
    KDL::ChainFkSolverPos_recursive l_arm_fk_solver(humanoid.l_arm);
    nav_msgs::Path ptest;
    int sampling = 200;
    // check el path bta3 el footstep
    ros::Subscriber sub_path = nh.subscribe<nav_msgs::Path>("humanoid/footstep_path", 100, boost::bind(get_path, _1, sampling, &rate, &humanoid, &r_leg_fk_solver, &l_leg_fk_solver,&r_arm_fk_solver,&l_arm_fk_solver));

    double home[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    humanoid.set_jointpose(home);
    humanoid.set_T_jointpose(home);
    humanoid.humanoid_will_go_on('d', &r_leg_fk_solver, &l_leg_fk_solver,&r_arm_fk_solver,&l_arm_fk_solver, &rate, 70);

    humanoid.T_Right_Thigh_Link = -0.3;
    humanoid.T_Right_Calf_Link = 0.55;
    humanoid.T_Left_Thigh_Link = -0.3;
    humanoid.T_Left_Calf_Link = 0.55;
    humanoid.humanoid_will_go_on('d', &r_leg_fk_solver, &l_leg_fk_solver,&r_arm_fk_solver,&l_arm_fk_solver,&rate, 100);

    std::clock_t begin = clock();
    ros::spin();
    std::clock_t end = clock();
    std::cout << "elapsed time is " << double(end - begin) / CLOCKS_PER_SEC << "\n";
    return 1;
}
