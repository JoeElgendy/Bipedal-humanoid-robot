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
//#define foot_x_size 0.03
//#define foot_x_size2 -0.04
//#define foot_y_size 0.018
//#define foot_y_size2 -0.018
//#define stance_value 0.001
//Initializing mass point
class point_mass
{
  public:
    double x = 0, y = 0, z = 0, mass = 0;
    char state;
};
//Initializing Stability point
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
        pb_com,
        pb_centroid,
        pb_com_x,
        pb_centroid_x,
        pb_com_y,
        pb_centroid_y,
        pb_traj;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    //Pointstamped: This represents a Point with reference coordinate frame and timestamp
    geometry_msgs::PointStamped ps, ps_centroid, ps_left_foot, ps_right_foot;
    // initializing humanoid tree we right joint array we left joint array
    KDL::Tree humanoid_tree;
    KDL::JntArray r_jntarray;
    KDL::JntArray l_jntarray;
    // initializing array of home
    double home[8] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    //initializing 2 point masses(humanoid CoM and Centroid)
    point_mass humanoid_CoM, centroid;
    //initializing variables of both (legs, imu and base) roll pitch yaw
    double r_roll, r_pitch, r_yaw,
        l_roll, l_pitch, l_yaw,
        imu_roll, imu_pitch, imu_yaw,
        base_roll, base_pitch, base_yaw;
    //intializing subscribers
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
    //hena ana ba5od el reading bta3t kol ellinks elly ma3molaha subscribe we ba5azenhom fe el right joint array welleft joint array
    void fread_Right_Hip_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_jntarray.data[5] = msg->process_value;
        l_jntarray.data[6] = msg->process_value;
    }
    void fread_Right_Thigh_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_jntarray.data[4] = msg->process_value;
        l_jntarray.data[7] = msg->process_value;
    }
    void fread_Right_Calf_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_jntarray.data[3] = msg->process_value;
        l_jntarray.data[2] = msg->process_value;
    }
    void fread_Right_Foot_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_jntarray.data[1] = msg->process_value;
        l_jntarray.data[0] = msg->process_value;
    }
    void fread_Left_Hip_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_jntarray.data[6] = msg->process_value;
        l_jntarray.data[5] = msg->process_value;
    }
    void fread_Left_Thigh_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_jntarray.data[7] = msg->process_value;
        l_jntarray.data[4] = msg->process_value;
    }
    void fread_Left_Calf_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_jntarray.data[2] = msg->process_value;
        l_jntarray.data[3] = msg->process_value;
    }
    void fread_Left_Foot_Link(const control_msgs::JointControllerState::ConstPtr &msg)
    {
        r_jntarray.data[0] = msg->process_value;
        l_jntarray.data[1] = msg->process_value;
    }
  public:
    bool gazebo = 1,
         real = 0;
    //ana fo2 3araft tree we joint array ! dlwa2ty ba3araf chain we frame
    KDL::Chain right_leg, left_leg;
    KDL::Frame right_foot, left_foot;
    // ba3araf variables  bkoll ellinks btoo3y
    double Right_Hip_Link, Right_Thigh_Link, Right_Calf_Link, Right_Foot_Link;
    double Left_Hip_Link, Left_Thigh_Link, Left_Calf_Link, Left_Foot_Link;
    // ba3araf variables b transformation lkol ellinks btoo3y
    double T_Right_Hip_Link, T_Right_Thigh_Link, T_Right_Calf_Link, T_Right_Foot_Link;
    double T_Left_Hip_Link, T_Left_Thigh_Link, T_Left_Calf_Link, T_Left_Foot_Link;
    //hena ana bady lclass humanoid_kd 5 parameters
    //1.rosnode handle, 2.file path llurdf file, 3. esm elbase link fl urdf, 4. esm e end effectors elly fl urdf
    //ba3dein bat2aked en elcode 3aref y integrate m3 el urdf file
    humanoid_kd(ros::NodeHandle *n, std::string urdf, std::string base, std::string r_endf, std::string l_endf)
    {
        if (!kdl_parser::treeFromFile(urdf, humanoid_tree))
        {
            std::cout << "Failed to construct kdl tree\n";
        }
        else
        {
            std::cout << "Success to construct kdl tree\n";
        }
        if (!uthai_tree.getChain(r_endf, l_endf, right_leg))
        {
            std::cout << "Failed to get right_leg kinematics chain\n";
        }
        if (!uthai_tree.getChain(l_endf, r_endf, left_leg))
        {
            std::cout << "Failed to get left_leg kinematics chain\n";
        }
        else
        {
            std::cout << "Success to get kinematics chain\n";
            r_jntarray = KDL::JntArray(right_leg.getNrOfJoints());
            l_jntarray = KDL::JntArray(left_leg.getNrOfJoints());
        }
        //ba5ali el publishers btoo3y y publisho 3la elmawateer
        pb_Right_Hip_Link = n->advertise<std_msgs::Float64>("/humanoid/Right_Hip_Joint_position/command", 10);
        pb_Right_Thigh_Link = n->advertise<std_msgs::Float64>("/humanoid/Right_Thigh_Joint_position/command", 10);
        pb_Right_Calf_Link = n->advertise<std_msgs::Float64>("/humanoid/Right_Calf_Joint_position/command", 10);
        pb_Right_Foot_Link = n->advertise<std_msgs::Float64>("/humanoid/Right_Foot_Joint_position/command", 10);
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
        //subscribe from joint position states with a buffer size of 1000 we bspecify callback function fread mn class humanoid 3shan tshta8al 
        read_Right_Hip_Link = n->subscribe("/humanoid/Right_Hip_Joint_position/state", 1000, &humanoid_kd::fread_Right_Hip_Link, this);
        read_Right_Thigh_Link = n->subscribe("/humanoid/Right_Thigh_Joint_position/state", 1000, &humanoid_kd::fread_Right_Thigh_Link, this);
        read_Right_Calf_Link = n->subscribe("/humanoid/Right_Calf_Joint_position/state", 1000, &humanoid_kd::fread_Right_Calf_Link, this);
        read_Right_Foot_Link = n->subscribe("/humanoid/Right_Foot_Joint_position/state", 1000, &humanoid_kd::fread_Right_Foot_Link, this);
        read_Left_Hip_Link = n->subscribe("/humanoid/Left_Hip_Joint_position/state", 1000, &humanoid_kd::fread_Left_Hip_Link, this);
        read_Left_Thigh_Link = n->subscribe("/humanoid/Left_Thigh_joint_position/state", 1000, &humanoid_kd::fread_Left_Thigh_Link, this);
        read_Left_Calf_Link = n->subscribe("/humanoid/Left_Calf_Joint_position/state", 1000, &humanoid_kd::fread_Left_Calf_Link, this);
        read_Left_Foot_Link = n->subscribe("/humanoid/Left_foot_Joint_position/state", 1000, &humanoid_kd::fread_Left_Foot_Link, this);
    }
    void set_T_jointpose(double *jnt)
    {
        T_Right_Hip_Link = jnt[0];
        T_Right_Thigh_Link = jnt[1];
        T_Right_Calf_Link = jnt[2];
        T_Right_Foot_Link = 0 - (jnt[2] + jnt[1]);
        T_Left_Hip_Link = jnt[3];
        T_Left_Thigh_Link = jnt[4];
        T_Left_Calf_Link = jnt[5];
        T_Left_Foot_Link = 0 - (jnt[4] + jnt[5]);
    }

    void set_T_equal_jointpose()
    {
        T_Right_Hip_Link = Right_Hip_Link;
        T_Right_Thigh_Link = Right_Thigh_Link;
        T_Right_Calf_Link = Right_Calf_Link;
        T_Right_Foot_Link = Right_Foot_Link;
        T_Left_Hip_Link = Left_Hip_Link;
        T_Left_Thigh_Link = Left_Thigh_Link;
        T_Left_Calf_Link = Left_Calf_Link;
        T_Left_Foot_Link = Left_Foot_Linklhip_r;
        
    }
    void set_jointpose(double *jnt)
    {
        Right_Hip_Link = jnt[0];
        Right_Thigh_Link = jnt[1];
        Right_Calf_Link = jnt[2];
        Right_Foot_Link = 0 - (jnt[2] + jnt[1]);
        Left_Hip_Link = jnt[3];
        Left_Thigh_Link = jnt[4];
        Left_Calf_Link = jnt[5];
        Left_Foot_Link = 0 - (jnt[4] + jnt[5]);
    }
    void add_jointpose(double *jnt)
    {
        Right_Hip_Link += jnt[0];
        Right_Thigh_Link += jnt[1];
        Right_Calf_Link += jnt[2];
        Right_Foot_Link += 0 - (jnt[2] + jnt[1]);
        Left_Hip_Link += jnt[3];
        Left_Thigh_Link += jnt[4];
        Left_Calf_Link += jnt[5];
        Left_Foot_Link += 0 - (jnt[4] + jnt[5]);
    }
    //void auto_ankle(char state)
    //{

      //  if (state == 'r')
        //    lhip_r = rhip_r;
        //else if (state == 'l')
        //    rhip_r = lhip_r;
        //rankle_r = -rhip_r;
        //lankle_r = -lhip_r;

       // rankle_p = 0 - (rhip_p + rknee_p);
       // lankle_p = 0 - (lhip_p + lknee_p);
    //}
    void set_kdjointpose()
    {
        r_jntarray.data[0] = Left_Foot_Link;  // r_hip yaw
        r_jntarray.data[1] = Right_Foot_Link;  // r_hip rol
        r_jntarray.data[2] = Left_Calf_Link;   // r_hip pitch
        r_jntarray.data[3] = Right_Calf_Link;    // r_knee pitch
        r_jntarray.data[4] = Right_Thigh_Link;    // r_ankle pitch
        r_jntarray.data[5] = Right_Hip_Link;    // r_ankle roll
        r_jntarray.data[6] = Left_Hip_Link;    // r_hip yaw
        r_jntarray.data[7] = Left_Thigh_Link;    // r_hip rol


        l_jntarray.data[0] = Right_Foot_Link;  // r_hip yaw
        l_jntarray.data[1] = Left_Foot_Link;  // r_hip rol
        l_jntarray.data[2] = Right_Calf_Link;   // r_hip pitch
        l_jntarray.data[3] = Left_Calf_Link;    // r_knee pitch
        l_jntarray.data[4] = Left_Thigh_Link;    // r_ankle pitch
        l_jntarray.data[5] = Left_Hip_Link;    // r_ankle roll
        l_jntarray.data[6] = Right_Hip_Link;    // r_hip yaw
        l_jntarray.data[7] = Right_Thigh_Link;    // r_hip rol

    }
    //Interface the publishers after the kdl with gazebo
    void joint_publish(ros::Rate *r)
    {
        if (gazebo)
        {
            std_msgs::Float64 radi;
            radi.data = Right_Hip_Link;
            pb_Right_Hip_Link.publish(radi);
            radi.data = Right_Thigh_Link;
            pb_Right_Thigh_Link.publish(radi);
            radi.data = Right_Calf_Link;
            pb_Right_Calf_Link.publish(radi);
            radi.data = Right_Foot_Link;
            pb_Right_Foot_Link.publish(radi);
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
            radi.data = Right_Hip_Link;
            temp.push_back(radi.data);
            radi.data = Right_Thigh_Link;
            temp.push_back(radi.data);
            radi.data = Right_Calf_Link;
            temp.push_back(radi.data);
            radi.data = Right_Foot_Link;
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

    void com_publish(char state,ros::Rate *r){
        ps.point.x = humanoid_CoM.x;
        ps.point.y = humanoid_CoM.y;
        ps.point.z = 0;

        if(state == 'r' || state == 'd')
        {
            ps.header.frame_id= "right_foot_link";
            ps_centroid.header.frame_id="left_foot_link" ; 
        }
// by5od values el ps_centroid mn el values ely stored fil centroid
        ps_centroid.point.x = centroid.x;
        ps_centroid.point.y = centroid.y;
        ps_centroid.point.z = ps.point.z;
        ps_left_foot.header.frame_id= "right_foot_link";

/***********************************************************/
        ps_left_foot.point.x = left_foot.p.data[0];
        ps_left_foot.point.y = left_foot.p.data[1];
        ps_left_foot.point.z = left_foot.p.data[2];
/***********************************************************/

// by5od amaken el points mn el .p.data
// bs hwa bygyb el right_foot.p.data mnen asln ?????!!!!!

/***********************************************************/
        ps_right_foot.header.frame_id = "left_foot_link";
        ps_right_foot.point.x = right_foot.p.data[0];
        ps_right_foot.point.y = right_foot.p.data[1];
        ps_right_foot.point.z = right_foot.p.data[2];
/***********************************************************/
        pb.com.publish(ps);
        pb_centroid.publish(ps_centroid);
        pb_left_foot.publish(ps_left_foot);
        pb_right_foot.publish(ps_right_foot);

        std_msgs::Float64 centroid_x,com_x,centroid_y,com_y;
        centroid_x.data=centroid.x;
        com_x.data=humanoid_CoM.x;
        centroid_y.data=centroid.y;
        com_y.data=humanoid_CoM.y;
        pb_com_x.publish(com_x);
        pb_centroid_x.publish(centroid_x);
        pb_com_y.publish(com_y);
        pb_centroid_y.publish(centroid_y);
        r->sleep();
    }
/* The following function computes the center of mass of the robot
 * State parameter is used to determine which foot to calculate the CoM for
 * right_leg_fk is a pointer to object of type KDL::ChainFkSolverPos_recursive forward kinematics solver for the right leg
 * Left_leg_fk is a pointer to object of type KDL::ChainFkSolverPos_recursive forward kinematics solver for the left leg
 * verbose is used to print the CoM for each segment
 * the function creates two point_mass objects that reperesnt the CoM of the right and left legs and loops over the segments of the right and left legs to calculate the CoM of each segment
 * the JntToCart function of the KDL::ChainFkSolverPos_recursive class is used to calculate the position of each segment
 * left_foot is then used to compute the inverse of raotation matrix of the segment and then position of the segment is rotated to the base frame 
 * the CoG of current segment is stored in link_cog
 * the position is then calculated and stored in link_cog_refbase this position is then added to  right_CoM or left_CoM
 * The CoM of each leg is then divided by the total mass of the leg to get the final CoM of the leg
 * if verbose is true the CoM of each segment is printed (for debugging)
*/
point_mass compute_com(char state, KDL::ChainFkSolverPos_recursive *right_leg_fksolver, KDL::ChainFkSolverPos_recursive *left_leg_fksolver, bool verbose = 0){
    point_mass right_CoM, left_CoM;
    if(state == "r" || state== "d")
    {
        for(int i=0 ; i <right_leg.getNrOfSegments();i++)
        {
            right_leg_fksolver -> JntToCart(right_leg_jnt_pos, left_foot, i+1);
            double r_roll, r_pitch, r_yaw;
            left_foot_fk_pos.M.GetRPY(r_roll, r_pitch, r_yaw);
            KDL::Rotation rot_inv= left_foot_fk_pos.M.Inverse();
            KDL::Vector link_cog = right_leg.getSegment(i).getInertia().getCOG();
            KDL::Vector link_cog_refbase ;
            KDL::Vector link_temp = rot_inv.operator*(left_foot.p);
            link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
            link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
            link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
            right_CoM.x += link_cog_refbase.data[0] * right_leg.getSegment(i).getInertia().getMass();
            right_CoM.y += link_cog_refbase.data[1] * right_leg.getSegment(i).getInertia().getMass();
            right_CoM.z += link_cog_refbase.data[2] * right_leg.getSegment(i).getInertia().getMass();

            if(i==6){
                base_roll = r_roll;
                base_pitch= r_pitch;
                base_yaw = r_yaw;
            }
            
            r_CoM.mass += right_leg.getSegment(i).getInertia().getMass();
        
    
        if(verbose){
            std::cout << std::setprecision(5) << i << ".) state ="
            << "  " << r_leg.getSegment(i).getName() << " mass = " << r_leg.getSegment(i).getInertia().getMass()
            << "        Rotation : " << r_roll << "  " << r_pitch << "  " << r_yaw
            << "           Mass Trans : " << r_CoM.x << "  " << r_CoM.y << "  " << r_CoM.z << ""
            << "       Translation main : " << l_foot.p[0] << "  " << l_foot.p[1] << "  " << l_foot.p[2] << "\n";
        }
        humanoid_CoM.x = r_CoM.x/r_CoM.mass;
        humanoid_CoM.y = r_CoM.y/r_CoM.mass;
    }
    if(state== 'l'){
        for (int = 0 ; i<left_leg.getNrOfSegments();i++){
            left_leg_fksolver -> JntToCart(left_leg_jnt_pos, right_foot, i+1);
            double l_roll, l_pitch, l_yaw;
            right_foot_fk_pos.M.GetRPY(l_roll, l_pitch, l_yaw);
            KDL::Rotation rot_inv= right_foot_fk_pos.M.Inverse();
            KDL::Vector link_cog = left_leg.getSegment(i).getInertia().getCOG();
            KDL::Vector link_cog_refbase ;
            KDL::Vector link_temp = rot_inv.operator*(right_foot.p);
            link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
            link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
            link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
            left_CoM.x += link_cog_refbase.data[0] * left_leg.getSegment(i).getInertia().getMass();
            left_CoM.y += link_cog_refbase.data[1] * left_leg.getSegment(i).getInertia().getMass();
            left_CoM.z += link_cog_refbase.data[2] * left_leg.getSegment(i).getInertia().getMass();
            l_CoM.mass += left_leg.getSegment(i).getInertia().getMass();
            if(verbose){
                std::cout << std::setprecision(5) << i << ".) state ="
                << "  " << l_leg.getSegment(i).getName() << " mass = " << l_leg.getSegment(i).getInertia().getMass()\
                << "        Rotation : " << l_roll << "  " << l_pitch << "  " << l_yaw
                << "            Mass Trans :" << l_CoM.x << "  " << l_CoM.y << "  " << l_CoM.z << ""
                << "       Translation main : " << r_foot.p[0] << "  " << r_foot.p[1] << "  " << r_foot.p[2] << "\n";
            }
            humanoid_CoM.x = l_CoM.x/l_CoM.mass;
            humanoid_CoM.y = l_CoM.y/l_CoM.mass;

        }
    }
    return humanoid_CoM;
}
