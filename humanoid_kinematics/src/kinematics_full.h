/****************************************************************/
/*				Author:Humanoid Graduation Project              */	
/*				Date: 19-5-2023					                */
/*				Version: 1.0					                */
/*				Module:Kinematics header file  	                */
/****************************************************************/

#ifndef KINEMATICS_FULL_H
#define KINEMATICS_FULL_H
#include <iostream>
#include <stdio.h>
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
#define foot_x_size 0.03
#define foot_x_size2 -0.04
#define foot_y_size 0.018
#define foot_y_size2 -0.018

using namespace KDL;
class point_mass
{
public:
    double x, y, z, mass;
    char state;
};

class stability
{
public:
    bool x,y;
    bool check (){
        return x&&y;
    }
};

class kinematics_full{
    private:
        ros::Publisher pb_Right_Hip_Link;
        ros::Publisher pb_Right_Thigh_Link;
        ros::Publisher pb_Right_Calf_Link;
        ros::Publisher pb_Right_Foot_Link;

        ros::Publisher pb_Left_Hip_Link;
        ros::Publisher pb_Left_Thigh_Link;
        ros::Publisher pb_Left_Calf_Link;
        ros::Publisher pb_Left_Foot_Link;

        ros::Publisher pb_Right_Upper_Shoulder_Link;
        ros::Publisher pb_Right_Mid_Shoulder_Link;
        ros::Publisher pb_Right_Lower_Shoulder_Link;

        ros::Publisher pb_Left_Upper_Shoulder_Link;
        ros::Publisher pb_Left_Mid_Shoulder_Link;
        ros::Publisher pb_Left_Lower_Shoulder_Link;

        ros::Publisher pb_com;
        ros::Publisher pb_centroid;

        ros::Publisher pb_com_x;
        ros::Publisher pb_centroid_x;

        ros::Publisher pb_com_y;
        ros::Publisher pb_centroid_y;

        ros::Publisher pb_traj;

        ros::Subscriber read_Right_Hip_Link;
        ros::Subscriber read_Right_Thigh_Link;
        ros::Subscriber read_Right_Calf_Link;
        ros::Subscriber read_Right_Foot_Link;

        ros::Subscriber read_Left_Hip_Link;
        ros::Subscriber read_Left_Thigh_Link;
        ros::Subscriber read_Left_Calf_Link;
        ros::Subscriber read_Left_Foot_Link;

        ros::Subscriber read_Right_Upper_Shoulder_Link;
        ros::Subscriber read_Right_Mid_Shoulder_Link;
        ros::Subscriber read_Right_Lower_Shoulder_Link;

        ros::Subscriber read_Left_Upper_Shoulder_Link;
        ros::Subscriber read_Left_Mid_Shoulder_Link;
        ros::Subscriber read_Left_Lower_Shoulder_Link;
        ros::Subscriber sub_imu;

        trajectory_msgs::JointTrajectoryPoint traj_point;

        geometry_msgs::PointStamped ps;
        geometry_msgs::PointStamped ps_centroid;
        geometry_msgs::PointStamped ps_Left_Foot;
        geometry_msgs::PointStamped ps_Right_Foot;
        geometry_msgs::PointStamped ps_Right_Lower_Shoulder;
        geometry_msgs::PointStamped ps_Left_Lower_Shoulder;

        KDL::Tree humanoid_tree;
        KDL::JntArray Right_Leg_jntarray;
        KDL::JntArray Left_Leg_jntarray;
        KDL::JntArray Right_Arm_jntarray;
        KDL::JntArray Left_Arm_jntarray;

        point_mass humanoid_CoM;
        point_mass centroid;

        double r_roll;
        double r_pitch;
        double r_yaw;
        double l_roll;
        double l_pitch;
        double l_yaw;
        double imu_roll;
        double imu_pitch;
        double imu_yaw;
        double base_roll;
        double base_pitch;
        double base_yaw;

        void fread_Right_Upper_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Right_Mid_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Right_Lower_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Left_Upper_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Left_Mid_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Left_Lower_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg);

        void fread_Right_Hip_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Right_Thigh_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Right_Calf_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Right_Foot_Link(const control_msgs::JointControllerState::ConstPtr& msg);

        void fread_Left_Hip_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Left_Thigh_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Left_Calf_Link(const control_msgs::JointControllerState::ConstPtr& msg);
        void fread_Left_Foot_Link(const control_msgs::JointControllerState::ConstPtr& msg);

    public:
        bool gazebo;
        bool real;
        KDL::Chain Right_Leg_Chain;
        KDL::Chain Left_Leg_Chain;
        KDL::Chain Right_Arm_Chain;
        KDL::Chain Left_Arm_Chain;

        KDL::Frame Right_Foot;
        KDL::Frame Left_Foot;
        KDL::Frame Right_Lower_Shoulder;
        KDL::Frame Left_Lower_Shoulder;

        double Right_Upper_Shoulder_Link;
        double Right_Mid_Shoulder_Link;
        double Right_Lower_Shoulder_Link;
        double Left_Upper_Shoulder_Link;
        double Left_Mid_Shoulder_Link;
        double Left_Lower_Shoulder_Link;

        double Right_Hip_Link;
        double Right_Thigh_Link;
        double Right_Calf_Link;
        double Right_Foot_Link;

        double Left_Hip_Link;
        double Left_Thigh_Link;
        double Left_Calf_Link;
        double Left_Foot_Link;

        double T_Right_Upper_Shoulder_Link;
        double T_Right_Mid_Shoulder_Link;
        double T_Right_Lower_Shoulder_Link;
        double T_Left_Upper_Shoulder_Link;
        double T_Left_Mid_Shoulder_Link;
        double T_Left_Lower_Shoulder_Link;
        double T_Right_Hip_Link;
        double T_Right_Thigh_Link;
        double T_Right_Calf_Link;
        double T_Right_Foot_Link;
        double T_Left_Hip_Link;
        double T_Left_Thigh_Link;
        double T_Left_Calf_Link;
        double T_Left_Foot_Link;
        double jntstate[15];
        double kp;
        double ki;
        double kd;
        double error_x;
        double error_y;
        double derivative_error_x;
        double derivative_error_y;
        double integral_error_x;
        double integral_error_y;
        double P_limit;
        double I_limit;
        double D_limit;
        double Link_limit;
        bool stable;
        stability humanoid_stability;
        double P_x;
        double P_y;
        double I_x;
        double I_y;
        double D_x;
        double D_y;
        double te_x;
        double te_y;
        double te_z;
        kinematics_full(ros::NodeHandle *n,std::string urdf,std::string base ,std::string Right_Leg_End_Effector, std::string Left_Leg_End_Effector,std::string Right_Arm_End_Effector, std::string Left_Arm_End_Effector);
        void set_T_jointpose(double *jnt);
        void set_T_equal_jointpose();
        void set_jointpose(double *jnt);
        void add_jointpose(double *jnt);
        void set_kdjointpose();
        void joint_publish(ros::Rate *r);
        void com_publish(char state,ros::Rate *r);
        point_mass compute_com(char state,KDL::ChainFkSolverPos_recursive *Right_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Left_Leg_fk_solver,KDL::ChainFkSolverPos_recursive *Right_Arm_fk_solver,KDL::ChainFkSolverPos_recursive *Left_Arm_fk_solver, bool verbose );
        point_mass compute_centroid(char state,bool verbose );
        stability quick_is_stable(bool verbose );
        bool humanoid_will_go_on(char state, KDL::ChainFkSolverPos_recursive *Right_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Left_Leg_fk_solver,KDL::ChainFkSolverPos_recursive *Right_Arm_fk_solver,KDL::ChainFkSolverPos_recursive *Left_Arm_fk_solver,ros::Rate *rate , int freq);
        bool move_leg(double *target, char state, KDL::ChainFkSolverPos_recursive *Right_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Left_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Right_Arm_fk_solver,KDL::ChainFkSolverPos_recursive *Left_Arm_fk_solver,ros::Rate *rate, int freq );
};
void get_path(const nav_msgs::Path::ConstPtr &msg, int sampling, ros::Rate *rate, kinematics_full *humanoid, KDL::ChainFkSolverPos_recursive *Right_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Left_Leg_fk_solver,KDL::ChainFkSolverPos_recursive *Right_Arm_fk_solver,KDL::ChainFkSolverPos_recursive *Left_Arm_fk_solver);

#endif 
