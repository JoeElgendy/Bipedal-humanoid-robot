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

// unit test for the function
int main(int argc, char **argv){
    KDL::ChainFkSolverPos_recursive *right_leg_fksolver = new KDL::ChainFkSolverPos_recursive(right_leg);
    KDL::ChainFkSolverPos_recursive *left_leg_fksolver = new KDL::ChainFkSolverPos_recursive(left_leg);
    point_mass CoM = compute_com('r', right_leg_fksolver, left_leg_fksolver, true);
    std::cout << "Humanoid CoM : " << CoM.x << "  " << CoM.y << "  " << CoM.z << "\n";
    return 0;
}
