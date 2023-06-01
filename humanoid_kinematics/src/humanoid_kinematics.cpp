#include "kinematics_full.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "humanoid_kinematics");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    std::string urdf_file;
    nh.getParam("urdf_file", urdf_file);
    kinematics_full humanoid(&nh, urdf_file, "dummy", "Right_Foot_Link", "Left_foot_Link","Right_Lower_Shoulder_link","Left_Lower_Shoulder_Link");

    KDL::ChainFkSolverPos_recursive Right_Leg_fk_solver(humanoid.Right_Leg_Chain);
    KDL::ChainFkSolverPos_recursive Left_Leg_fk_solver(humanoid.Left_Leg_Chain);
    KDL::ChainFkSolverPos_recursive Right_Arm_fk_solver(humanoid.Right_Arm_Chain);
    KDL::ChainFkSolverPos_recursive Left_Arm_fk_solver(humanoid.Left_Arm_Chain);

    nav_msgs::Path ptest;
    int sampling = 200;
        // boost::shared_ptr<nav_msgs::Path const> sharedPtr;
        // nav_msgs::Path sub_path2;
        // sharedPtr  = ros::topic::waitForMessage<nav_msgs::Path>("humanoid/footstep_path", ros::Duration(10));
        // if (sharedPtr == NULL) ROS_INFO("NO path received");
        // else sub_path2 = *sharedPtr;

    ros::Subscriber sub_path = nh.subscribe<nav_msgs::Path>("humanoid/footstep_path", 100, boost::bind(get_path, _1, sampling, &rate, &humanoid, &Right_Leg_fk_solver, &Left_Leg_fk_solver,&Right_Arm_fk_solver,&Left_Arm_fk_solver));
    double home[15] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0};
    humanoid.set_jointpose(home);
    humanoid.set_T_jointpose(home);
    humanoid.humanoid_will_go_on('d', &Right_Leg_fk_solver, &Left_Leg_fk_solver,&Right_Arm_fk_solver,&Left_Arm_fk_solver,&rate, 70);
    //humanoid.humanoid_will_go_on('r', &Right_Leg_fk_solver, &Left_Leg_fk_solver,&Right_Arm_fk_solver,&Left_Arm_fk_solver,&rate, 70);
    // humanoid.humanoid_will_go_on('l', &Right_Leg_fk_solver, &Left_Leg_fk_solver,&Right_Arm_fk_solver,&Left_Arm_fk_solver,&rate, 70);
    humanoid.T_Right_Thigh_Link = 0.25;
    humanoid.T_Right_Calf_Link  = 0.25;
    humanoid.T_Left_Thigh_Link  = 0.25;
    humanoid.T_Left_Calf_Link   = -0.25;
    humanoid.humanoid_will_go_on('d', &Right_Leg_fk_solver, &Left_Leg_fk_solver,&Right_Arm_fk_solver,&Left_Arm_fk_solver,&rate, 70);

    std::cout << "finished 2\n joe \n ";

    std::clock_t begin = clock();

    ros::spin();
    humanoid.humanoid_will_go_on('d', &Right_Leg_fk_solver, &Left_Leg_fk_solver,&Right_Arm_fk_solver,&Left_Arm_fk_solver,&rate, 70);
    humanoid.humanoid_will_go_on('l', &Right_Leg_fk_solver, &Left_Leg_fk_solver,&Right_Arm_fk_solver,&Left_Arm_fk_solver,&rate, 70);

    std::clock_t end = clock();
    std::cout << "elapsed time is \n " << double(end - begin) / CLOCKS_PER_SEC << "\n";

    return 1;

}