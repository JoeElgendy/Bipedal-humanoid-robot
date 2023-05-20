/****************************************************************/
/*				Author:Humanoid Graduation Project              */	
/*				Date: 19-5-2023					                */
/*				Version: 1.0					                */
/*				Module:Kinematics cpp file  	                */
/****************************************************************/
#include "kinematics_full.h"

//upper 0 , mid 1 , lower 2 , hip 3 , thigh 4 , calf 5 , foot 6 , upper 7 , mid 8 , lower 9 , hip 10 , thigh 11 , calf 12 , foot 13
void kinematics_full::fread_Right_Upper_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[0] = msg->process_value;
    Left_Arm_jntarray.data[7] = msg->process_value;
}
void kinematics_full::fread_Right_Mid_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[1] = msg->process_value;
    Left_Arm_jntarray.data[8] = msg->process_value;
}
void kinematics_full::fread_Right_Lower_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[2] = msg->process_value;
    Left_Arm_jntarray.data[9] = msg->process_value;
}
void kinematics_full::fread_Right_Hip_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[3] = msg->process_value;
    Left_Arm_jntarray.data[10] = msg->process_value;
}
void kinematics_full::fread_Right_Thigh_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[4] = msg->process_value;
    Left_Arm_jntarray.data[11] = msg->process_value;
}
void kinematics_full::fread_Right_Calf_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[5] = msg->process_value;
    Left_Arm_jntarray.data[12] = msg->process_value;
}
void kinematics_full::fread_Right_Foot_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[6] = msg->process_value;
    Left_Arm_jntarray.data[13] = msg->process_value;
}

void kinematics_full::fread_Left_Upper_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[7] = msg->process_value;
    Left_Arm_jntarray.data[0] = msg->process_value;
}
void kinematics_full::fread_Left_Mid_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[8] = msg->process_value;
    Left_Arm_jntarray.data[1] = msg->process_value;
}
void kinematics_full::fread_Left_Lower_Shoulder_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[9] = msg->process_value;
    Left_Arm_jntarray.data[2] = msg->process_value;
}
void kinematics_full::fread_Left_Hip_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[10] = msg->process_value;
    Left_Arm_jntarray.data[3] = msg->process_value;
}
void kinematics_full::fread_Left_Thigh_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[11] = msg->process_value;
    Left_Arm_jntarray.data[4] = msg->process_value;
}
void kinematics_full::fread_Left_Calf_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[12] = msg->process_value;
    Left_Arm_jntarray.data[5] = msg->process_value;
}
void kinematics_full::fread_Left_Foot_Link(const control_msgs::JointControllerState::ConstPtr& msg){
    Right_Arm_jntarray.data[13] = msg->process_value;
    Left_Arm_jntarray.data[6] = msg->process_value;
}

kinematics_full::kinematics_full(ros::NodeHandle *n,std::string urdf,std::string base ,std::string Right_Leg_End_Effector, std::string Left_Leg_End_Effector,std::string Right_Arm_End_Effector, std::string Left_Arm_End_Effector){

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
    pb_com_x = n->advertise<std_msgs::Float64>("humanoid/com/x", 10);
    pb_centroid_x = n->advertise<std_msgs::Float64>("humanoid/centroid/x", 10);
    pb_com_y = n->advertise<std_msgs::Float64>("humanoid/com/y", 10);
    pb_centroid_y = n->advertise<std_msgs::Float64>("humanoid/centroid/y", 10);
    pb_traj = n->advertise<trajectory_msgs::JointTrajectory>("humanoid/joint_command", 10);
    
    read_Right_Upper_Shoulder_Link=n->subscribe("/humanoid/Right_Upper_Shoulder_Joint_position/state",1000, &kinematics_full::fread_Right_Upper_Shoulder_Link,this);
    read_Right_Mid_Shoulder_Link=n->subscribe("/humanoid/Right_Mid_Shoulder_Joint_position/state",1000,& kinematics_full::fread_Right_Mid_Shoulder_Link,this);
    read_Right_Lower_Shoulder_Link=n->subscribe("/humanoid/Right_Lower_Shoulder_Joint_position/state",1000,&kinematics_full::fread_Right_Lower_Shoulder_Link,this);
    read_Right_Hip_Link = n->subscribe("/humanoid/Right_Hip_Joint_position/state", 1000, &kinematics_full::fread_Right_Hip_Link, this);
    read_Right_Thigh_Link = n->subscribe("/humanoid/Right_Thigh_Joint_position/state", 1000, &kinematics_full::fread_Right_Thigh_Link, this);
    read_Right_Calf_Link = n->subscribe("/humanoid/Right_Calf_Joint_position/state", 1000, &kinematics_full::fread_Right_Calf_Link, this);
    read_Right_Foot_Link = n->subscribe("/humanoid/Right_Foot_Joint_position/state", 1000, &kinematics_full::fread_Right_Foot_Link, this);
    read_Left_Upper_Shoulder_Link=n->subscribe("/humanoid/Left_Upper_Shoulder_Joint_position/state",1000,&kinematics_full:: fread_Left_Upper_Shoulder_Link,this);
    read_Left_Mid_Shoulder_Link=n->subscribe("/humanoid/Left_Mid_Shoulder_Joint_position/state",1000,&kinematics_full::fread_Left_Mid_Shoulder_Link,this);
    read_Left_Lower_Shoulder_Link=n->subscribe("/humanoid/Left_Lower_Shoulder_Joint_postion/state",1000,&kinematics_full::fread_Left_Lower_Shoulder_Link,this);
    read_Left_Hip_Link = n->subscribe("/humanoid/Left_Hip_Joint_position/state", 1000, &kinematics_full::fread_Left_Hip_Link, this);
    read_Left_Thigh_Link = n->subscribe("/humanoid/Left_Thigh_joint_position/state", 1000, &kinematics_full::fread_Left_Thigh_Link, this);
    read_Left_Calf_Link = n->subscribe("/humanoid/Left_Calf_Joint_position/state", 1000, &kinematics_full::fread_Left_Calf_Link, this);
    read_Left_Foot_Link = n->subscribe("/humanoid/Left_foot_Joint_position/state", 1000, &kinematics_full::fread_Left_Foot_Link, this);

    if(!kdl_parser::treeFromFile(urdf,humanoid_tree)){
        std::cout << "Failed to construct kdl tree \n" ;
    }
    else{
        std::cout << "Succeeded to construct kdl tree \n" ;
    }
    std::string root_link=humanoid_tree.getRootSegment()->first;
    std::vector<Chain> humanoid_chains;
    
    KDL::Chain Right_Arm_Chain,Left_Arm_Chain,Right_Leg_Chain,Left_Leg_Chain;

    if(!humanoid_tree.getChain(root_link,Right_Leg_End_Effector,Right_Leg_Chain)){
        std::cout << "Failed to construct Right Leg Chain \n" ;
    }
    else{
        std::cout << "Succeeded to construct Right Leg Chain \n" ;
        humanoid_chains.push_back(Right_Leg_Chain);
    }
    if(!humanoid_tree.getChain(root_link,Left_Leg_End_Effector,Left_Leg_Chain)){
        std::cout << "Failed to construct Left Leg Chain \n" ;
        
    }
    else{
        std::cout << "Succeeded to construct Left Leg Chain \n" ;
        humanoid_chains.push_back(Left_Leg_Chain);
        Right_Leg_jntarray=KDL::JntArray(Right_Leg_Chain.getNrOfJoints());
        Left_Leg_jntarray=KDL::JntArray(Left_Leg_Chain.getNrOfJoints());
    }
    if(!humanoid_tree.getChain(root_link,Right_Arm_End_Effector,Right_Arm_Chain)){
        std::cout << "Failed to construct Right Arm Chain \n" ;
    }
    else{
        std::cout << "Succeeded to construct Right Arm Chain \n" ;
        humanoid_chains.push_back(Right_Arm_Chain);
    }
    if(!humanoid_tree.getChain(root_link,Left_Arm_End_Effector,Left_Arm_Chain)){
        std::cout << "Failed to construct Left Arm Chain \n" ;
    }
    else{
        std::cout << "Succeeded to construct Left Arm Chain \n" ;
        humanoid_chains.push_back(Left_Arm_Chain);
        Right_Arm_jntarray=KDL::JntArray(Right_Arm_Chain.getNrOfJoints());
        Left_Arm_jntarray=KDL::JntArray(Left_Arm_Chain.getNrOfJoints());
    }

}

void kinematics_full::set_jointpose(double *jnt){
    Right_Upper_Shoulder_Link=jnt[0];
    Right_Mid_Shoulder_Link=jnt[1];
    Right_Lower_Shoulder_Link=jnt[2];

    Right_Hip_Link=jnt[3];
    Right_Thigh_Link=jnt[4];
    Right_Calf_Link=jnt[5];
    Right_Foot_Link=jnt[6];

    Left_Upper_Shoulder_Link=jnt[7];
    Left_Mid_Shoulder_Link=jnt[8];
    Left_Lower_Shoulder_Link=jnt[9];

    Left_Hip_Link=jnt[10];
    Left_Thigh_Link=jnt[11];
    Left_Calf_Link=jnt[12];
    Left_Foot_Link=jnt[13];
}

void kinematics_full::set_T_jointpose(double *jnt){
    T_Right_Upper_Shoulder_Link=jnt[0];
    T_Right_Mid_Shoulder_Link=jnt[1];
    T_Right_Lower_Shoulder_Link=jnt[2];

    T_Right_Hip_Link=jnt[3];
    T_Right_Thigh_Link=jnt[4];
    T_Right_Calf_Link=jnt[5];
    T_Right_Foot_Link=jnt[6];

    T_Left_Upper_Shoulder_Link=jnt[7];
    T_Left_Mid_Shoulder_Link=jnt[8];
    T_Left_Lower_Shoulder_Link=jnt[9];

    T_Left_Hip_Link=jnt[10];
    T_Left_Thigh_Link=jnt[11];
    T_Left_Calf_Link=jnt[12];
    T_Left_Foot_Link=jnt[13];
}

void kinematics_full::set_T_equal_jointpose(){
    T_Right_Upper_Shoulder_Link=Right_Upper_Shoulder_Link;
    T_Right_Mid_Shoulder_Link=Right_Mid_Shoulder_Link;
    T_Right_Lower_Shoulder_Link=Right_Lower_Shoulder_Link;

    T_Right_Hip_Link=Right_Hip_Link;
    T_Right_Thigh_Link=Right_Thigh_Link;
    T_Right_Calf_Link=Right_Calf_Link;
    T_Right_Foot_Link=Right_Foot_Link;

    T_Left_Upper_Shoulder_Link=Left_Upper_Shoulder_Link;
    T_Left_Mid_Shoulder_Link=Left_Mid_Shoulder_Link;
    T_Left_Lower_Shoulder_Link=Left_Lower_Shoulder_Link;

    T_Left_Hip_Link=Left_Hip_Link;
    T_Left_Thigh_Link=Left_Thigh_Link;
    T_Left_Calf_Link=Left_Calf_Link;
    T_Left_Foot_Link=Left_Foot_Link;
}

void kinematics_full::set_kdjointpose(){
    Right_Arm_jntarray.data[7]  = Left_Upper_Shoulder_Link;
    Right_Arm_jntarray.data[0]  = Right_Upper_Shoulder_Link;

    Right_Arm_jntarray.data[8]  = Left_Mid_Shoulder_Link;
    Right_Arm_jntarray.data[1]  = Right_Mid_Shoulder_Link;

    Right_Arm_jntarray.data[9]  = Left_Lower_Shoulder_Link;
    Right_Arm_jntarray.data[2]  = Right_Lower_Shoulder_Link;

    Right_Leg_jntarray.data[13] = Left_Foot_Link;   
    Right_Leg_jntarray.data[6] = Right_Foot_Link;  

    Right_Leg_jntarray.data[12] = Left_Calf_Link;   
    Right_Leg_jntarray.data[5] = Right_Calf_Link;  

    Right_Leg_jntarray.data[4] = Right_Thigh_Link; 
    Right_Leg_jntarray.data[3] = Right_Hip_Link;   

    Right_Leg_jntarray.data[10] = Left_Hip_Link;    
    Right_Leg_jntarray.data[11] = Left_Thigh_Link;  

    Left_Arm_jntarray.data[7]  = Right_Upper_Shoulder_Link; 
    Left_Arm_jntarray.data[0]  = Left_Upper_Shoulder_Link;

    Left_Arm_jntarray.data[8]  = Right_Mid_Shoulder_Link;
    Left_Arm_jntarray.data[1]  = Left_Mid_Shoulder_Link;

    Left_Arm_jntarray.data[9]  = Right_Lower_Shoulder_Link;
    Left_Arm_jntarray.data[2]  = Left_Lower_Shoulder_Link;

    Left_Leg_jntarray.data[13] = Right_Foot_Link;  
    Left_Leg_jntarray.data[6] = Left_Foot_Link;   

    Left_Leg_jntarray.data[12] = Right_Calf_Link;  
    Left_Leg_jntarray.data[5] = Left_Calf_Link;   

    Left_Leg_jntarray.data[4] = Left_Thigh_Link;  
    Left_Leg_jntarray.data[3] = Left_Hip_Link;    

    Left_Leg_jntarray.data[10] = Right_Hip_Link;   
    Left_Leg_jntarray.data[11] = Right_Thigh_Link; 
}

void kinematics_full::add_jointpose(double *jnt){
    Right_Upper_Shoulder_Link +=*(jnt+0);
    Right_Mid_Shoulder_Link +=*(jnt+1);
    Right_Lower_Shoulder_Link +=*(jnt+2);

    Right_Hip_Link += *(jnt+3);
    Right_Thigh_Link += *(jnt+4);
    Right_Calf_Link += *(jnt+5);
    Right_Foot_Link += *(jnt+6);

    Left_Upper_Shoulder_Link +=*(jnt+7);
    Left_Mid_Shoulder_Link +=*(jnt+8);
    Left_Lower_Shoulder_Link +=*(jnt+9);

    Left_Hip_Link += *(jnt+10);
    Left_Thigh_Link += *(jnt+11);
    Left_Calf_Link += *(jnt+12);
    Left_Foot_Link += *(jnt+13);
}

void kinematics_full::joint_publish(ros::Rate *r){
    std_msgs::Float64 radi;

    ros::Duration duration(1/20);
    radi.data= Right_Upper_Shoulder_Link; if(radi.data) pb_Right_Upper_Shoulder_Link.publish(radi);
    radi.data = Right_Mid_Shoulder_Link;  if(radi.data) pb_Right_Mid_Shoulder_Link.publish(radi);
    radi.data = Right_Lower_Shoulder_Link;if(radi.data)pb_Right_Lower_Shoulder_Link.publish(radi);

    radi.data = Right_Hip_Link;   if(radi.data) pb_Right_Hip_Link.publish(radi);
    radi.data = Right_Thigh_Link; if(radi.data) pb_Right_Thigh_Link.publish(radi);
    radi.data = Right_Calf_Link;  if(radi.data) pb_Right_Calf_Link.publish(radi);
    radi.data = Right_Foot_Link;  if(radi.data) pb_Right_Foot_Link.publish(radi);

    radi.data = Left_Upper_Shoulder_Link; if(radi.data) pb_Left_Upper_Shoulder_Link.publish(radi);
    radi.data = Left_Mid_Shoulder_Link;   if(radi.data) pb_Left_Mid_Shoulder_Link.publish(radi);
    radi.data = Left_Lower_Shoulder_Link; if(radi.data) pb_Left_Lower_Shoulder_Link.publish(radi);
    
    radi.data = Left_Hip_Link;  if(radi.data) pb_Left_Hip_Link.publish(radi);
    radi.data = Left_Thigh_Link;if(radi.data) pb_Left_Thigh_Link.publish(radi);
    radi.data = Left_Calf_Link; if(radi.data) pb_Left_Calf_Link.publish(radi);
    radi.data = Left_Foot_Link; if(radi.data) pb_Left_Foot_Link.publish(radi);

    duration.sleep();
}
void kinematics_full::com_publish(char state,ros::Rate *r){
    ps.point.x = humanoid_CoM.x;
    ps.point.y = humanoid_CoM.y;
    ps.point.z = 0;
    ros::Duration duration(1/20);
        if (state == 'r' || state == 'd')
        {
            ps.header.frame_id = "Right_Foot_Link";
            ps_centroid.header.frame_id = "Right_Foot_Link";
        }
        else if (state == 'l')
        {
            ps.header.frame_id = "Left_Foot_link";
            ps_centroid.header.frame_id = "Left_Foot_link";
        }
        ps_centroid.point.x = centroid.x;
        ps_centroid.point.y = centroid.y;
        ps_centroid.point.z = ps.point.z;
        ps_Left_Foot.header.frame_id = "Right_Foot_link";
        ps_Left_Foot.point.x = Left_Foot.p.data[0];
        ps_Left_Foot.point.y = Left_Foot.p.data[1];
        ps_Left_Foot.point.z = Left_Foot.p.data[2];
        ps_Right_Foot.header.frame_id = "Left_Foot_link";
        ps_Right_Foot.point.x = Right_Foot.p.data[0];
        ps_Right_Foot.point.y = Right_Foot.p.data[1];
        ps_Right_Foot.point.z = Right_Foot.p.data[2];
        if(pb_com)             pb_com.publish(ps);
        if(pb_centroid)        pb_centroid.publish(ps_centroid);
        if(pb_Left_Foot_Link)  pb_Left_Foot_Link.publish(ps_Left_Foot);
        if(pb_Right_Foot_Link) pb_Right_Foot_Link.publish(ps_Right_Foot);

        std_msgs::Float64 centroid_x;
        std_msgs::Float64 centroid_y;
        std_msgs::Float64 com_x;
        std_msgs::Float64  com_y;
        centroid_x.data = centroid.x;
        com_x.data = humanoid_CoM.x;
        centroid_y.data = centroid.y;
        com_y.data = humanoid_CoM.y;
        if(pb_com_x)      pb_com_x.publish(com_x);
        if(pb_centroid_x) pb_centroid_x.publish(centroid_x);
        if(pb_com_y)      pb_com_y.publish(com_y);
        if(pb_centroid_y) pb_centroid_y.publish(centroid_y);
        duration.sleep();
}
point_mass kinematics_full::compute_com(char state,KDL::ChainFkSolverPos_recursive *Right_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Left_Leg_fk_solver,KDL::ChainFkSolverPos_recursive *Right_Arm_fk_solver,KDL::ChainFkSolverPos_recursive *Left_Arm_fk_solver, bool verbose = 0)
{
    point_mass r_CoM;
    point_mass l_CoM;
    double r_roll,r_pitch,r_yaw;
    KDL::Rotation rot_inv;
    KDL::Vector link_cog;
    KDL::Vector link_cog_refbase;
    KDL::Vector link_temp;
    for(int i=0;i<Right_Arm.getNrOfSegments();i++){
        Right_Arm_fk_solver->JntToCart(Right_Arm_jntarray,Right_Lower_Shoulder,i);
        Right_Lower_Shoulder.M.GetRPY(r_roll,r_pitch,r_yaw);
        rot_inv=Right_Lower_Shoulder.M.Inverse();
        link_cog=Right_Arm.getSegment(i).getInertia().getCOG();
        link_temp=rot_inv.operator*(Right_Lower_Shoulder.p);
        link_cog_refbase.data[0]=link_temp.data[0] + link_cog.data[0];
        link_cog_refbase.data[1]=link_temp.data[1] + link_cog.data[1];
        link_cog_refbase.data[2]=link_temp.data[2] + link_cog.data[2];
        link_cog_refbase.operator=(Right_Lower_Shoulder.M.operator*(link_cog_refbase));
        r_CoM.x+=link_cog_refbase.data[0]*Right_Arm.getSegment(i).getInertia().getMass();
        r_CoM.y+=link_cog_refbase.data[1]*Right_Arm.getSegment(i).getInertia().getMass();
        r_CoM.z+=link_cog_refbase.data[2]*Right_Arm.getSegment(i).getInertia().getMass();
    
    if(verbose){
                std::cout << std::setprecision(5)<< i<<".) state ="
                          <<" " <<Right_Arm.getSegment(i).getName()<<" mass =" << Right_Arm.getSegment(i).getInertia().getMass()
                          <<"     Rotation :" <<r_roll<< " " <<r_pitch<<" "<<r_yaw
                          << "    Mass Trans : " << r_CoM.x << "  " << r_CoM.y << "  " << r_CoM.z << ""
                          <<"     Translation main : "<<Right_Lower_Shoulder.p[0]<<" "<<Right_Lower_Shoulder.p[1]<<" "<<Right_Lower_Shoulder.p[2]<<"\n";

            }
    humanoid_CoM.x=r_CoM.x/r_CoM.mass;
    humanoid_CoM.y=r_CoM.y/r_CoM.mass;
    }
    for(int i=0;i<Left_Arm.getNrOfSegments();i++){
        Left_Arm_fk_solver->JntToCart(Left_Arm_jntarray,Left_Lower_Shoulder,i);
        Left_Lower_Shoulder.M.GetRPY(r_roll,r_pitch,r_yaw);
        rot_inv=Left_Lower_Shoulder.M.Inverse();
        link_cog=Left_Arm.getSegment(i).getInertia().getCOG();
        link_temp=rot_inv.operator*(Left_Lower_Shoulder.p);
        link_cog_refbase.data[0]=link_temp.data[0] + link_cog.data[0];
        link_cog_refbase.data[1]=link_temp.data[1] + link_cog.data[1];
        link_cog_refbase.data[2]=link_temp.data[2] + link_cog.data[2];
        link_cog_refbase.operator=(Left_Lower_Shoulder.M.operator*(link_cog_refbase));
        l_CoM.x+=link_cog_refbase.data[0]*Left_Arm.getSegment(i).getInertia().getMass();
        l_CoM.y+=link_cog_refbase.data[1]*Left_Arm.getSegment(i).getInertia().getMass();
        l_CoM.z+=link_cog_refbase.data[2]*Left_Arm.getSegment(i).getInertia().getMass();
    
    if(verbose){
                        std::cout << std::setprecision(5)<< i<<".) state ="
                          <<" " <<Left_Arm.getSegment(i).getName()<<" mass =" << Left_Arm.getSegment(i).getInertia().getMass()
                          <<"     Rotation :" <<l_roll<< " " <<l_pitch<<" "<<l_yaw
                          << "    Mass Trans : " << l_CoM.x << "  " << l_CoM.y << "  " << l_CoM.z << ""
                          <<"     Translation main : "<<Left_Lower_Shoulder.p[0]<<" "<<Left_Lower_Shoulder.p[1]<<" "<<Left_Lower_Shoulder.p[2]<<"\n";
    }
    humanoid_CoM.x+=l_CoM.x/l_CoM.mass;
    humanoid_CoM.y+=l_CoM.y/l_CoM.mass;
    }
            if (state == 'r' || state == 'd')
        {
            for (int i = 0; i < Right_Leg.getNrOfSegments(); i++)
            {
                Right_Leg_fk_solver->JntToCart(Right_Leg_jntarray, Left_Foot, i + 1); // calculate the forward kinematics and stores it in Left_Foot
                double r_roll, r_pitch, r_yaw;
                Left_Foot.M.GetRPY(r_roll, r_pitch, r_yaw);
                KDL::Rotation rot_inv = Left_Foot.M.Inverse();                       // calculate the orientation with respect frame
                KDL::Vector link_cog = Right_Leg.getSegment(i).getInertia().getCOG(); // the center of mass of the link with respect to the link frame
                KDL::Vector link_cog_refbase;                                     // the center of mass of the link with respect to the base frame
                KDL::Vector link_temp = rot_inv.operator*(Left_Foot.p);
                // the COG of the link is computed with respect to the base frame and added to the total COG of the robot
                link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
                link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
                link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
                link_cog_refbase.operator=(Left_Foot.M.operator*(link_cog_refbase));
                r_CoM.x += link_cog_refbase.data[0] * Right_Leg.getSegment(i).getInertia().getMass();
                r_CoM.y += link_cog_refbase.data[1] * Right_Leg.getSegment(i).getInertia().getMass();
                r_CoM.z += link_cog_refbase.data[2] * Right_Leg.getSegment(i).getInertia().getMass();

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
                              << "  " << Right_Leg.getSegment(i).getName() << " mass = " << Right_Leg.getSegment(i).getInertia().getMass()
                              << "        Rotation : " << r_roll << "  " << r_pitch << "  " << r_yaw
                              << "           Mass Trans : " << r_CoM.x << "  " << r_CoM.y << "  " << r_CoM.z << ""
                              << "       Translation main : " << Left_Foot.p[0] << "  " << Left_Foot.p[1] << "  " << Left_Foot.p[2] << "\n";
                }
                humanoid_CoM.x += r_CoM.x / r_CoM.mass;
                humanoid_CoM.y += r_CoM.y / r_CoM.mass;
            }
        }
        if (state == 'l')
        {
            for (int i = 0; i < Left_Leg.getNrOfSegments(); i++)
            {
                Left_Leg_fk_solver->JntToCart(Left_Leg_jntarray, Right_Foot, i + 1);
                double l_roll, l_pitch, l_yaw;
                Right_Foot.M.GetRPY(l_roll, l_pitch, l_yaw);

                KDL::Rotation rot_inv = Right_Foot.M.Inverse();
                KDL::Vector link_cog = Left_Leg.getSegment(i).getInertia().getCOG();
                KDL::Vector link_cog_refbase;
                KDL::Vector link_temp = rot_inv.operator*(Right_Foot.p);
                link_cog_refbase.data[0] = link_temp.data[0] + link_cog.data[0];
                link_cog_refbase.data[1] = link_temp.data[1] + link_cog.data[1];
                link_cog_refbase.data[2] = link_temp.data[2] + link_cog.data[2];
                link_cog_refbase.operator=(Right_Foot.M.operator*(link_cog_refbase));
                l_CoM.x += link_cog_refbase.data[0] * Left_Leg.getSegment(i).getInertia().getMass();
                l_CoM.y += link_cog_refbase.data[1] * Left_Leg.getSegment(i).getInertia().getMass();
                l_CoM.z += link_cog_refbase.data[2] * Left_Leg.getSegment(i).getInertia().getMass();
                l_CoM.mass += Left_Leg.getSegment(i).getInertia().getMass();
                if (verbose)
                {
                    std::cout
                        << std::setprecision(5) << i << ".) state = "
                        << "  " << Left_Leg.getSegment(i).getName() << " mass = " << Left_Leg.getSegment(i).getInertia().getMass()
                        << "        Rotation : " << l_roll << "  " << l_pitch << "  " << l_yaw
                        << "           Mass Trans : " << l_CoM.x << "  " << l_CoM.y << "  " << l_CoM.z << ""
                        << "       Translation main : " << Right_Foot.p[0] << "  " << Right_Foot.p[1] << "  " << Right_Foot.p[2] << "\n";
                }
                humanoid_CoM.x += l_CoM.x / l_CoM.mass;
                humanoid_CoM.y += l_CoM.y / l_CoM.mass;
            }
        }
        return humanoid_CoM;
}

point_mass kinematics_full::compute_centroid(char state,bool verbose){
        if (state == 'r' || state == 'l') // R leg is the stance
        {
            centroid.x = 0;
            centroid.y = 0;
            centroid.z = 0;
        } 
        else if (state == 'd') // R is main leg for double support
        {
            centroid.x = Left_Foot.p.data[0] / 2.0;
            centroid.y = Left_Foot.p.data[1] / 2.0;
            centroid.z = Left_Foot.p.data[2] / 2.0;
        }
        centroid.state = state;
        if (verbose)
        {
            std::cout << centroid.state << "centroid :(" << centroid.x << "," << centroid.y << "," << centroid.z << ")\n"
                      << std::endl;
        }
        return centroid;
}

stability kinematics_full::quick_is_stable(bool verbose){
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

bool kinematics_full::humanoid_will_go_on(char state, KDL::ChainFkSolverPos_recursive *Right_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Left_Leg_fk_solver,KDL::ChainFkSolverPos_recursive *Right_Arm_fk_solver,KDL::ChainFkSolverPos_recursive *Left_Arm_fk_solver,ros::Rate *rate , int freq=200){
            jntstate[0]= ((T_Right_Upper_Shoulder_Link - Right_Upper_Shoulder_Link)/freq);
            jntstate[1]=((T_Right_Mid_Shoulder_Link-Right_Mid_Shoulder_Link)/freq);
            jntstate[2]=((T_Right_Lower_Shoulder_Link-Right_Lower_Shoulder_Link)/freq);

            jntstate[3]=((T_Right_Hip_Link - Right_Hip_Link)/freq);
            jntstate[4]=((T_Right_Thigh_Link - Right_Thigh_Link) / freq);
            jntstate[5]=((T_Right_Calf_Link - Right_Calf_Link) / freq);
            jntstate[6]=((T_Right_Foot_Link - Right_Foot_Link) / freq);

            jntstate[7]=((T_Left_Upper_Shoulder_Link - Left_Upper_Shoulder_Link)/freq);
            jntstate[8]=((T_Left_Mid_Shoulder_Link-Left_Mid_Shoulder_Link)/freq);
            jntstate[9]=((T_Left_Lower_Shoulder_Link-Left_Lower_Shoulder_Link)/freq);

            jntstate[10]=((T_Left_Hip_Link - Left_Hip_Link)/freq);
            jntstate[11]=((T_Left_Thigh_Link - Left_Thigh_Link) / freq);
            jntstate[12]=((T_Left_Calf_Link - Left_Calf_Link) / freq);
            jntstate[13]=((T_Left_Foot_Link - Left_Foot_Link) / freq);

            stable=false;
            kp=20.0 / freq;
            ki=1.5 / freq;
            kd=0.0 / freq;

            error_x=0;
            error_y=0;
            derivative_error_x=0;
            derivative_error_y=0;
            integral_error_x=0;
            integral_error_y=0;

            P_limit=0.2/freq;
            I_limit=4.0/freq;
            D_limit=20000.0/freq;

            for(int k=0 ; k<freq ; k++){
                kinematics_full::add_jointpose(jntstate);
                kinematics_full::set_kdjointpose();
                humanoid_CoM=kinematics_full::compute_com(state,Right_Leg_fk_solver,Left_Leg_fk_solver,Right_Arm_fk_solver,Left_Arm_fk_solver);
                centroid=kinematics_full::compute_centroid(state,0);
                humanoid_stability=kinematics_full::quick_is_stable(0);
                stable=humanoid_stability.check();
                error_x = humanoid_CoM.x - centroid.x;
                error_y = humanoid_CoM.y - centroid.y;
                derivative_error_x = (humanoid_CoM.x - centroid.x) - error_x;
                derivative_error_y = (humanoid_CoM.y - centroid.y) - error_y;
                integral_error_x += error_x;
                integral_error_y += error_y;
                P_x=(kp* error_x);
                P_y=(kp* error_y);
                I_x=(ki* integral_error_x);
                I_y=(ki* integral_error_y);
                D_x=(kd* derivative_error_x);
                D_y=(kd* derivative_error_y);
                if (P_x > P_limit) P_x = P_limit; else if (P_x < -P_limit) P_x = -P_limit;
                if (P_y > P_limit) P_y = P_limit; else if (P_y < -P_limit) P_y = -P_limit;
                if (I_x > I_limit) I_x = I_limit; else if (I_x < -I_limit) I_x = -I_limit;
                if (I_y > I_limit) I_y = I_limit; else if (I_y < -I_limit) I_y = -I_limit;
                if (D_x > D_limit) D_x = D_limit; else if (D_x < -D_limit) D_x = -D_limit;
                if (D_y > D_limit) D_y = D_limit; else if (D_y < -D_limit) D_y = -D_limit;

                if(state == 'r'){
                    Right_Hip_Link += (P_y + I_y + D_y);
                    Right_Thigh_Link +=(P_x + I_x + D_x);
                }
                else if(state == 'l'){
                    Left_Hip_Link += (P_y + I_y + D_y);
                    Left_Thigh_Link +=(P_x + I_x + D_x);
                }
                else if(state == 'b'){
                    Right_Hip_Link += (P_y + I_y + D_y);
                    Right_Thigh_Link +=(P_x + I_x + D_x);
                    Left_Hip_Link += (P_y + I_y + D_y);
                    Left_Thigh_Link +=(P_x + I_x + D_x);
                }
                kinematics_full::joint_publish(rate);
                kinematics_full::com_publish(state,rate);          
        }
        kinematics_full::set_T_equal_jointpose();
        }
        
bool kinematics_full::move_leg(double *target, char state, KDL::ChainFkSolverPos_recursive *Right_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Left_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Right_Arm_fk_solver,KDL::ChainFkSolverPos_recursive *Left_Arm_fk_solver,ros::Rate *rate, int freq = 200){
            jntstate[0]= ((T_Right_Upper_Shoulder_Link - Right_Upper_Shoulder_Link)/freq);
            jntstate[1]=((T_Right_Mid_Shoulder_Link-Right_Mid_Shoulder_Link)/freq);
            jntstate[2]=((T_Right_Lower_Shoulder_Link-Right_Lower_Shoulder_Link)/freq);

            jntstate[3]=((T_Right_Hip_Link - Right_Hip_Link)/freq);
            jntstate[4]=((T_Right_Thigh_Link - Right_Thigh_Link) / freq);
            jntstate[5]=((T_Right_Calf_Link - Right_Calf_Link) / freq);
            jntstate[6]=((T_Right_Foot_Link - Right_Foot_Link) / freq);

            jntstate[7]=((T_Left_Upper_Shoulder_Link - Left_Upper_Shoulder_Link)/freq);
            jntstate[8]=((T_Left_Mid_Shoulder_Link-Left_Mid_Shoulder_Link)/freq);
            jntstate[9]=((T_Left_Lower_Shoulder_Link-Left_Lower_Shoulder_Link)/freq);

            jntstate[10]=((T_Left_Hip_Link - Left_Hip_Link)/freq);
            jntstate[11]=((T_Left_Thigh_Link - Left_Thigh_Link) / freq);
            jntstate[12]=((T_Left_Calf_Link - Left_Calf_Link) / freq);
            jntstate[13]=((T_Left_Foot_Link - Left_Foot_Link) / freq);
            stable=false;
            kp=30.0 / freq;
            ki=1.5 / freq;
            kd=0.0 / freq;
            error_x=0;
            error_y=0;
            derivative_error_x=0;
            derivative_error_y=0;
            integral_error_x=0;
            integral_error_y=0;
            te_x=0.0;
            te_y=0.0;
            te_z=0.0;
            P_limit=0.2/freq;
            I_limit=4.0/freq;
            D_limit=20000.0/freq;

            for(int k=0 ; k<freq ; k++){
                kinematics_full::add_jointpose(jntstate);
                Right_Foot_Link = 0 - (Right_Thigh_Link + Right_Calf_Link);
                Left_Foot_Link = 0 - (Left_Thigh_Link + Left_Calf_Link);
                kinematics_full::set_kdjointpose();
                humanoid_CoM=kinematics_full::compute_com(state,Right_Leg_fk_solver,Left_Leg_fk_solver,Right_Arm_fk_solver,Left_Arm_fk_solver);
                centroid=kinematics_full::compute_centroid(state,0);
                humanoid_stability=kinematics_full::quick_is_stable(0);
                stable=humanoid_stability.check();
                error_x = humanoid_CoM.x - centroid.x;
                error_y = humanoid_CoM.y - centroid.y;
                derivative_error_x = (humanoid_CoM.x - centroid.x) - error_x;
                derivative_error_y = (humanoid_CoM.y - centroid.y) - error_y;
                integral_error_x += error_x;
                integral_error_y += error_y;
                P_x=(kp* error_x);
                P_y=(kp* error_y);
                I_x=(ki* integral_error_x);
                I_y=(ki* integral_error_y);
                D_x=(kd* derivative_error_x);
                D_y=(kd* derivative_error_y);
                if (P_x > P_limit) P_x = P_limit; else if (P_x < -P_limit) P_x = -P_limit;
                if (P_y > P_limit) P_y = P_limit; else if (P_y < -P_limit) P_y = -P_limit;
                if (I_x > I_limit) I_x = I_limit; else if (I_x < -I_limit) I_x = -I_limit;
                if (I_y > I_limit) I_y = I_limit; else if (I_y < -I_limit) I_y = -I_limit;
                if (D_x > D_limit) D_x = D_limit; else if (D_x < -D_limit) D_x = -D_limit;
                if (D_y > D_limit) D_y = D_limit; else if (D_y < -D_limit) D_y = -D_limit;

                if(state == 'r'){
                te_x = Left_Foot.p.data[0] - target[0];
                te_y = Left_Foot.p.data[1] - target[1];
                te_z = Left_Foot.p.data[2] - target[2];
                Right_Hip_Link += (P_y + I_y + D_y);
                Right_Thigh_Link -= (P_x + I_x + D_x);

                if (Left_Foot.p.data[1] < 0.16)
                {
                    Left_Hip_Link += kp / 3 * (0.16 - Left_Foot.p.data[1]);
                }
                Left_Hip_Link -= kp * te_y;
                Left_Thigh_Link += kp * te_x;
                Left_Calf_Link -= kp * te_z;
            }
            else if (state == 'l')
            {
                te_x = Right_Foot.p.data[0] - target[0];
                te_y = Right_Foot.p.data[1] - target[1];
                te_z = Right_Foot.p.data[2] - target[2];
                Left_Hip_Link += (P_y + I_y + D_y);
                Left_Thigh_Link -= (P_x + I_x + D_x);
                if (Right_Foot.p.data[1] > -0.16)
                {
                    Right_Hip_Link += kp / 3 * (-0.16 - Right_Foot.p.data[1]);
                }
                Right_Hip_Link -= kp * te_y;
                Right_Thigh_Link += kp * te_x;
                Right_Calf_Link -= kp * te_z;
            }
            else if (state == 'd')
            {
                Right_Hip_Link += (P_y + I_y + D_y);
                Right_Thigh_Link -= (P_x + I_x + D_x);
                Left_Hip_Link += (P_y + I_y + D_y);
                Left_Thigh_Link -= (P_x + I_x + D_x);
                }
            kinematics_full::joint_publish(rate);
            kinematics_full::com_publish(state,rate);
            }
            kinematics_full::set_T_equal_jointpose();
};
void get_path(const nav_msgs::Path::ConstPtr &msg, int sampling, ros::Rate *rate, kinematics_full *humanoid, KDL::ChainFkSolverPos_recursive *Right_Leg_fk_solver, KDL::ChainFkSolverPos_recursive *Left_Leg_fk_solver,KDL::ChainFkSolverPos_recursive *Right_Arm_fk_solver,KDL::ChainFkSolverPos_recursive *Left_Arm_fk_solver)
{


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
        humanoid->humanoid_will_go_on(foot_stance, Right_Leg_fk_solver, Left_Leg_fk_solver,Right_Arm_fk_solver,Left_Arm_fk_solver,rate, sampling * 1.5);
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
        humanoid->humanoid_will_go_on(foot_stance, Right_Leg_fk_solver, Left_Leg_fk_solver,Right_Arm_fk_solver,Left_Arm_fk_solver,rate, sampling);
        double tfoot[3] = {x, y, 0.0};
        humanoid->move_leg(tfoot, foot_stance, Right_Leg_fk_solver, Left_Leg_fk_solver,Right_Arm_fk_solver,Left_Arm_fk_solver,rate, sampling * 2);
        humanoid->humanoid_will_go_on('d', Right_Leg_fk_solver, Left_Leg_fk_solver,Right_Arm_fk_solver,Left_Arm_fk_solver,rate, sampling);
    }
    humanoid->T_Right_Thigh_Link = -0.3;
    humanoid->T_Right_Calf_Link = 0.55;
    humanoid->T_Left_Thigh_Link = -0.3;
    humanoid->T_Left_Calf_Link = 0.55;
    humanoid->humanoid_will_go_on('d', Right_Leg_fk_solver,Left_Leg_fk_solver, Right_Arm_fk_solver,Left_Arm_fk_solver,rate, 100);
    std::cout << "Finish! \n";
}

