
#include "formation.h"
#include "formation_control_standalone.h"
#include "csv.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_controller");
    ros::NodeHandle nh("~");
    // The setpoint publishing rate MUST be faster than 2Hz
    FormationController formationController(nh);
    
    while (ros::ok())
    {
        string identifier = formationController.identifier;
        // Lone subscriber to leader pose
        formationController.leader_msg_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "/" + identifier + "/global_pose", 100, &FormationController::uavLeaderGlobalPoseCb, &formationController);
        // leader_msg_sub.shutdown();

        boost::shared_ptr<geometry_msgs::PoseStamped const> sub_pose;
        geometry_msgs::PoseStamped pose;
        sub_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                "/" + identifier + "/global_pose", nh);

        // Lone subscriber to leader vel
        formationController.leader_enu_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(
            "/" + identifier + "/mavros/local_position/velocity_local", 100, &FormationController::uavleaderEnuVelCb, &formationController);
        
        boost::shared_ptr<geometry_msgs::TwistStamped const> sub_vel;
        geometry_msgs::TwistStamped vel;
        sub_vel = ros::topic::waitForMessage<geometry_msgs::TwistStamped>(
                "/" + identifier + "/mavros/local_position/velocity_local", nh);
        // if(sub_vel != NULL){
        //     vel = *sub_vel;
        //     printf("vel here\n");
        // }


        // printf("%s[main_standalone.cpp] Leader uavLeaderGlobalPoseCb %s %s|%s\n", KBLU, KNRM, formationController._id.c_str(), 
        //     formationController.identifier.c_str());

        ros::spinOnce();
    }

    return 0;
}

