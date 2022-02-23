#ifndef FORMATION_CONTROL_H
#define FORMATION_CONTROL_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float32MultiArray.h>

#include <tf/tf.h>
#include <string>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;

#define PI 3.14159

enum ControlMode
{
    position,
    velocity
};

class FormationController
{
    private:
        ros::NodeHandle _nh; ///< ros node handle

        ros::Publisher setpoint_raw_pub;
        
        ros::Subscriber uav_global_sub, command_msg_sub;

                    
        geometry_msgs::PoseStamped leader_pose; // leader global pose
        geometry_msgs::PoseStamped uav_global_pose; // follower global pose
        geometry_msgs::PoseStamped uav_offset;

        ControlMode ctrlMode;

        ros::Timer mission_timer;

        string mode;

        double pub_rate; // mission timer interval

        double roll, pitch, yaw;
        double leader_roll, leader_pitch, leader_yaw;
        double offset_angle;

        Vector3d relative;
        Vector3d leader_nwu_vel;
        
        Vector3d leader_current_pos, current_pos;

        std::string prefix ="S";
        

    public:

        ros::Subscriber leader_msg_sub, leader_enu_vel_sub;        

        std::string _id; ///< individual UAV id

        std::string identifier = prefix + "0";
        
        FormationController(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            _nh.param<std::string>("agent_id", _id, "0");
            _nh.param<double>("publish_rate", pub_rate, 30.0);
            _nh.param<std::string>("control_mode", mode, "position");

            if (!mode.compare("position"))
                ctrlMode = position;
            if (!mode.compare("velocity"))
                ctrlMode = velocity;

            printf("%s[formation_control_standalone.h] Formation Controller is setting up for UAV %s\n", KGRN, _id.c_str());
        
            /* ------------ Publishers ------------ */
            setpoint_raw_pub = _nh.advertise<mavros_msgs::PositionTarget>(
                "/" + _id + "/formation/setpoint_raw", 10);
            
            
            /* ------------ Subscribers ------------ */
            uav_global_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
                "/" + _id + "/global_pose", 10, &FormationController::uavGlobalPoseCb, this);
            command_msg_sub = _nh.subscribe<std_msgs::Float32MultiArray>(
                "/" + _id + "/global_planner_formation", 10, &FormationController::commandPoseCb, this);


            mission_timer = _nh.createTimer(ros::Duration(1/pub_rate), 
                &FormationController::missionTimer, this, false, false);

            mission_timer.start();

            printf("%s[formation_control_standalone.h] Formation Controller Setup for UAV %s is Ready! \n", KGRN, _id.c_str());
        }

       ~FormationController(){}


        void missionTimer(const ros::TimerEvent &)
        {
            mavros_msgs::PositionTarget setpoint_raw;

            setpoint_raw.header.stamp = ros::Time::now();

            switch (ctrlMode)
            {

            case position:
            {
                // Get offset from the offset message
                setpoint_raw.position.x = leader_current_pos.x() + relative.x();
                setpoint_raw.position.y = leader_current_pos.y() + relative.y();
                setpoint_raw.position.z = leader_current_pos.z();
                
                setpoint_raw.yaw = leader_yaw;

                setpoint_raw.type_mask = 2552;
                setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                setpoint_raw_pub.publish(setpoint_raw);
                break;
            }

            case velocity:
            // Simple P-controller, output velocity commands
            {
                // Get offset from the offset message
                setpoint_raw.position.x = leader_current_pos.x() + relative.x();
                setpoint_raw.position.y = leader_current_pos.y() + relative.y();
                setpoint_raw.position.z = leader_current_pos.z();

                setpoint_raw.velocity.x = leader_nwu_vel.x() * 0.5;
                setpoint_raw.velocity.y = leader_nwu_vel.y() * 0.5;
                // setpoint_raw.velocity.z = leader_vel_enu.twist.linear.z;

                setpoint_raw.yaw = leader_yaw;

                setpoint_raw.type_mask = 2528;
                setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                setpoint_raw_pub.publish(setpoint_raw);

                break;
            }
            
            default:
                break;
            }

            // printf("%s[formation_control_standalone.h] formation pub %s %s|%s %f %f %f\n", KBLU, KNRM, _id.c_str(), identifier.c_str(), setpoint_raw.position.x, setpoint_raw.position.y, setpoint_raw.position.z);
            // printf("%s[formation_control_standalone.h] leader pose %s %s %lf %lf %lf\n", KBLU, KNRM, _id.c_str(), leader_current_pos.x(), leader_current_pos.y(), leader_current_pos.z());
        }

        /**
         * @brief Get UAV current global location
         */
        void uavGlobalPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            uav_global_pose = *msg;
        }

        /** 
        * @brief Get current agent velocity
        */
        void uavleaderEnuVelCb(const geometry_msgs::TwistStamped::ConstPtr &msg)
        {
            // printf("%s[formation_control_standalone.h] Leader uavleaderEnuVelCb %s %s\n", KBLU, KNRM, _id.c_str());

            geometry_msgs::TwistStamped uav_enu_vel = *msg;
            leader_nwu_vel.x() = (double)uav_enu_vel.twist.linear.y;
            leader_nwu_vel.y() = -(double)uav_enu_vel.twist.linear.x;
            leader_nwu_vel.z() = (double)uav_enu_vel.twist.linear.z;
        }

        /**
         * @brief Get leader uav current global location
         */
        void uavLeaderGlobalPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            // To update
            leader_pose = *msg;
            leader_current_pos.x() = (double)leader_pose.pose.position.x;
            leader_current_pos.y() = (double)leader_pose.pose.position.y;
            leader_current_pos.z() = (double)leader_pose.pose.position.z;

            tf::Quaternion q(
                leader_pose.pose.orientation.x, leader_pose.pose.orientation.y,
                leader_pose.pose.orientation.z, leader_pose.pose.orientation.w);
            tf::Matrix3x3 m(q);

            m.getRPY(leader_roll, leader_pitch, leader_yaw);
            // printf("%s[formation_control_standalone.h] Leader uavLeaderGlobalPoseCb %s %s %lf %lf %lf\n", KBLU, KNRM, _id.c_str(), leader_current_pos.x(), leader_current_pos.y(), leader_current_pos.z());
        }

        /**
         * @brief Get leader and offset message (only message and leader)
         */
        // void commandPoseCb(const std_msgs::Float32MultiArray::ConstPtr &msg)
        // {
        //     std_msgs::Float32MultiArray multi_array = *msg;
            
        //     int integer_id = (int)(multi_array.data[0]);
        //     identifier = prefix + to_string(integer_id);

        //     relative.x() = (double)multi_array.data[1];
        //     relative.y() = (double)multi_array.data[2];
        //     relative.z() = (double)multi_array.data[3];

        // }

        /**
         * @brief Get leader and offset message (Combined message with px4_path_planner)
         */
        void commandPoseCb(const std_msgs::Float32MultiArray::ConstPtr &msg)
        {
            std_msgs::Float32MultiArray multi_array = *msg;

            // Reject data if leader is not in formation size
            if ((int)(multi_array.data[5]) > 30)
                return;
                     
            int integer_id = (int)(multi_array.data[5]);
            identifier = prefix + to_string(integer_id);

            relative.x() = (double)multi_array.data[6];
            relative.y() = (double)multi_array.data[7];
            relative.z() = (double)multi_array.data[8];

        }


};

#endif
