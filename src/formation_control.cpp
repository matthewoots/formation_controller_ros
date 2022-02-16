#include "formation_control.h"

FormationController::FormationController(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
{
    printf("%s[Formation Controller Constructor] Formation Controller is setting up for UAV %d\n", KGRN, uav_id);


    _nh.param<std::string>("agent_id", _id, "uav0");
    _nh.param<double>("publish_rate", pub_rate, 20);
    _nh.param<int>("uav_id", uav_id, 0);
    _nh.param<double>("offset_angle", offset_angle, 0.0);
    _nh.param<int>("agent_number", agent_number, 5);
    _nh.param<double>("radius", radius, 5);
    _nh.param<double>("x_offset", uav_offset.pose.position.x, 0.0);
    _nh.param<double>("y_offset", uav_offset.pose.position.y, 0.0);
    _nh.param<double>("z_offset", uav_offset.pose.position.z, 0.0);

    offset_angle = offset_angle/180.0;

    isLeader = !_id.compare("uav0"); // return 0 if two strings compare equal

    if(isLeader){
        printf("%sI am a leader!\n",KCYN);
    }
    else
    {
        printf("%sI am a follower! My ID is %d\n",KYEL,uav_id);
    }


    formation_position_pub = _nh.advertise<geographic_msgs::GeoPoseStamped>(
        "/" + _id + "/mavros/setpoint_position/global", 10);

    /** 
    * @brief Publisher that publishes control position setpoints to Mavros
    */
    local_position_pub = _nh.advertise<geometry_msgs::PoseStamped>(
        "/" + _id + "/mavros/setpoint_position/local", 10);
    
    /** 
    * @brief Get Mavros State of PX4
    */
    uav_state_sub = _nh.subscribe<mavros_msgs::State>(
        "/" + _id + "/mavros/state", 10, &FormationController::uavStateCb, this);

    leader_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/uav0/mavros/local_position/pose", 10, &FormationController::leaderPoseCb, this);

    leader_global_pos_sub = _nh.subscribe<sensor_msgs::NavSatFix>(
        "/uav0/mavros/global_position/global", 1, &FormationController::leaderGlobalPosCb, this);

    uav_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/" + _id + "/mavros/local_position/pose", 10, &FormationController::uavPoseCb, this);

    uav_global_pos_sub = _nh.subscribe<sensor_msgs::NavSatFix>(
        "/" + _id + "/mavros/global_position/global", 1, &FormationController::globalPosCb, this);

    user_cmd_sub = _nh.subscribe<std_msgs::Byte>(
        "/" + _id + "/user", 1, &FormationController::userCmdCb, this);

    /** 
    * @brief Service Client that handles arming in Mavros
    */
    arming_client = _nh.serviceClient<mavros_msgs::CommandBool>(
        "/" + _id + "/mavros/cmd/arming");

    /** 
    * @brief Service Client that handles mode switching in Mavros
    */
    set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>(
        "/" + _id + "/mavros/set_mode");

    mission_timer = _nh.createTimer(ros::Duration(1/pub_rate), &FormationController::missionTimer, this, false, false);


    printf("%s[Formation Controller Constructor] Formation Controller Setup for UAV %d is Ready! \n", KGRN, uav_id);
}

FormationController::~FormationController()
{
}

void FormationController::initialisation()
{
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();

    // compute the relative position w.r.t the leader using agent number

    double theta = 2 * PI * (uav_id-1)/(agent_number-1) + offset_angle;
    relative_x = radius * cos(theta);
    relative_y = radius * sin(theta);
    if(!isLeader)
    {
        printf("%sI am UAV %d\n", KGRN, uav_id);
        printf("%sMy relative x is %f\n", KGRN, relative_x);
        printf("%sMy relative y is %f\n\n", KGRN, relative_y);
    }


    // Make Sure FCU is connected, wait for 5s if not connected.
    printf("%s[formation_control.cpp] FCU Connection is %s \n", uav_current_state.connected? KBLU : KRED, uav_current_state.connected? "up" : "down");
    while (ros::ok() && !uav_current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            printf("%s[main.cpp] FCU not connected for 5s. Stop initialisation procedure \n", KRED);
            printf("%s[main.cpp] Check for FCU connection \n", KRED);
            last_request = ros::Time::now();
            return;
        }
    }

    printf("%s[main.cpp] FCU connected! \n", KBLU);
    _initialised = true;
    
    // Set Takeoff Position
    takeoff_pos.x() = uav_pose.pose.position.x;
    takeoff_pos.y() = uav_pose.pose.position.y;
    takeoff_pos.z() = uav_pose.pose.position.z + _takeoff_height;

    return;
}



void FormationController::uavStateCb(const mavros_msgs::State::ConstPtr &msg)
{
    uav_current_state = *msg;

    if (!_initialised)
        FormationController::initialisation();
}


void FormationController::globalPosCb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    uav_global_pos = *msg;
    latitude_curr = double(uav_global_pos.latitude);
    longitude_curr = double(uav_global_pos.longitude);
}

void FormationController::leaderGlobalPosCb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    leader_global_pos = *msg;
    // leader_latitude_curr = double(leader_global_pos.latitude);
    // leader_longitude_curr = double(leader_global_pos.longitude);
    // leader_altitude_curr = double(leader_global_pos.altitude);
    // LLtoUTM(lat, long, north, east, zone)
    //LLtoUTM(leader_latitude_curr, leader_longitude_curr, leader_utm_ny, leader_utm_ex, zone);
    LLtoUTM(leader_global_pos.latitude, leader_global_pos.longitude, leader_utm_ny, leader_utm_ex, zone);
    // printf("uav %d UTM E is %f\n", uav_id, leader_utm_ex);
    // printf("uav %d UTM N is %f\n", uav_id, leader_utm_ny);
}

void FormationController::leaderPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    leader_pose = *msg;
    leader_current_pos.x() = (double)leader_pose.pose.position.x;
    leader_current_pos.y() = (double)leader_pose.pose.position.y;
    leader_current_pos.z() = (double)leader_pose.pose.position.z;

    tf::Quaternion q(
        leader_pose.pose.orientation.x, leader_pose.pose.orientation.y,
        leader_pose.pose.orientation.z, leader_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(leader_roll, leader_pitch, leader_yaw);
}

void FormationController::uavPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pose = *msg;
    current_pos.x() = (double)uav_pose.pose.position.x;
    current_pos.y() = (double)uav_pose.pose.position.y;
    current_pos.z() = (double)uav_pose.pose.position.z;

    tf::Quaternion q(
        uav_pose.pose.orientation.x, uav_pose.pose.orientation.y,
        uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

bool FormationController::set_offboard()
{
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();

    home.pose.position.x = uav_pose.pose.position.x;
    home.pose.position.y = uav_pose.pose.position.y;
    home.pose.position.z = uav_pose.pose.position.z;
    home_yaw = yaw;

    std::cout << KYEL << "[main.cpp] Home Position : " << KNRM <<
    home.pose.position.x << " " << home.pose.position.y << " " <<
    home.pose.position.z << " " << home_yaw << std::endl;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_position_pub.publish(home);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    bool is_mode_ready = false;
    last_request = ros::Time::now();

    while (!is_mode_ready)
    {
        if (uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            printf("%s[main.cpp] Try set offboard \n", KYEL);
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                printf("%s[main.cpp] Offboard Enabled \n", KGRN);
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!uav_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                printf("%s[main.cpp] Try arm \n", KYEL);
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    printf("%s[main.cpp] Vehicle armed \n", KGRN);  
                }
                last_request = ros::Time::now();
            }
        }
        local_position_pub.publish(home);
        is_mode_ready = (uav_current_state.mode == "OFFBOARD") && uav_current_state.armed;
        ros::spinOnce();
        rate.sleep();
    }

    if (is_mode_ready)
    {
        printf("%s[main.cpp] Offboard mode activated! \n", KBLU);
    }

    return is_mode_ready;
}

void FormationController::missionTimer(const ros::TimerEvent &)
{
    switch (uavTaskState)
    {
    case kTakeOff:
        local_position_pub.publish(takeoff_position);
        //printf("Takeoff position z is %f\n", takeoff_position.pose.position.z);
        break;

    case kMission:
        // compute desired local/global positon if it is follower
        if(!isLeader)
        {
            // convert leader's current lat and long to UTM positions in meters
            // done in subscirber call back

            // compute ownself's desired UTM position by adding the relative_x, relative_y
            // which was computed during initilization
            // double desired_ex = leader_utm_ex + relative_x;
            // double desired_ny = leader_utm_ny + relative_y;

            // // convert desired current UTM position to lat and long
            // double desired_lat;
            // double desired_long;

            // UTMtoLL(desired_ny, desired_ex, zone, desired_lat,desired_long);

            // // publish the desired global position to /uavX/mavros/setpoint_position/global
            // // using formation_position_pub

            // geographic_msgs::GeoPoseStamped global_pos_desired;
            // global_pos_desired.pose.position.latitude = desired_lat;
            // global_pos_desired.pose.position.longitude = desired_long;
            // global_pos_desired.pose.position.altitude = leader_global_pos.altitude - 47.22; // offset from ellipsoid height to AMSL
            
            // global_pos_desired.pose.orientation = leader_pose.pose.orientation;
            // formation_position_pub.publish(global_pos_desired);

            double desired_global_pos_x = leader_current_pos.x() + relative_x;
            double desired_global_pos_y = leader_current_pos.y() + relative_y;
            desired_local_pose.pose.position.x = desired_global_pos_x - uav_offset.pose.position.x;
            desired_local_pose.pose.position.y = desired_global_pos_y - uav_offset.pose.position.y;
            desired_local_pose.pose.position.z = leader_pose.pose.position.z;
            desired_local_pose.pose.orientation = leader_pose.pose.orientation;
            local_position_pub.publish(desired_local_pose);
        }
    
    default:
        break;
    }

}

void FormationController::userCmdCb(const std_msgs::Byte::ConstPtr &msg)
{
    int cmd = msg->data;
    printf("%s[main.cpp] User command %d received \n", KBLU, cmd);
    int tries = 0;

    switch (cmd)
    {
    case TAKEOFF:
    {
        if (!uav_current_state.armed && retry < tries)
        {
            printf("%s[main.cpp] Vehicle is not armed, please ask safety pilot to arm the vehicle [Attempt %d] \n", KRED, retry + 1);
            printf("%s[main.cpp] Overwrite and arm in %d \n", KRED, tries - (retry));
            retry++;
            break;
        }

        uavTaskState = kTakeOff;
        printf("%s[main.cpp] Takeoff command received! \n", KYEL);
        arm_cmd.request.value = true;

        if (set_offboard())
        {
            printf("%s[main.cpp] Offboard mode activated going to run takeoff \n", KBLU);
            mission_timer.start();
            printf("%s[main.cpp] Mission timer started! \n", KGRN);
            takeoff_flag = true;
        }

        // Set Takeoff Position
        takeoff_pos.x() = uav_pose.pose.position.x;
        takeoff_pos.y() = uav_pose.pose.position.y;
        takeoff_pos.z() = uav_pose.pose.position.z + _takeoff_height;

        takeoff_position.pose.position.x = uav_pose.pose.position.x;
        takeoff_position.pose.position.y = uav_pose.pose.position.y;
        takeoff_position.pose.position.z = uav_pose.pose.position.z + 5;

        
        /** @brief Set up Takeoff Waypoints */
        MatrixXd wp = MatrixXd::Zero(3,1);
        wp.col(0) = takeoff_pos;
        std::cout << KBLU << "[main.cpp] " << "[Takeoff Waypoint] " << std::endl << KNRM << wp << std::endl;

        double clean_buffer = ros::Time::now().toSec();

        double buffer = 5.0;
        std::cout << KBLU << "[main.cpp] " << "Takeoff buffer of " << KNRM << buffer << KBLU << "s" << std::endl;
        double last_interval = ros::Time::now().toSec();
        
        // // while loop to clean out buffer for command for 5s
        // while (abs(clean_buffer - ros::Time::now().toSec()) < buffer)
        // {
        //     // WARNING : Publishing too fast will result in the mavlink bandwidth to be clogged up hence we need to limit this rate
        //     if (ros::Time::now().toSec() - last_interval > _send_desired_interval) 
        //     {
        //         Vector3d home_pose = {home.pose.position.x, home.pose.position.y, home.pose.position.z};
        //         uavDesiredControlHandler(home_pose, 
        //             Vector3d (0,0,0),
        //             Vector3d (0,0,0),
        //             home_yaw);
        //         // std::cout << KBLU << "[main.cpp] Publish buffer" << KNRM << home_pose << std::endl;
        //         last_interval = ros::Time::now().toSec();
        //     }
            
        // }

        break;
    }

    case MISSION:
    {
        if (!takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
            break;
        }
        printf("%s[main.cpp] Mission command received! \n", KYEL);
        printf("%s[main.cpp] Loading Trajectory... \n", KBLU);

        uavTaskState = kMission;

        break;
    }

    case LAND:
    {
        if (!takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
            break;
        }
        // Check if its at XY of home position, if not we send it to HOME then to land
        double sqdist_hpos = pow(takeoff_pos.x(),2) + pow(takeoff_pos.y(),2);
        double sqdist_pos = pow(uav_pose.pose.position.x,2) + pow(uav_pose.pose.position.y,2);

        if (sqdist_pos - sqdist_hpos > 1.0)
        {
            printf("%s[main.cpp] Call home first \n", KRED);
            printf("%s[main.cpp] Position not suitable for landing, no RTL enabled, at dist %lf \n", KRED, sqdist_pos - sqdist_hpos);
            break;
        }
        printf("%s[main.cpp] Land command received! \n", KYEL);
        printf("%s[main.cpp] Loading Trajectory... \n", KBLU);

        /** @brief Set up Land Waypoints */
        MatrixXd wp = MatrixXd::Zero(3,1);
        wp.col(0) = Vector3d (home.pose.position.x, home.pose.position.y, home.pose.position.z);
        std::cout << KBLU << "[main.cpp] " << "[Land Waypoint] " << std::endl << KNRM << wp << std::endl;

        printf("%s[main.cpp] UAV is landing! \n", KBLU);
        break;
    }

    default:
        break;
    }
}