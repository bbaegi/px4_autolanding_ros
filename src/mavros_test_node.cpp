#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <cmath>

#include <std_msgs/String.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>

#include "autolanding/pid.h"

#define PI 3.14159265

class MavrosCommons
{
public:

    MavrosCommons()
    {
        // ROS services
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");
        set_param_client = nh.serviceClient<mavros_msgs::ParamSet>
                ("mavros/param/set");
        get_param_client = nh.serviceClient<mavros_msgs::ParamGet>
                ("mavros/param/get");

        // ROS Subscriber
        state_sub = nh.subscribe<mavros_msgs::State>
                    ("mavros/state", 10, &MavrosCommons::state_cb, this);
        ext_state_sub = nh.subscribe<mavros_msgs::ExtendedState>
                ("mavros/extended_state", 10, &MavrosCommons::ext_state_cb, this);
        alt_sub = nh.subscribe<mavros_msgs::Altitude>
                ("mavros/altitude", 10, &MavrosCommons::alt_cb, this);
        imu_sub = nh.subscribe<sensor_msgs::Imu>
                ("mavros/imu/data", 10, &MavrosCommons::imu_cb, this);
        pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", 10, &MavrosCommons::pose_cb, this);
        vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                ("mavros/local_position/velocity_local", 10, &MavrosCommons::vel_cb, this);

        ROS_INFO("Subscribe Topic Start!");
    }

    //functions
    void run();

    // Callback functions
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void alt_cb(const mavros_msgs::Altitude::ConstPtr& msg);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool get_arm(bool arm_in);
    std::string get_mode();






private:

    //Variable
    mavros_msgs::State current_state;
    mavros_msgs::ExtendedState current_ext_state;
    mavros_msgs::Altitude current_alt;
    sensor_msgs::Imu current_imu;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::TwistStamped current_vel;

    // ROS node handle
    ros::NodeHandle nh;

    // ROS services
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient set_param_client;
    ros::ServiceClient get_param_client;

    // ROS subscribers
    ros::Subscriber state_sub;
    ros::Subscriber ext_state_sub;
    ros::Subscriber alt_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub;

};

void MavrosCommons::run()
{
    ROS_INFO("Current State : %s", current_state.mode.c_str());
}

void MavrosCommons::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void MavrosCommons::ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){

    /*
    if(MavrosCommons::current_ext_state.landed_state != msg.landed_state)
    {
        ROS_INFO("landed state changed.");
    }
    */

    current_ext_state = *msg;
}

void MavrosCommons::alt_cb(const mavros_msgs::Altitude::ConstPtr& msg)
{
    current_alt = *msg;
}

void MavrosCommons::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_imu = *msg;
}

void MavrosCommons::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

void MavrosCommons::vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel = *msg;
}

// Helper methods
bool MavrosCommons::get_arm(bool arm_in)
{
    if(current_state.armed == arm_in)
        return true;
    else
        return false;
}

std::string MavrosCommons::get_mode()
{
    return current_state.mode;
}

//////////////////// Main Loop ////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_test_node");

    MavrosCommons mrc;

    ros::Rate loop_rate(100);

    while(ros::ok()){
        mrc.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
      
   return 0;
}
