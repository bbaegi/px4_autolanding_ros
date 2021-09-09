#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/String.h> 
#include <cmath>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/State.h>

#include "autolanding/Command_srv.h"
#include "autolanding/PIDtune_srv.h"

#include "apriltag_ros/AprilTagDetectionArray.h"
#include "autolanding/pid.h"

#define PI 3.14159265

double rad2deg(double radian)
{
    return radian*180/PI;
}
double deg2rad(double degree)
{
    return degree*PI/180;
}

struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(double w, double x, double y, double z) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

////////////////////Variable////////////////////
double cmd_pos_x = 0.0;
double cmd_pos_y = 0.0;
double cmd_pos_z = 0.0;
double cmd_pos_april_x = 0.0;

double cmd_ori_x = 0.0;
double cmd_ori_y = 0.0;
double cmd_ori_z = 0.0;
double cmd_ori_w = 0.0;

double cmd_ori_roll = 0.0;
double cmd_ori_pitch = 0.0;
double cmd_ori_yaw = 0.0;

int cmd_state = 0;
int cmd_setmode = 0;
int cmd_setparam = 0;
int cmd_arm = 0;

void do_command();
void wait_sequence();
void takeoff_sequence();
void setpos_sequence();
void return_sequence();
void landing_sequence();
void autolanding_sequence();

bool cmdreact(autolanding::Command_srv::Request &req,
              autolanding::Command_srv::Response &res)
{
    res.resp = true;

    cmd_state = req.command;
    ROS_INFO_STREAM("request : "<<req.command);

    return true;
}

//////////////////// TunePID Service server ////////////////////
double pid_dt = 0.01;
double pid_max = 0.2;
double pid_min = -0.2;
double pid_Kp = 0.1;
double pid_Kd = 0.0;
double pid_Ki = 0.0;

bool tunepid(autolanding::PIDtune_srv::Request &req,
              autolanding::PIDtune_srv::Response &res)
{
    res.resp = true;
    pid_dt = req.dt;
    pid_max = req.max;
    pid_min = req.min;
    pid_Kp = req.Kp;
    pid_Kd = req.Kd;
    pid_Ki = req.Ki;

    ROS_INFO("Set PID : % 3.2f % 3.2f % 3.2f % 3.2f % 3.2f % 3.2f", req.dt, req.max, req.min, req.Kp, req.Kd, req.Ki);
    return true;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

apriltag_ros::AprilTagDetectionArray current_apriltag_array;
geometry_msgs::Point current_apriltag_pose;
double current_apriltag_pose_x = 0.0;
double current_apriltag_pose_y = 0.0;
bool isdetectionOn = false;

void pose_ap(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    //current_apriltag_array = *msg;
    if(msg->detections.size()>=1)
    {
        isdetectionOn = true;

        current_apriltag_pose = msg->detections[0].pose.pose.pose.position;
        current_apriltag_pose_x = current_apriltag_pose.x;
        current_apriltag_pose_y = current_apriltag_pose.y;
        printf("apriltag position x val:% 7.3f, y val:% 7.3f\n", current_apriltag_pose_x, current_apriltag_pose_y);
    }
    else
    {
        isdetectionOn = false;
    }
    //printf("subscribe tag_detections\n");
}


void do_command()
{
    //Check command
    if(cmd_state == 0) // wait
    {
        wait_sequence();
    }

    else if(cmd_state == 1) // set mode Offboard
    {
        cmd_setmode = 1;
        cmd_setparam = 1;
    }

    else if(cmd_state == 2) // Motor arming, take off
    {
        cmd_arm = 1;
        takeoff_sequence();
    }

    else if(cmd_state == 3) // set position
    {
        setpos_sequence();
    }

    else if(cmd_state == 4) // return
    {
        return_sequence();
    }

    else if(cmd_state == 5) // landing
    {
        //landing_sequence();
        cmd_setmode = 5;
    }

    else if(cmd_state == 6) // auto landing
    {
        autolanding_sequence();
    }

    else
    {

    }
}

//////////////////// 0. Wait Command Sequence ////////////////////
void wait_sequence()
{
    if(cmd_setmode==1)
    {
        cmd_setmode = 0;
        ROS_INFO("Offboard enabled & set parameter finished, wait other command");
    }
    else if(cmd_arm==1)
    {
        cmd_arm = 0;
        ROS_INFO("Arming & Take Off enabled, wait other command");
    }
    else if(cmd_setmode==5)
    {
        ROS_INFO("Landing enabled, wait other command");
        if(!current_state.armed)
        {
            cmd_setmode = 1;
        }
    }
    else
    {
        //ROS_INFO("Wait command");
    }
}


//////////////////// 2. Takeoff Sequence ////////////////////
void takeoff_sequence()
{
    cmd_pos_z = 10;
}

//////////////////// 3. Set Position Sequence ////////////////////
PID pid = PID(0.01, 100, -100, 0.9, 0.0, 0.0);

void setpos_sequence()
{
    printf("Set position sequence\n");
    double pid_out_x = pid.calculate(15, current_pose.pose.position.x);
    double pid_out_y = pid.calculate(15, current_pose.pose.position.y);
    cmd_pos_x = 15.0 - pid_out_x;
    cmd_pos_y = 15.0 - pid_out_y;
    printf("val:% 7.3f inc:% 7.3f\n", current_pose.pose.position.x, cmd_pos_x);
}

//////////////////// 4. Return Sequence ////////////////////
void return_sequence()
{
    cmd_pos_x = 0.0;
    cmd_pos_y = 0.0;
}

//////////////////// 5. Landing Sequence ////////////////////
void landing_sequence()
{
    //printf("Landing sequence\n");
}

//////////////////// 6. Auto Landing Sequence ////////////////////
double apriltag_err_x = 0.0;
double apriltag_err_y = 0.0;
double target_pos_x = 0.0;
double target_pos_y = 0.0;
double target_pos_z = 0.0;

PID pid_altdown = PID(pid_dt, pid_max, pid_min, pid_Kp, pid_Kd, pid_Ki);
void autolanding_sequence()
{
    //printf("Auto landing sequence\n");
    if(current_pose.pose.position.z >= 3.5)
    {
        cmd_pos_z = 3.4;
    }
    else
    {
        if(isdetectionOn == true)
        {
            double current_yaw_rad = ToEulerAngles(current_pose.pose.orientation.w,
                                               current_pose.pose.orientation.x,
                                               current_pose.pose.orientation.y,
                                               current_pose.pose.orientation.z).yaw;
            double current_yaw_deg = rad2deg(current_yaw_rad);

            //PID pid_apriltag = PID(pid_dt, pid_max, pid_min, pid_Kp, pid_Kd, pid_Ki);
            //apriltag_err_x = -current_pose.pose.position.z*tan(current_apriltag_pose_y*0.5);
            //apriltag_err_y = -current_pose.pose.position.z*tan(current_apriltag_pose_x*0.5);
            apriltag_err_x = -current_apriltag_pose_y;
            apriltag_err_y = -current_apriltag_pose_x;
            double err_point_theta = atan2(apriltag_err_x, apriltag_err_y);
            double err_point_r = sqrt(apriltag_err_x*apriltag_err_x + apriltag_err_y*apriltag_err_y);
            double angle_calculated_err_x = 0.0;
            double angle_calculated_err_y = 0.0;

            angle_calculated_err_x = err_point_r * sin(err_point_theta - current_yaw_rad);
            angle_calculated_err_y = err_point_r * cos(err_point_theta - current_yaw_rad);


            target_pos_x = current_pose.pose.position.x + angle_calculated_err_x;
            target_pos_y = current_pose.pose.position.y + angle_calculated_err_y;
            //double pid_out_x = pid_apriltag.calculate(target_pos_x, current_pose.pose.position.x);
            //double pid_out_y = pid_apriltag.calculate(target_pos_y, current_pose.pose.position.y);
            cmd_pos_x = target_pos_x;//current_pose.pose.position.x + pid_out_x; //
            cmd_pos_y = target_pos_y;//current_pose.pose.position.y + pid_out_y; //

            printf("[yaw]cur:% 7.3f cmd: not yet\n", current_yaw_deg);
            printf("[x]cur:% 7.3f cmd:% 7.3f\n", current_pose.pose.position.x, cmd_pos_x);
            printf("[y]cur:% 7.3f cmd:% 7.3f\n", current_pose.pose.position.y, cmd_pos_y);
            printf("[apriltag error]x:% 7.3f y:% 7.3f\n", angle_calculated_err_x, angle_calculated_err_y);


            if(angle_calculated_err_x <= 0.5 &&
                    angle_calculated_err_x >=-0.5 &&
                    angle_calculated_err_y <= 0.5 &&
                    angle_calculated_err_y >=-0.5)
            {
                if(current_pose.pose.position.z >= 3.0)
                {
                    cmd_pos_z = 2.9;
                    printf("Phase 1, [z]cur:% 7.3f cmd:% 7.3f\n", current_pose.pose.position.z, cmd_pos_z);
                }
                else
                {
                    if(angle_calculated_err_x <= 0.25 &&
                            angle_calculated_err_x >=-0.25 &&
                            angle_calculated_err_y <= 0.25 &&
                            angle_calculated_err_y >=-0.25)
                    {
                        if(current_pose.pose.position.z >= 1.0)
                        {
                            target_pos_z = 0.9;
                            double pid_out_z = pid_altdown.calculate(target_pos_z, pid_out_z);
                            cmd_pos_z = pid_out_z;
                            /*double pid_out_z = pid_altdown.calculate(target_pos_z, current_pose.pose.position.z);
                            cmd_pos_z = current_pose.pose.position.z + pid_out_z;*/
                            printf("Phase 2, [z]cur:% 7.3f cmd:% 7.3f\n", current_pose.pose.position.z, cmd_pos_z);
                        }
                        else
                        {
                            if(angle_calculated_err_x <= 0.10 &&
                                    angle_calculated_err_x >=-0.10 &&
                                    angle_calculated_err_y <= 0.10 &&
                                    angle_calculated_err_y >=-0.10)
                            {
                                if(current_pose.pose.position.z >= 0.4)
                                {
                                    target_pos_z = 0.3;
                                    double pid_out_z = pid_altdown.calculate(target_pos_z, pid_out_z);
                                    cmd_pos_z = pid_out_z;
                                    /*
                                    double pid_out_z = pid_altdown.calculate(target_pos_z, current_pose.pose.position.z);
                                    cmd_pos_z = current_pose.pose.position.z + pid_out_z;*/
                                    printf("Phase 3, [z]cur:% 7.3f cmd:% 7.3f\n", current_pose.pose.position.z, cmd_pos_z);
                                }
                                else
                                {
                                    cmd_state = 5;
                                    printf("Phase 4, altitude less than 0.4m, Autolanding");
                                }
                            }
                            else
                            {
                               cmd_pos_z = current_pose.pose.position.z;
                            }
                        }
                    }
                    else
                    {
                      cmd_pos_z = current_pose.pose.position.z;
                    }
                }
            }
            else
            {
                cmd_pos_z = current_pose.pose.position.z;
            }

        }
        else
        {
            if(current_pose.pose.position.z < 0.4)
            {
                cmd_state = 5;
                printf("Phase 4, altitude less than 0.4m, Autolanding");
            }
            cmd_pos_z = current_pose.pose.position.z;
            //printf("no apriltag detection\n");
        }
    }
}

int val_MPC_XY_VEL_MAX = 0;
double val_real_MPC_XY_VEL_MAX = 0.0;
int val_MPC_LAND_SPEED = 0;
double val_real_MPC_LAND_SPEED = 0.0;

//////////////////// Main Loop ////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "autolanding");
    ros::NodeHandle n;

    ros::ServiceServer autolanding_server = n.advertiseService("Command_srv",cmdreact);
    ros::ServiceServer pidtune_server = n.advertiseService("PIDtune_srv",tunepid);
    ROS_INFO("ready srv server!");

    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);

    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber april_sub = n.subscribe<apriltag_ros::AprilTagDetectionArray>
            ("tag_detections", 10, pose_ap);

    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient set_param_client = n.serviceClient<mavros_msgs::ParamSet>
            ("mavros/param/set");
    ros::ServiceClient get_param_client = n.serviceClient<mavros_msgs::ParamGet>
            ("mavros/param/get");

    ros::Rate loop_rate(100);

    // wait for FCU connection
    int connect_cnt = 0; 

    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(connect_cnt == 50){
            printf("wait for FCU connection\n");
            connect_cnt = 0;
        }
        else
            connect_cnt++;
    }

    geometry_msgs::PoseStamped msg;

    mavros_msgs::SetMode offb_set_mode;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    mavros_msgs::ParamSet offb_set_param_MPC_XY_VEL_MAX;
    mavros_msgs::ParamSet offb_set_param_MPC_LAND_SPEED;
    mavros_msgs::ParamSet offb_set_param_MPC_Z_VEL_MAX_UP;

    mavros_msgs::ParamGet offb_get_param_MPC_XY_VEL_MAX;
    offb_get_param_MPC_XY_VEL_MAX.request.param_id = "MPC_XY_VEL_MAX";

    mavros_msgs::ParamGet offb_get_param_MPC_LAND_SPEED;
    offb_get_param_MPC_LAND_SPEED.request.param_id = "MPC_LAND_SPEED";

    int count = 1;

    //////////////////////////////////////////////////////////////////

    bool check_setmode = false;
    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(3.0)) && cmd_setmode == 1 )
        {
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                cmd_state = 0;
            }

            /*
            if(cmd_setparam == 1)
            {
                offb_set_param_MPC_XY_VEL_MAX.request.param_id = "MPC_XY_VEL_MAX";
                offb_set_param_MPC_XY_VEL_MAX.request.value.integer = 0;
                offb_set_param_MPC_XY_VEL_MAX.request.value.real = 3.0;
                if( set_param_client.call(offb_set_param_MPC_XY_VEL_MAX) &&
                                          offb_set_param_MPC_XY_VEL_MAX.response.success)
                {
                       ROS_INFO("Set Param MPC_VEL_MAX to % 7.3f", offb_set_param_MPC_XY_VEL_MAX.request.value.real);
                }

                offb_set_param_MPC_Z_VEL_MAX_UP.request.param_id = "MPC_Z_VEL_MAX_UP";
                offb_set_param_MPC_Z_VEL_MAX_UP.request.value.integer = 0;
                offb_set_param_MPC_Z_VEL_MAX_UP.request.value.real = 1.0;
                if( set_param_client.call(offb_set_param_MPC_Z_VEL_MAX_UP) &&
                                          offb_set_param_MPC_Z_VEL_MAX_UP.response.success)
                {
                       ROS_INFO("Set Param MPC_Z_VEL_MAX_UP to % 7.3f", offb_set_param_MPC_Z_VEL_MAX_UP.request.value.real);
                }

                offb_set_param_MPC_LAND_SPEED.request.param_id = "MPC_LAND_SPEED";
                offb_set_param_MPC_LAND_SPEED.request.value.integer = 0;
                offb_set_param_MPC_LAND_SPEED.request.value.real = 0.5;
                if( set_param_client.call(offb_set_param_MPC_LAND_SPEED) &&
                                          offb_set_param_MPC_LAND_SPEED.response.success)
                {
                       ROS_INFO("Set Param MPC_LAND_SPEED to % 7.3f", offb_set_param_MPC_LAND_SPEED.request.value.real);
                }
                cmd_setparam = 0;
            }
            */

            last_request = ros::Time::now();
        }

        else if( current_state.mode != "AUTO.LAND" &&
                    (ros::Time::now() - last_request > ros::Duration(3.0)) && cmd_setmode == 5 )
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Auto Landing enabled");
                cmd_pos_z = 0.0;
                cmd_state = 0;
            }
            last_request = ros::Time::now();
        }

        else
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(3.0)) && cmd_arm == 1 )
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");

                    cmd_ori_x = current_pose.pose.orientation.x;
                    cmd_ori_y = current_pose.pose.orientation.y;
                    cmd_ori_z = current_pose.pose.orientation.z;
                    cmd_ori_w = current_pose.pose.orientation.w;
                    cmd_state = 0;
                }
                last_request = ros::Time::now();
            }

            /*
            //////////////////// Get Parameter ////////////////////


            if ( get_param_client.call(offb_get_param_MPC_XY_VEL_MAX)&&
                 offb_get_param_MPC_XY_VEL_MAX.response.success)
            {
                val_MPC_XY_VEL_MAX = offb_get_param_MPC_XY_VEL_MAX.response.value.integer;
                val_real_MPC_XY_VEL_MAX = offb_get_param_MPC_XY_VEL_MAX.response.value.real;
                ROS_INFO("Get Param MPC_XY_VEL_MAX : % 7.3f", val_real_MPC_XY_VEL_MAX);
            }



            if ( get_param_client.call(offb_get_param_MPC_LAND_SPEED)&&
                 offb_get_param_MPC_LAND_SPEED.response.success)
            {
                val_MPC_LAND_SPEED = offb_get_param_MPC_LAND_SPEED.response.value.integer;
                val_real_MPC_LAND_SPEED = offb_get_param_MPC_LAND_SPEED.response.value.real;
                ROS_INFO("Get Param MPC_LAND_SPEED : % 7.3f", val_real_MPC_LAND_SPEED);
            }
            */
        }

        do_command();
        msg.header.stamp = ros::Time::now();
        msg.header.seq=count;
        msg.header.frame_id = 1;
        msg.pose.position.x = cmd_pos_x;
        msg.pose.position.y = cmd_pos_y;
        msg.pose.position.z = cmd_pos_z;

        //cmd_ori_yaw = deg2rad(290.0);
        //cmd_ori_x = ToQuaternion(cmd_ori_yaw, 0, 0).x;
        //cmd_ori_y = ToQuaternion(cmd_ori_yaw, 0, 0).y;
        //cmd_ori_z = ToQuaternion(cmd_ori_yaw, 0, 0).z;
        //cmd_ori_w = ToQuaternion(cmd_ori_yaw, 0, 0).w;

        msg.pose.orientation.x = cmd_ori_x;
        msg.pose.orientation.y = cmd_ori_y;
        msg.pose.orientation.z = cmd_ori_z;
        msg.pose.orientation.w = cmd_ori_w;

        chatter_pub.publish(msg);
        ros::spinOnce();
        count++;
        loop_rate.sleep();
    }
   
      
   return 0;
}
