#include <ros/ros.h>
#include "autolanding/PIDtune_srv.h"
#include <cstdlib>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pidtune_client");

    if (argc != 7)
    {
    ROS_INFO("usage: pidtune_client dt max min Kp Kd Ki ");
    return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<autolanding::PIDtune_srv>("PIDtune_srv");

    autolanding::PIDtune_srv srv;

    srv.request.dt = atof(argv[1]);
    srv.request.max = atof(argv[2]);
    srv.request.min = atof(argv[3]);
    srv.request.Kp = atof(argv[4]);
    srv.request.Kd = atof(argv[5]);
    srv.request.Ki = atof(argv[6]);

    if (client.call(srv))
    {
      ROS_INFO("Send srv, dt max min Kp Kd Ki : %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f", (double)srv.request.dt, (double)srv.request.max, (double)srv.request.min, (double)srv.request.Kp, (double)srv.request.Kd, (double)srv.request.Ki);
    }
    else
    {
      ROS_ERROR("Failed to call service PIDtune_srv");
      return 1;
    }

    return 0;
}
