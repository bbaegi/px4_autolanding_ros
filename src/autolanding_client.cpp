#include <ros/ros.h>
#include "autolanding/Command_srv.h"
#include <cstdlib>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "autolanding_client");

    if (argc != 2)
    {
    ROS_INFO("usage: autolanding_client X ");
    return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<autolanding::Command_srv>("Command_srv");

    autolanding::Command_srv srv;

    srv.request.command = atoll(argv[1]);

    if (client.call(srv))
    {
      ROS_INFO("Send srv, srv.Request.command : %d", (int)srv.request.command);
    }
    else
    {
      ROS_ERROR("Failed to call service Command_srv");
      return 1;
    }

    return 0;
}
