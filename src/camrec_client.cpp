#include <ros/ros.h>
#include "autolanding/Camrec_srv.h"
#include <cstdlib>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "camrec_client");

    if (argc != 2)
    {
    ROS_INFO("usage: camrec_client X ");
    return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<autolanding::Camrec_srv>("Camrec_srv");

    autolanding::Camrec_srv srv;

    srv.request.OnOff = atoll(argv[1]);

    if (client.call(srv))
    {
      ROS_INFO("Send srv, srv.Request.OnOff : %d", srv.request.OnOff);
    }
    else
    {
      ROS_ERROR("Failed to call service Camrec_srv");
      return 1;
    }

    return 0;
}
