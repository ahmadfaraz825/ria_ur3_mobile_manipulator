#include <iostream>
#include <ros/ros.h>

#include <cstdlib>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/StreamRate.h>


//using namespace std;

int main(int argc, char **argv)
{
sleep(10);
    int rate = 10;
    ros::init(argc, argv, "mavros_service_call");
ros::NodeHandle n;
    ros::Rate r(rate);
ros::ServiceClient client = n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
mavros_msgs::StreamRate srv;
srv.request.stream_id = 0;
srv.request.message_rate = 10;
srv.request.on_off = 1;
if(client.call(srv)){
    ROS_INFO("Service called");
}else{
    ROS_INFO("Failed to call service");
}

 while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

return 0;
}
