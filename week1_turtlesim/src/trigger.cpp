#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pepe_trigger");
    ros::NodeHandle n;

    ros::Publisher trigger_pub = n.advertise<std_msgs::Bool>("/driver_start", 1000);
    ros::Rate loop_rate(1);
    
    while (ros::ok())
    {
        std_msgs::Bool trigger;
        trigger.data = true;
        trigger_pub.publish(trigger);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}