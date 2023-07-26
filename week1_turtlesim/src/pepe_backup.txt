#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Bool.h"
#include <std_srvs/Empty.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>
#include <sstream>

using namespace std;
#define PI 3.14159265359

bool first_start = true;
bool mission1 = false;
bool pepe_reset = false;
bool mission2 = false;

ros::ServiceClient clear;
ros::ServiceClient kill;
ros::ServiceClient spawn;

void triggerCallback(const std_msgs::Bool::ConstPtr& trigger)
{
    if(first_start)
    {
        mission1 = trigger->data;
        first_start = false;
    }
}

void move(ros::Publisher twist_pub, double linear, double angular)
{
    geometry_msgs::Twist pepe_go;
    pepe_go.linear.x = linear;
    pepe_go.angular.z = angular;
    twist_pub.publish(pepe_go);
}

void forward(ros::Publisher twist_pub, double length)
{
    move(twist_pub, length, 0);
}

void rotate(ros::Publisher twist_pub, int angle)
{
    move(twist_pub, 0.0, PI/180*angle);
}

void make_circle(ros::Publisher twist_pub, double angular)
{
    move(twist_pub, 2.0, angular);
}

void ready_for_next_pepe(void)
{
    //kill turtle1
    turtlesim::Kill pepe_kill;
    pepe_kill.request.name = "turtle1";
    kill.call(pepe_kill);
    //clear background
    std_srvs::Empty empty;
    clear.call(empty);
    //make turtle2
    turtlesim::Spawn new_pepe;
    new_pepe.request.name = "turtle1";
    new_pepe.request.x = 5.544445;
    new_pepe.request.y = 5.544445;
    spawn.call(new_pepe);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pepe");
    ros::NodeHandle n;

    bool stop = false;
    int count = 0, it_angle = 0, it_length = 0, it_circle = 1;
    int angle[5] = {90, 135, -90, 135, 90};
    double length[6] = {2.5, 2.5, 3.0, 3.0, 2.5, 2.5};
    double w, theta, t;

    ros::Subscriber trigger_sub = n.subscribe("/driver_start", 1000, triggerCallback);
    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
    clear = n.serviceClient<std_srvs::Empty>("clear");
    kill = n.serviceClient<turtlesim::Kill>("kill");
    spawn = n.serviceClient<turtlesim::Spawn>("spawn");

    ros::Rate rate(10);
    while(ros::ok())
    {
        if(mission1) 
        {
            if(count == 0) cout<<"start\r\n";
            if(count % 70 == 10) forward(twist_pub, length[it_length++]);
            if(count % 70 == 60) {rotate(twist_pub, angle[it_angle++]); count = 0;}
            count++;
            if(it_length == 6 && count == 60) {mission1 = false; pepe_reset = true; count = 0;}
        }
        else if(pepe_reset)
        {
            if(count == 10) 
            {
                pepe_reset = false; mission2 = true;
                ready_for_next_pepe();
                count = -1;
            }
            count++;
        }
        else if(mission2)
        {
            if(stop)
            {
                if(it_circle > 4) mission2 = false;
                if(count > 50) {stop = false; count = 0;}
                move(twist_pub, 0.0, 0.0);
            }
            else
            {
                switch(it_circle)
                {
                    case 1:
                    w = 1.0; theta = 2* PI;
                    break;

                    case 2:
                    w = -1.0; theta = PI;
                    break;

                    case 3:
                    w = -2.0; theta = PI;
                    break;

                    case 4:
                    w = 2.0; theta = PI;
                    break;
                }
                t = theta/abs(w) * 10;
                make_circle(twist_pub, w);

                if(count > t)
                {
                    if(it_circle != 3) stop = true; 
                    count = 0;
                    it_circle++;
                }
            }
            count++;
        }
        else move(twist_pub, 0.0, 0.0);
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}




