#include "pepe.h"

using namespace std;
#define PI 3.14159265359

Pepe::Pepe(ros::NodeHandle* n):n_(n)
{
    initializeSubscribers(); 
    initializePublishers();
    initializeServices();

    first_start = true;
    mission1 = false;
    pepe_reset = false;
    mission2 = false;

    stop = false;
    count = 0, it_angle = 0, it_length = 0, it_circle = 1;
    // double angle[5] = {90, 135, -90, 135, 90};
    // double length[6] = {2.5, 2.5, 3.0, 3.0, 2.5, 2.5};
}

void Pepe::initializeSubscribers()
{
    trigger_sub = n_->subscribe("/driver_start", 1000, &Pepe::triggerCallback, this); // "this" to refer to the current instance of Pepe (pepe의 주소를 가리킴)
}

void Pepe::initializePublishers()
{
    twist_pub = n_->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
}

void Pepe::initializeServices()
{
    clear = n_->serviceClient<std_srvs::Empty>("clear");
    kill = n_->serviceClient<turtlesim::Kill>("kill");
    spawn = n_->serviceClient<turtlesim::Spawn>("spawn");
}

void Pepe::triggerCallback(const std_msgs::Bool::ConstPtr& trigger)
{
    if(first_start)
    {
        mission1 = trigger->data;
        first_start = false;
    }
}

void Pepe::move(ros::Publisher twist_pub, double linear, double angular)       
{
    geometry_msgs::Twist pepe_go;
    pepe_go.linear.x = linear;
    pepe_go.angular.z = angular;
    twist_pub.publish(pepe_go);
}

void Pepe::forward(ros::Publisher twist_pub, double length)
{
    move(twist_pub, length, 0);
}

void Pepe::rotate(ros::Publisher twist_pub, double angle)
{
    move(twist_pub, 0.0, PI/180*angle);
}

void Pepe::make_circle(ros::Publisher twist_pub, double angular)
{
    move(twist_pub, 2.0, angular);
}

void Pepe::ready_for_next_pepe(void)
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

void Pepe::driving_test(void)
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
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pepe");  // pepe : node name
    ros::NodeHandle n;
    Pepe pepe(&n);

    ros::Rate rate(10);
    while(ros::ok())
    {
        pepe.driving_test();
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

