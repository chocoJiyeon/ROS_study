#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud.h>
#include <vector>

#define PI 3.141592
float range[360];

void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    cv::Mat scan_data(2000,2000,CV_8UC1,200);

    for(int i=0; i<360; i++)
    {
        range[i] = scan->ranges.at(i) * 1000;    // 1 pixel == 1 mm
        if(range[i] < 3000)
        {
            int r = (int)(range[i] *cos(i*PI/180)+1000);
            int c = (int)(range[i] *sin(i*PI/180)+1000);
            if(r < 0) r = 0;
            else if(r > 1999) r = 1999;
            if(c < 0) c = 0;
            else if(c > 1999) c = 1999;
            cv::line(scan_data,cv::Point(1000,1000),cv::Point(c,r), 255,3);
            cv::circle(scan_data,cv::Point(c,r),2,0,2);
        }
    }
    cv::namedWindow("lidar-based grid map",cv::WINDOW_NORMAL);
    cv::imshow("lidar-based grid map",scan_data);
    cv::resizeWindow("lidar-based grid map",200,200);
    cv::waitKey(1);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_grid_map");
    ros::NodeHandle n;
    ros::Subscriber laserscan = n.subscribe<sensor_msgs::LaserScan>("/scan",1, scanCallBack); 

    ros::spin();
    
    return 0;
}

