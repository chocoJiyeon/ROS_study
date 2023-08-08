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
    cv::Mat scan_data(600,600,CV_8UC1,200);
    // std::cout<<scan->ranges.at(30)<<'\n';
    for(int i=0; i<360; i++)
    {
        range[i] = scan->ranges.at(i) * 500;    // 1 pixel == 2 mm
        if(range[i] < 300)
        {
            scan_data.at<uchar>((int)(range[i] *cos(i*PI/180)+300) , (int)(range[i] *sin(i*PI/180)+300)) = 0;
        }
    }
    
    cv::imshow("scan img",scan_data);
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