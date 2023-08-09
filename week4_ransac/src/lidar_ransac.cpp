#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud.h>
#include <vector>

#define PI 3.141592
int num_of_edge, num_of_it = 200, max_inlier_size = 10;
float range[360];
double tolerance = 3.0;

void ransac(cv::Mat scan_data, std::vector<std::pair<int,int>> edge, std::vector<std::pair<float,cv::Point>>* slope)
{
    for(int i=0; i< num_of_it; i++)
    {
        int rand_edge1, rand_edge2;
        int a1,a2,b1,b2;
        for(;;)
        {
            rand_edge1 = rand() % num_of_edge;
            rand_edge2 = rand() % num_of_edge;
            if(rand_edge1 != rand_edge2) break;
        }
        
        a1 = edge[rand_edge1].first;    b1 = edge[rand_edge1].second;
        a2 = edge[rand_edge2].first;    b2 = edge[rand_edge2].second;

        int cnt_inlier = 0;
        for(auto it : edge)
        {
            if((it.first == a1 && it.second == b1) || (it.first == a2 && it.second == b2)) break;
            int a,b;
            double distance;
            a = it.first; b = it.second;
            distance = abs((b2-b1)*a - (a2-a1)*b - a1*(b2-b1) + b1*(a2-a1))/std::sqrt(pow((b2-b1),2)+ pow((a2-a1),2));
            if(distance < tolerance)
            {
                cnt_inlier++;
            }
        }

        if(cnt_inlier > max_inlier_size)
        {
            float m = (float)(b2 - b1)/(float)(a2-a1);
            if(!slope->size()) slope->push_back(std::make_pair(m,cv::Point(a1,b1)));
            for(auto it : *slope)
            {
                if(abs(it.first - m)< 0.1) break;
                else  slope->push_back(std::make_pair(m,cv::Point(a1,b1)));
            }
        }
    }//for
}//ransac()

void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    cv::Mat scan_data(2000,2000,CV_8UC1,200);
    std::vector<std::pair<int,int>> edge;
    std::vector<std::pair<float,cv::Point>> slope;
    num_of_edge =0;

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

            edge.push_back(std::make_pair(r,c));
            num_of_edge++;
        }
    }

    ransac(scan_data, edge, &slope);
    for(auto it : slope)
    {
        cv::line(scan_data, cv::Point(1999, (int)((1999-it.second.y)/it.first) +it.second.x), cv::Point(0, (int)(-it.second.y/it.first) +it.second.x), cv::Scalar::all(100));
    }

    cv::namedWindow("ransac line detection",cv::WINDOW_NORMAL);
    cv::imshow("ransac line detection",scan_data);
    cv::resizeWindow("ransac line detection",600,600);
    cv::waitKey(1);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_line_detection");
    ros::NodeHandle n;
    ros::Subscriber laserscan = n.subscribe<sensor_msgs::LaserScan>("/scan",1, scanCallBack); 

    ros::spin();
    return 0;
}

