#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "CompPub_node");  
    
    ros::NodeHandle n;
    ros::Publisher img_pub = n.advertise<sensor_msgs::CompressedImage>("sensor_msgs/CompressedImage",1);
    cv::namedWindow("frame");
    cv::startWindowThread();
    
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) return 1;

    cv::Mat frame;
    sensor_msgs::CompressedImagePtr compressed_image;

    ros::Rate rate(30);
    while(ros::ok())
    {
        cap >> frame;
        if(!frame.empty())
        {
            cv::imshow("frame",frame);
            compressed_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8",frame).toCompressedImageMsg();
            img_pub.publish(compressed_image);
            cv::waitKey(1);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}