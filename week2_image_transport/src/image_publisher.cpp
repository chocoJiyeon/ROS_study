#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImagePub_node");  
    
    ros::NodeHandle n;
    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("sensor_msgs/image", 1);
    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub = it.advertise("sensor_msgs/image", 1);
    cv::namedWindow("view");
    cv::startWindowThread();
    
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) return 1;

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate rate(30);
    while(ros::ok())
    {
        cap >> frame;
        if(!frame.empty())
        {
            cv::imshow("frame",frame);
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",frame).toImageMsg();
            img_pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}