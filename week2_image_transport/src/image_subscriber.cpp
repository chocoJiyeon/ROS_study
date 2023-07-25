#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImage img;
  img.image = cv_bridge::toCvShare(msg, "bgr8")->image;
  
  uchar b,g,r;
  cv::Mat grey_img(img.image.rows, img.image.cols, CV_8UC1);
  for(int i=0; i<img.image.rows; i++)
  {
      for(int j=0; j<img.image.cols; j++)
      {
          b = img.image.at<cv::Vec3b>(i,j)[0];
          g = img.image.at<cv::Vec3b>(i,j)[1];
          r = img.image.at<cv::Vec3b>(i,j)[2];
          grey_img.at<uchar>(i,j) = uchar((0.299 * r) + (0.587 * g) + (0.114 * b));
      }
  }
  cv::imshow("grey image", grey_img);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImageSub_node");  
    ros::NodeHandle n;
    cv::namedWindow("grey image");
    cv::startWindowThread();

    // image_transport::ImageTransport it(n);
    // image_transport::Subscriber img_sub = it.subscribe("sensor_msgs/image", 5, imageCallback);
    ros::Subscriber sub = n.subscribe("sensor_msgs/image", 1, imageCallback);

    ros::spin();
    cv::destroyWindow("grey image");
    return 0;
}