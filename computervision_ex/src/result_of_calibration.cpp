#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "calibration_result");  
  ros::NodeHandle n;

  cv::startWindowThread();
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) return 1;

  cv::Mat img, mapX, mapY;

  cv::Matx33f K(795.358459, 0, 639.5,
                0, 795.358459, 359.5,
                0, 0, 1);
  cv::Vec<float, 5> k(-0.285411835, 0.0766505376, 0, 0, 0);

  cv::Size frameSize(1280, 720);
  cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

  ros::Rate rate(30);
  while(ros::ok() )
  {
    cap >> img;
    if(!img.empty())
    {
      cv::imshow("original img", img);

      cv::Mat imgUndistorted;
      cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
      cv::imshow("calibrated img", imgUndistorted);
      cv::waitKey(30);
    } 
    ros::spinOnce();
    rate.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}

  // cv::Matx33f K(1056.3961, 0, 639.5,
  //               0, 1056.3961, 359.5,
  //               0, 0, 1);
  // cv::Vec<float, 5> k(-0.542427, 0.338683, 0, 0, 0);
  // cv::Matx33f K(772.17267, 0, 639.5,
  //               0, 772.17267, 359.5,
  //               0, 0, 1);
  // cv::Vec<float, 5> k(-0.242598, 0.0491639, 0, 0, 0);

  // cv::Matx33f K(990.89545, 0, 639.5,
  //               0, 990.89545, 359.5,
  //               0, 0, 1);
  // cv::Vec<float, 5> k(-0.402428, 0.135507, 0, 0, 0);
  // cv::Matx33f K(931.96875, 0, 639.5,
  //               0, 931.96875, 359.5,
  //               0, 0, 1);
  // cv::Vec<float, 5> k(-0.370029, 0.11703, 0, 0, 0);