#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "camera_calibration");  
  ros::NodeHandle n;

  cv::startWindowThread();
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) return 1;

  int num_of_images = 20;
  int cnt =0;
  cv::Mat img;

  ros::Rate rate(30);
  while(ros::ok() )
  {
    cap >> img;
    if(!img.empty())
    {
      cv::imshow("capture img", img);
      int key = cv::waitKey(60);
      if(key != -1)
      {
        cnt++;
        std::string img_name = "cap_img"+ std::to_string(cnt) + ".png";
        cv::imwrite(img_name,img);
        std::cout<<cnt<<'\n';
      }
      if(cnt >= num_of_images) break; 
    } 
    ros::spinOnce();
    rate.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}