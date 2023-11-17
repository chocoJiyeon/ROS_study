#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

#define ROBOT 1

int num_of_images = 20;
int cnt =0;
bool keep_capture = true;

// *** 인코딩 방식을 상수화하는 함수 *** //
int encoding_mat_type(const std::string & encoding)
{
  if (encoding == "mono8")
  {
    return CV_8UC1;
  }
  else if (encoding == "bgr8")
  {
    return CV_8UC3;
  }
  else if (encoding == "mono16")
  {
    return CV_16SC1;
  }
  else if (encoding == "rgba8")
  {
    return CV_8UC4;
  }
  else if (encoding == "bgra8")
  {
    return CV_8UC4;
  }
  else if (encoding == "32FC1")
  {
    return CV_32FC1;
  }
  else if (encoding == "rgb8")
  {
    return CV_8UC3;   // color의 image_raw 인코딩 형식
  }
  else if (encoding == "16UC1")
  {
    return CV_16UC1;  // depth의 image_raw 인코딩 형식
  }
  else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

void camera_scan_callback(const sensor_msgs::Image::ConstPtr& image_messages)
{
  cv::Mat frame(
  image_messages->height - 80,  // 480px - 80px
  image_messages->width,        // 640px
  encoding_mat_type(image_messages->encoding),
  const_cast<unsigned char*>(image_messages->data.data()),
  image_messages->step
  );

  if(keep_capture)
  {
    cv::imshow("capture img", frame);
    int key = cv::waitKey(60);
    if(key != -1)
    {
      cnt++;
      std::string img_name = "cap_img"+ std::to_string(cnt) + ".png";
      cv::imwrite(img_name,frame);
      std::cout<<cnt<<'\n';
    }
    if(cnt >= num_of_images) keep_capture = false;
  }
}//camera_scan_callback

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "camera_calibration");  
  ros::NodeHandle n;

  cv::startWindowThread();
  if(!ROBOT)
  {
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) return 1;

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
  }//not robot
  

  if(ROBOT) 
  {
    ros::Subscriber camera_subscriber = n.subscribe("/camera_topRGBD/color/image_raw", 1, camera_scan_callback);
    ros::spin();
  }
  cv::destroyAllWindows();
  return 0;
}

