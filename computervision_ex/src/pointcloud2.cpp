#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

// ros::Publisher depth_img_pub;
image_transport::Publisher depth_img_pub;

void depthImgCallback(const sensor_msgs::PointCloud2ConstPtr depth_msg)
{
  int row_step = depth_msg->row_step;
  int point_step = depth_msg->point_step;
  int num_of_width = row_step / point_step;
  int num_of_height = depth_msg->height;
  int first, 
      offset_x = depth_msg->fields[0].offset, 
      offset_y = depth_msg->fields[1].offset, 
      offset_z = depth_msg->fields[2].offset;
  float X = 0.0, Y = 0.0, Z = 0.0;

  cv::Mat depthImage(num_of_height, num_of_width, CV_32FC1);
  for(int r = 0; r < num_of_height; r++)
  {
    for(int c = 0; c < num_of_width; c++)
    {
      first = r*row_step + c*point_step;
      memcpy(&X, &depth_msg->data[first + offset_x], sizeof(float));
      memcpy(&Y, &depth_msg->data[first + offset_y], sizeof(float));
      memcpy(&Z, &depth_msg->data[first + offset_z], sizeof(float));
      depthImage.at<float>(r,c) = Z;
    }
  } 
  cv::imshow("depth image", depthImage);
  cv::waitKey(1);

  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, depthImage).toImageMsg();
  depth_img_pub.publish(img_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CompPub_node");  
  ros::NodeHandle n;

  ros::Subscriber depth_sub_ = n.subscribe("/camera/depth/points", 1, depthImgCallback);
  // depth_img_pub = n.advertise<sensor_msgs::Image>("/depth_imggg", 1);
  image_transport::ImageTransport it(n);
  depth_img_pub = it.advertise("/depth_imggg", 1);

  ros::spin();
  return 0;
}