#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "aruco_maker_detection");  
  ros::NodeHandle n;
  
  cv::startWindowThread();
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) return 1;

  cv::Mat img;

  std::vector<int> markerIds;
  
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

  cv::Matx33f cameraMatrix(795.35846, 0, 639.5,
                0, 795.35846, 359.5,
                0, 0, 1);
  cv::Vec<float, 5> distCoeffs(-0.285412, 0.0766505, 0, 0, 1);
  std::vector<cv::Vec3d> rvecs, tvecs;

  ros::Rate rate(30);
  while(ros::ok() )
  {
    cap >> img;
    if(!img.empty())
    {
      
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
      cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
      cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
      std::cout<<tvecs[0]*100<<'\n';

      cv::imshow("img", img);
      cv::waitKey(1);
    } 
    ros::spinOnce();
    rate.sleep();
  }

  cv::destroyAllWindows();
  return 0;
}
