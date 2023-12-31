#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>

using namespace std;

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "aruco_maker_detection");  
  ros::NodeHandle n;
  
  cv::startWindowThread();
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) return 1;

  std::cout<<fixed;
  std::cout.precision(3);
  std::cout.setf(ios::showpos);

  cv::Mat img;

  std::vector<int> markerIds;
  
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  //dictionary size : number of markers

  cv::Matx33f cameraMatrix(795.358459, 0, 639.5,
                            0, 795.358459, 359.5,
                            0, 0, 1);
  cv::Vec<float, 5> distCoeffs(-0.285411835, 0.0766505376, 0, 0, 0);
  std::vector<cv::Vec3d> rvecs, tvecs;

  ros::Rate rate(30);
  while(ros::ok() )
  {
    cap >> img;
    if(!img.empty())
    {
      
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
      if(markerIds.size())
      {
        cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.07, cameraMatrix, distCoeffs, rvecs, tvecs);
        cv::aruco::drawAxis(img, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1);
        std::cout<<"[x, y, z] = "<<tvecs[0]*100<<"     "<<"[yaw pitch roll] ="<<rvecs[0]*180/3.141592<<'\n';
      }
      

      cv::imshow("img", img);
      cv::waitKey(1);
    } 
    ros::spinOnce();
    rate.sleep();
  }

  cv::destroyAllWindows();
  return 0;
}
