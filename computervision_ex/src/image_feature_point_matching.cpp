#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <vector>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "image_feature_point_matching");  
  ros::NodeHandle n;

  cv::Mat srcImage1 = cv::imread("/home/cona/catkin_ws/src/jy_project/computervision_ex/images/keypoint_image/snack.jpg", cv::IMREAD_GRAYSCALE);  //큰 이미지
  cv::Mat srcImage2 = cv::imread("/home/cona/catkin_ws/src/jy_project/computervision_ex/images/keypoint_image/eye.jpg", cv::IMREAD_GRAYSCALE);    //찾을 작은 이미지
  if(srcImage1.empty() || srcImage2.empty())  return -1;

  //detect the keypoints & descriptors
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  cv::Mat descriptors1, descriptors2;

  cv::Ptr<cv::ORB> orbF = cv::ORB::create(1000);
  orbF->detectAndCompute(srcImage1, cv::noArray(), keypoints1, descriptors1); //(input image, mask, keypoint, descriptors)
  orbF->detectAndCompute(srcImage2, cv::noArray(), keypoints2, descriptors2);
  std::cout<<"ketpoints1.size() = "<<keypoints1.size()<<std::endl;

  // matching descriptor vectors (using flannIndex.knnSearch)
  int k = 2;  //num of nearest neighbors to search for
  cv::Mat indices, dists;
  cv::flann::Index flannIndex(descriptors1, cv::flann::LshIndexParams(12,20,2), cvflann::FLANN_DIST_HAMMING);
  flannIndex.knnSearch(descriptors2, indices, dists, k, cv::flann::SearchParams());

  std::vector<cv::DMatch> goodMatches;
  float nndrRatio = 0.6f;
  for(int i = 0; i < descriptors2.rows; i++)
  {
    float d1, d2;
    d1 = (float)dists.at<int>(i,0);
    d2 = (float)dists.at<int>(i,1);

    if(indices.at<int>(i,0) >= 0 && indices.at<int>(i,1) >= 0 && d1 <= nndrRatio*d2)
    {
      std::cout<<"i = "<<i<<", d1 = "<<d1<<std::endl;
      cv::DMatch match(i, indices.at<int>(i,0), d1);
      goodMatches.push_back(match);
    }
  }
  std::cout<<"goodMatches.size() = "<<goodMatches.size()<<std::endl;

  // draw good matches
  cv::Mat imgMaches;
  cv::drawMatches(srcImage2, keypoints2, srcImage1, keypoints1, goodMatches, imgMaches,
                  cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  if(goodMatches.size()< 4) return 0;

  // find homography btw keypoint1, keypoint2
  std::vector<cv::Point2f> obj, scene;
  for(int i = 0; i< goodMatches.size(); i++)
  {
    obj.push_back(keypoints2[goodMatches[i].queryIdx].pt);
    scene.push_back(keypoints1[goodMatches[i].queryIdx].pt);
  }
  cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);

  std::vector<cv::Point2f> objP(4);
  objP[0] = cv::Point2f(0,0);
  objP[1] = cv::Point2f(srcImage2.cols,0);
  objP[2] = cv::Point2f(srcImage2.cols,srcImage2.rows);
  objP[3] = cv::Point2f(0,srcImage2.rows);

  std::vector<cv::Point2f> sceneP(4);
  cv::perspectiveTransform(objP, sceneP, H);

  for(int i=0; i<4; i++)
  {
    sceneP[i] += cv::Point2f(srcImage2.cols, 0);
  }
  for(int i=0; i<4; i++)
  {
    cv::line(imgMaches, sceneP[i], sceneP[(i+1)%4], cv::Scalar(255,0,0), 4);
  }

  cv::imshow("img", imgMaches);
  cv::waitKey();

  return 0;
}