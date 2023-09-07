#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/master.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/make_shared.hpp>

// std::string output_frame_id_ = "camera_depth_frame";
int scan_height_ = 1, scan_h = 0.0;
double scan_time_ = 0.033, range_max_ = 5.0, range_min_ = 0.45;

image_geometry::PinholeCameraModel cam_model_1, cam_model_2;
ros::Publisher scan_pub1, scan_pub2;

double magnitude_of_ray(const cv::Point3d& ray) 
{
  return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}

double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2)
{
  const double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
  const double magnitude1 = magnitude_of_ray(ray1);
  const double magnitude2 = magnitude_of_ray(ray2);;
  return acos(dot_product / (magnitude1 * magnitude2));
} 

bool use_point(const float new_value, const float old_value, const float range_min, const float range_max) 
{
  // Check for NaNs and Infs, a real number within our limits is more desirable than these.
  const bool new_finite = std::isfinite(new_value);
  const bool old_finite = std::isfinite(old_value);

  // Infs are preferable over NaNs (more information)
  if(!new_finite && !old_finite)
  { // Both are not NaN or Inf.
    if(!std::isnan(new_value))
    { // new is not NaN, so use it's +-Inf value.
      return true;
    }
    return false; // Do not replace old_value
  }

  // If not in range, don't bother
  const bool range_check = range_min <= new_value && new_value <= range_max;
  if(!range_check)
  {
    return false;
  }
  if(!old_finite)
  { // New value is in range and finite, use it.
    return true;
  }

  // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
  const bool shorter_check = new_value < old_value;
  return shorter_check;
}

static inline bool valid(uint16_t depth) { return depth != 0; }
static inline float toMeters(uint16_t depth) { return depth * 0.001f; } 
static inline bool valid(float depth) { return std::isfinite(depth); }
static inline float toMeters(float depth) { return depth; }

template<typename T>
void convert(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& cam_model,
              const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height) 
{
  // Use correct principal point from calibration
  const float center_x = cam_model.cx();
  const float center_y = cam_model.cy() - scan_h;

  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  const int row_step = depth_msg->step / sizeof(T); //image.step : row length in bytes

  const int offset = (int)(center_y - scan_height/2);
  depth_row += offset*row_step; // Offset to center of image

  float min_r = INFINITY;
  for(int v = offset; v < offset+scan_height_; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)depth_msg->width; ++u) // Loop over each pixel in row
    {
      const T depth = depth_row[u];
      double r = depth; // Assign to pass through NaNs and Infs
      const double th = -atan2((double)(u - center_x) / cam_model.fx(), 1);
      const int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;

      if (valid(depth))
      { // Not NaN or Inf // Calculate in XYZ
        double x = toMeters( T((u - center_x) * depth / cam_model.fx()) );
        double z = toMeters(depth);
        // Calculate actual distance 빗변 길이
        r = hypot(x, z);
      }

      // Determine if this point should be used.
      if(use_point(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max))
      {
        scan_msg->ranges[index] = r;
        if(r < min_r) min_r = r;
        // std::cout<<"obstacle!!\n";
      }
    }//for u (cols)
  }//for v (rows)
  // std::cout<<min_r<<'\n';
}//convert

void depthImgCallback1(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Set camera model
  cam_model_1.fromCameraInfo(info_msg);
  
  // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
  cv::Point2d raw_pixel_left(0, cam_model_1.cy());
  cv::Point2d rect_pixel_left = cam_model_1.rectifyPoint(raw_pixel_left);
  cv::Point3d left_ray = cam_model_1.projectPixelTo3dRay(rect_pixel_left);

  cv::Point2d raw_pixel_right(depth_msg->width-1, cam_model_1.cy());
  cv::Point2d rect_pixel_right = cam_model_1.rectifyPoint(raw_pixel_right);
  cv::Point3d right_ray = cam_model_1.projectPixelTo3dRay(rect_pixel_right);

  cv::Point2d raw_pixel_center(cam_model_1.cx(), cam_model_1.cy());
  cv::Point2d rect_pixel_center = cam_model_1.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_1.projectPixelTo3dRay(rect_pixel_center);

  const double angle_max = angle_between_rays(left_ray, center_ray);
  const double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image
  
  // // Fill in laserscan message
  sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
  scan_msg->header = depth_msg->header;
  std::string output_frame_id_ = "camera_01_link";
  if(output_frame_id_.length() > 0)
  {
    scan_msg->header.frame_id = output_frame_id_;
  }
  scan_msg->angle_min = angle_min;
  scan_msg->angle_max = angle_max;
  scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (depth_msg->width - 1);
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // Check scan_height vs image_height
  if(scan_height_/2 > cam_model_1.cy() || scan_height_/2 > depth_msg->height - cam_model_1.cy())
  {
    std::stringstream ss;
    ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
    throw std::runtime_error(ss.str());
  }

  // Calculate and fill the ranges
  const uint32_t ranges_size = depth_msg->width;
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    convert<uint16_t>(depth_msg, cam_model_1, scan_msg, scan_height_);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    convert<float>(depth_msg, cam_model_1, scan_msg, scan_height_);
  }
  else
  {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }
  scan_pub1.publish(scan_msg);
}//depthImgCallback1

void depthImgCallback2(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Set camera model
  cam_model_2.fromCameraInfo(info_msg);
  
  // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
  cv::Point2d raw_pixel_left(0, cam_model_2.cy());
  cv::Point2d rect_pixel_left = cam_model_2.rectifyPoint(raw_pixel_left);
  cv::Point3d left_ray = cam_model_2.projectPixelTo3dRay(rect_pixel_left);

  cv::Point2d raw_pixel_right(depth_msg->width-1, cam_model_2.cy());
  cv::Point2d rect_pixel_right = cam_model_2.rectifyPoint(raw_pixel_right);
  cv::Point3d right_ray = cam_model_2.projectPixelTo3dRay(rect_pixel_right);

  cv::Point2d raw_pixel_center(cam_model_2.cx(), cam_model_2.cy());
  cv::Point2d rect_pixel_center = cam_model_2.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_2.projectPixelTo3dRay(rect_pixel_center);

  const double angle_max = angle_between_rays(left_ray, center_ray);
  const double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image
  
  // // Fill in laserscan message
  sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
  scan_msg->header = depth_msg->header;
  std::string output_frame_id_ = "camera_02_link";
  if(output_frame_id_.length() > 0)
  {
    scan_msg->header.frame_id = output_frame_id_;
  }
  scan_msg->angle_min = angle_min;
  scan_msg->angle_max = angle_max;
  scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (depth_msg->width - 1);
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // Check scan_height vs image_height
  if(scan_height_/2 > cam_model_2.cy() || scan_height_/2 > depth_msg->height - cam_model_2.cy())
  {
    std::stringstream ss;
    ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
    throw std::runtime_error(ss.str());
  }

  // Calculate and fill the ranges
  const uint32_t ranges_size = depth_msg->width;
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    convert<uint16_t>(depth_msg, cam_model_2, scan_msg, scan_height_);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    convert<float>(depth_msg, cam_model_2, scan_msg, scan_height_);
  }
  else
  {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }
  scan_pub2.publish(scan_msg);
}//depthImgCallback2

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_image_to_laser_scan"); 
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  ros::Rate rate(5);
  while(n.ok())
  {
    image_transport::CameraSubscriber cam_sub1 = it.subscribeCamera("/camera_01/depth/image_raw", 1, depthImgCallback1);
    image_transport::CameraSubscriber cam_sub2 = it.subscribeCamera("/camera_02/depth/image_raw", 1, depthImgCallback2);
    scan_pub1 = n.advertise<sensor_msgs::LaserScan>("scan1",10);
    scan_pub2 = n.advertise<sensor_msgs::LaserScan>("scan2",10);

    ros::spin();
    rate.sleep();
  }
}