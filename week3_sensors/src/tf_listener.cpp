#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"

geometry_msgs::PointStamped front_laser_point, rear_laser_point, depth_camera_point;

void transformPoint(const tf::TransformListener& listener){
  
  front_laser_point.header.frame_id = "front_lidar";
  front_laser_point.header.stamp = ros::Time();
  front_laser_point.point.x = 0.0;
  front_laser_point.point.y = 0.0;
  front_laser_point.point.z = 0.0;

  rear_laser_point.header.frame_id = "rear_lidar";
  rear_laser_point.header.stamp = ros::Time();

    geometry_msgs::PointStamped base_point1, base_point2, base_point3;
    listener.transformPoint("base_link", front_laser_point, base_point1);

    ROS_INFO("front_lidar: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f", 
        front_laser_point.point.x, front_laser_point.point.y, front_laser_point.point.z,
        base_point1.point.x, base_point1.point.y, base_point1.point.z, base_point1.header.stamp.toSec());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener))); //duration 호출간격

  ros::spin();

}