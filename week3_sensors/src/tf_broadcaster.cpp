#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0.785398), tf::Vector3(0.07, 0.1, 0.045)), ros::Time::now(), "base_link", "front_lidar"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, -2.35619), tf::Vector3(-0.07, -0.1, 0.045)), ros::Time::now(), "base_link", "rear_lidar"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(1.5708, 0, -1.5708), tf::Vector3(0.1, 0, 0.035)), ros::Time::now(), "base_link", "depth_camera"));
    r.sleep();
  }
}