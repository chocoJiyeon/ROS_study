#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "laser_geometry/laser_geometry.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>


sensor_msgs::PointCloud cloud1,cloud2, scan1, scan2, cloud_cam, scan_cam;
sensor_msgs::PointCloud2 scan_cam2;
geometry_msgs::PointStamped front_laser_point, rear_laser_point, depth_camera_point;
geometry_msgs::PointStamped base_point1, base_point2, base_point3;

class Scan{
	public:
		Scan(ros::NodeHandle* n);
        ros::Publisher scan_pub1, scan_pub2, cam_pub;
	private:
		ros::Subscriber scan_sub1, scan_sub2, cam_sub;
		void scan1CallBack(const sensor_msgs::LaserScan::ConstPtr& scan_f);
        void scan2CallBack(const sensor_msgs::LaserScan::ConstPtr& scan_r);
        void depthImgCallback(const sensor_msgs::PointCloud2ConstPtr depth_msg);
};

Scan::Scan(ros::NodeHandle* n)
{
	scan_pub1 = n->advertise<sensor_msgs::PointCloud>("/tf_scan_front",1);
	scan_sub1 = n->subscribe<sensor_msgs::LaserScan>("/group2/scan",1, &Scan::scan1CallBack, this); 

    scan_pub2 = n->advertise<sensor_msgs::PointCloud>("/tf_scan_rear",1);
	scan_sub2 = n->subscribe<sensor_msgs::LaserScan>("/group1/scan",1, &Scan::scan2CallBack, this); 

    cam_pub = n->advertise<sensor_msgs::PointCloud2>("/tf_depth_point",1);
	cam_sub = n->subscribe("/camera/depth/points", 1, &Scan::depthImgCallback, this);
}

void Scan::scan1CallBack(const sensor_msgs::LaserScan::ConstPtr& scan_f)
{
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    projector.projectLaser(*scan_f, cloud1);
    scan_pub1.publish(scan1);
    // scan_pub2.publish(cloud1);
}
void Scan::scan2CallBack(const sensor_msgs::LaserScan::ConstPtr& scan_r)
{
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    projector.projectLaser(*scan_r, cloud2);
    scan_pub2.publish(scan2);
}

void Scan::depthImgCallback(const sensor_msgs::PointCloud2ConstPtr depth_msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(*depth_msg, cloud_cam);
    sensor_msgs::convertPointCloudToPointCloud2(scan_cam, scan_cam2);
    cam_pub.publish(scan_cam2);
}

void transformPoint(const tf::TransformListener& listener)
{
    front_laser_point.header.frame_id = "front_lidar";
    front_laser_point.header.stamp = ros::Time();
    int range1 = cloud1.points.size();
    scan1 = cloud1;
    scan1.header.frame_id = "base_link";
    for(int i=0; i<range1; i++)
    {
        front_laser_point.point.x = cloud1.points[i].x;
        front_laser_point.point.y = cloud1.points[i].y;
        front_laser_point.point.z = cloud1.points[i].z;
        listener.transformPoint("base_link", front_laser_point, base_point1);
        scan1.points[i].x = base_point1.point.x;
        scan1.points[i].y = base_point1.point.y;
        scan1.points[i].z = base_point1.point.z;
    }
    
    rear_laser_point.header.frame_id = "rear_lidar";
    rear_laser_point.header.stamp = ros::Time();
    int range2 = cloud2.points.size();
    scan2 = cloud2;
    scan2.header.frame_id = "base_link";
    for(int i=0; i<range2; i++)
    {
        rear_laser_point.point.x = cloud2.points[i].x;
        rear_laser_point.point.y = cloud2.points[i].y;
        rear_laser_point.point.z = cloud2.points[i].z;
        listener.transformPoint("base_link", rear_laser_point, base_point2);
        scan2.points[i].x = base_point2.point.x;
        scan2.points[i].y = base_point2.point.y;
        scan2.points[i].z = base_point2.point.z;
    }

    depth_camera_point.header.frame_id = "depth_camera";
    depth_camera_point.header.stamp = ros::Time();
    int range3 = cloud_cam.points.size();
    scan_cam = cloud_cam;
    scan_cam.header.frame_id = "base_link";
    for(int i=0; i<range3; i++)
    {
        depth_camera_point.point.x = cloud_cam.points[i].x;
        depth_camera_point.point.y = cloud_cam.points[i].y;
        depth_camera_point.point.z = cloud_cam.points[i].z;
        listener.transformPoint("base_link", depth_camera_point, base_point3);
        scan_cam.points[i].x = base_point3.point.x;
        scan_cam.points[i].y = base_point3.point.y;
        scan_cam.points[i].z = base_point3.point.z;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_coordinate"); 
    ros::NodeHandle n;
	Scan Scan(&n);

    tf::TransformListener listener(ros::Duration(1));
    ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener)));

	ros::spin();
}