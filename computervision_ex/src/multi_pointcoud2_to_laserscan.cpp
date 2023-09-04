#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>

class Pc2_to_ls
{
  public:
    Pc2_to_ls();
    void scanCallback(const sensor_msgs::PointCloud2 &cloud, std::string topic);

  private:
    ros::NodeHandle n;
    ros::Publisher scan_pub;

    void pointcloud2_topic_parser();
    std::string destination_frame;
    std::string cloud_destination_topic;
    std::string scan_destination_topic;
    std::string pointcloud2_topics;

    std::vector<bool> clouds_modified;
    std::vector<ros::Subscriber> cloud2_subscribers;
    // std::vector<pcl::PCLPointCloud2> clouds;
    std::vector<sensor_msgs::LaserScan> scans;
    std::vector<std::string> input_topics;
};

Pc2_to_ls::Pc2_to_ls()
{
  this->pointcloud2_topic_parser();
  scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_destination_topic.c_str(), 1, false);
}

void Pc2_to_ls::pointcloud2_topic_parser()
{
  // LaserScan topics to subscribe
	ros::master::V_TopicInfo topics;

	std::istringstream iss(pointcloud2_topics);
	std::set<std::string> tokens;
	std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::inserter<std::set<std::string>>(tokens, tokens.begin()));
	std::vector<std::string> tmp_input_topics;

  while (!tokens.empty())
	{
		ROS_INFO("Waiting for topics ...");
		ros::master::getTopics(topics);
		sleep(1);

		for (int i = 0; i < topics.size(); i++)
		{
			if (topics[i].datatype == "/camera/depth/points" && tokens.erase(topics[i].name) > 0)	
			{
				tmp_input_topics.push_back(topics[i].name);
			}
		}
	}

  sort(tmp_input_topics.begin(), tmp_input_topics.end());
	std::vector<std::string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

  if ((tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
	{
		// Unsubscribe from previous topics
		for (int i = 0; i < cloud2_subscribers.size(); i++)
		{	cloud2_subscribers[i].shutdown(); }

		input_topics = tmp_input_topics;

		if (input_topics.size() > 0)
		{
			cloud2_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			scans.resize(input_topics.size());
			ROS_INFO("Subscribing to topics\t%ld", cloud2_subscribers.size());
			for (int i = 0; i < input_topics.size(); ++i)
			{
				cloud2_subscribers[i] = n.subscribe<sensor_msgs::PointCloud2>(input_topics[i].c_str(), 1, boost::bind(&Pc2_to_ls::scanCallback, this, input_topics[i]));
				clouds_modified[i] = false;
				std::cout << input_topics[i] << " ";
			}
		}
		else
			ROS_INFO("Not subscribed to any topic.");
	}
}//pointcloud2_topic_parser()

void Pc2_to_ls::scanCallback(const sensor_msgs::PointCloud2 &cloud, std::string topic)
{
  
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc2_to_laserscan");

	ros::spin();
	return 0;
}
