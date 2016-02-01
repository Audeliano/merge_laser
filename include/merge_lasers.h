#ifndef MERGE_LASERS_H
#define MERGE_LASERS_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <tf/transform_broadcaster.h>

#include <stdio.h>

class Merge_Lasers
{
	public:
		Merge_Lasers(ros::NodeHandle n);
		virtual ~Merge_Lasers();
	public:
		void laser1Callback (const sensor_msgs::LaserScanConstPtr& scan);
		void laser2Callback (const sensor_msgs::LaserScanConstPtr& scan);
		void merge();
		void spin();

	private:
		ros::NodeHandle n_;
		ros::Subscriber scan_hokuyo_sub_;
		ros::Subscriber scan_lms_sub_;
		ros::Publisher scan_pub_;
		ros::Publisher scan_hokuyo_simplif_pub_;

		ros::Time scan_time_;

		sensor_msgs::LaserScan hokuyo_data_;
		sensor_msgs::LaserScan lms_data_;
		sensor_msgs::LaserScan merge_data_;

		float lms_vector_[180];
		float hokuyo_vector_[180];
		float range_[360];


};

#endif
