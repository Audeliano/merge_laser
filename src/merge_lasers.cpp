#include "merge_lasers.h"

Merge_Lasers::Merge_Lasers(ros::NodeHandle n)
{
	n_ = n;

	scan_hokuyo_sub_ = n.subscribe("scan_hokuyo", 10, &Merge_Lasers::laser1Callback, this);
	scan_lms_sub_ = n.subscribe("scan_lms", 10, &Merge_Lasers::laser2Callback, this);

	scan_pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 100, true);
	scan_hokuyo_simplif_pub_ = n.advertise<sensor_msgs::LaserScan>("scan_hokuyo_simplif", 100, true);

}

Merge_Lasers::~Merge_Lasers()
{
	scan_hokuyo_sub_.shutdown();
	scan_lms_sub_.shutdown();
	scan_pub_.shutdown();
	scan_hokuyo_simplif_pub_.shutdown();
}

void Merge_Lasers::laser1Callback(const sensor_msgs::LaserScanConstPtr& scan)
{
	int fator3 = 0;
	int cont = 0;
	int complem = 0;

	for (int laser_index = 0 ; laser_index < scan->ranges.size() ; laser_index++ )
	{
		fator3++;
		if(fator3 == 3)
		{
			//hokuyo_vector_[fim - cont - 1] = scan->ranges[laser_index];
			hokuyo_vector_[cont] = scan->ranges[laser_index];
			if(complem == 16)
			{
				cont++;
				complem = 0;
				hokuyo_vector_[cont] = scan->ranges[laser_index];
			}
			fator3 = 0;
			cont++;
			complem++;
		}
	}
	//std::cout<<"HOKUYO: scan_size: "<<scan->ranges.size()<<" | cont: "<<cont<<std::endl;
}

void Merge_Lasers::laser2Callback(const sensor_msgs::LaserScanConstPtr& scan)
{
/*	for (int laser_index = 0 ; laser_index < scan->ranges.size() - 1 ; laser_index++ )
	{
		lms_data_.ranges[scan->ranges.size() - laser_index - 1] = scan->ranges[laser_index];
	}
*/
	//scan_pub_.publish(lms_data_);
	int cont = 0;

	for (int laser_index = 0 ; laser_index < 180 ; laser_index++ )
	{
		//lms_vector_[180 - laser_index -1] = scan->ranges[laser_index];
		lms_vector_[laser_index] = scan->ranges[laser_index];
		cont++;
	}
	//std::cout<<"scan_size: "<<scan->ranges.size()<<std::endl;
	//std::cout<<"LMS: cont: "<<cont<<std::endl;
}

void Merge_Lasers::merge()
{
	int index_hokuyo = 180;
	int index_lms = 180;
	int cont = 0;

	scan_time_ = ros::Time::now();

	merge_data_.header.stamp = scan_time_;
	merge_data_.header.frame_id = "laser"; //scan->header.frame_id;

	merge_data_.angle_min = -1.565; //-3.13; //scan->angle_min;
	merge_data_.angle_max = 4.695; //scan->angle_max;
	merge_data_.angle_increment = 0.01745; //scan->angle_increment;

	merge_data_.time_increment = 3*0.00001; //scan->time_increment;
	merge_data_.scan_time = 0.1; //scan->scan_time;

	merge_data_.range_min = 0.01999; //scan->range_min;
	merge_data_.range_max = 5.59999; //scan->range_max;
	merge_data_.ranges.resize(index_hokuyo + index_lms);

	for(int i = 0 ; i < index_hokuyo ; i++)
	{
		merge_data_.ranges[i] = hokuyo_vector_[i];
		cont++;
	}

	for(int j = 0 ; j < index_lms ; j++)
	{
		merge_data_.ranges[index_hokuyo + j] = lms_vector_[j];
		cont++;
	}
	scan_pub_.publish(merge_data_);
	//std::cout<<"scan_size: "<<merge_data_.ranges.size()<<" | cont: "<<cont<<std::endl;

}

void Merge_Lasers::spin()
{
	ros::Rate loopRate(30);
	while(n_.ok())
	{
		ros::spinOnce();
		loopRate.sleep();

		merge();
	}
}

