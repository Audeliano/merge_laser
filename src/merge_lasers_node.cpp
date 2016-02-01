#include "merge_lasers.h"

int main (int argc, char** argv)
{
	ros::init (argc, argv, "merge_lasers_node");

	ros::NodeHandle n;

	Merge_Lasers mergelasers(n);

	mergelasers.spin();

	return 0;

}
