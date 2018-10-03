#define PCL_NO_PRECOMPILE

#include <pcl/features/normal_3d.h>

#include <include/DelphiSeg.h>

#include <std_msgs/String.h>


ros::Publisher groundPub;
ros::Publisher nonGroundPub;


void callBack(std_msgs::String fp)
{
	DelphiSeg cloud;

	cloud.ReadKitti(fp.data.c_str());

  	clock_t start = clock(), end;
	cloud.groundRemoval(3, 3, 20, 0.4, 0.2);
	end = clock();
	ROS_INFO("Ground removal time: %f", ((float) (end - start)) / CLOCKS_PER_SEC);

	sensor_msgs::PointCloud2 ground_msg, nonGround_msg;
	pcl::toROSMsg(*cloud.groundPoints, ground_msg);
	pcl::toROSMsg(*cloud.nonGroundPoints, nonGround_msg);
	ground_msg.header.frame_id = "velodyne";
	groundPub.publish(ground_msg);
	nonGround_msg.header.frame_id = "velodyne";
	nonGroundPub.publish(nonGround_msg);

	ros::spinOnce();
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test");
	ros::NodeHandle nh_;

	groundPub = nh_.advertise<sensor_msgs::PointCloud2>("/groundPts", 2, true);
	nonGroundPub = nh_.advertise<sensor_msgs::PointCloud2>("/nonGroundPts", 2, true);
  	ros::Subscriber sub = nh_.subscribe("/pcd_file", 2, callBack);

  	ros::spin();

	return 0;
}