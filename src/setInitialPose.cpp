#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

	geometry_msgs::PoseWithCovarianceStamped currentPosition;
	currentPosition.pose = msg->pose;

	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;

	ROS_INFO("Setting Initial Pose Of X: %f, Y: %f", x, y);

	ros::NodeHandle pnh;

	ros::Publisher location_pub_ipose = pnh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
	ros::Publisher location_pub_amclpose = pnh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10);
	
	ros::Rate loop_rate(10);

	while(ros::ok()){
		
		location_pub_ipose.publish(currentPosition);
		location_pub_amclpose.publish(currentPosition);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}

int main(int argc, char** argv){

	ros::init(argc, argv, "SetInitialPose");

	ros::NodeHandle snh;

	ros::Subscriber sub = snh.subscribe("odom", 10, OdomCallback);

	ros::spin();

	return 0;

}
