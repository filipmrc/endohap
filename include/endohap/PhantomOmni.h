#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class PhantomOmni
{
public:
	sensor_msgs::JointState state;
	geometry_msgs::Vector3 pos;
	geometry_msgs::Vector3 velocities;

	PhantomOmni(ros::NodeHandle n);

	// Set feedback for the phantom omni and send to phantom_omni package
	void setFeedback(geometry_msgs::Vector3 force);

	void callback(sensor_msgs::JointState st);

private:

	ros::Publisher force_pub, diag;
	ros::Subscriber joint_sub;
	tf::TransformListener listener;
	tf::StampedTransform transform_base_stylus;

	// Update states for both the endowrist and phantom omni
	void updateStates();
};
