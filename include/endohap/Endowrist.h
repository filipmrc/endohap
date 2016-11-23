#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>

class Endowrist
{
public:
	double force;

	Endowrist(ros::NodeHandle n);

	void callback(sensor_msgs::JointState st);

	void updateStates();

	// Estimate forces on Endowrist gripper
	void forceEstimation();

private:
	sensor_msgs::JointState state;
	std::vector<double> pos, vel, eff;
	ros::Subscriber joint_sub;
};
