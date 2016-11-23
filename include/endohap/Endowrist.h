#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>

class Endowrist
{
public:
	double force;

	Endowrist(ros::NodeHandle n);

	// Fresh data callback function
	void callback(sensor_msgs::JointState st);

	// Update Endowrist joint states
	void updateStates();

	// Estimate forces on Endowrist gripper
	void forceEstimation();

	//TODO Set joint states
	void setJoints();

private:
	ros::Publisher joint_pub;

	sensor_msgs::JointState state;
	std::vector<double> pos, vel, eff;
	ros::Subscriber joint_sub;
};
