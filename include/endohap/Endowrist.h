#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>

class Endowrist
{
public:
	double force;
	std::vector<double> pos, last_pos, vel, eff;

	Endowrist(ros::NodeHandle n, ros::Rate r);

	// Fresh data callback function
	void callback(sensor_msgs::JointState st);

	// Update Endowrist joint states
	void updateStates();

	// Estimate forces on Endowrist gripper
	void forceEstimation();

	//TODO Set joint states
	void setJoints(std::vector<double> cmd);

private:
	ros::Publisher joint_pub;
	ros::Subscriber joint_sub;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> acTraj;

	sensor_msgs::JointState state;

	double T;
};
