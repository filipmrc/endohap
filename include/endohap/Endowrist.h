#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <endohap/Kalman.h>
#include <endohap/utils.h>
#include <Eigen/Dense>
#include <endohap/deadzone_switch.h>
#define CONV_POS 1.953125
using Eigen::MatrixXd;

class Endowrist
{
public:
	geometry_msgs::Vector3 force;
	std::vector<double> pos, last_pos, vel, eff, setpoint, delta;

	Endowrist(ros::NodeHandle n, ros::Rate r);

	void initializeModels();

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
	MatrixXd A_p, B_p, C_p, x_p, y_p, Q_p, R_p;
	MatrixXd A_yaw, B_yaw, C_yaw, x_yaw, y_yaw, Q_yaw, R_yaw;
	Kalman f_yaw, f_pitch;

	Deadzone_switch roll_dz;
	Deadzone_switch clamp_dz;

	ros::Time last, current;

};
