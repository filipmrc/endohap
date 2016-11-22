#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class Endohap
{
public:
	ros::Publisher force_pub;
	ros::Subscriber joint_sub;
	tf::TransformListener listener;

	Endohap(ros::NodeHandle n)
	{
		force_pub = n.advertise<phantom_omni::OmniFeedback>(
				"omni1_force_feedback", 1);
		joint_sub = n.subscribe("/joint_states", 1, &Endohap::endowristCallback,
				this);

		endo_pos.resize(4);
		endo_eff.resize(4);
		endo_vel.resize(4);
	}
	;

	// Get current endowrist and phantom omni joint states and calculate feedback
	void endowristCallback(sensor_msgs::JointState state)
	{
		endo_state = state;

		updateOmniStates();

		updateEndoStates();

		endowristForceEstimation();

		setFeedback(1, 1, 1);
	}

	// Update states for both the endowrist and phantom omni
	void updateOmniStates()
	{
		// Check tf stream for current phantom omni eef position
		try
		{
			listener.lookupTransform("/base", "/tip", ros::Time(0),
					transform_base_stylus);
		} catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	void updateEndoStates()
	{
		// Physical motor angles Endowrist used in the dynamic model
		endo_pos[0] = (-11 / 8) * endo_state.position[3],
		endo_pos[1] = (-13/14) * endo_state.position[4],
        endo_pos[2] = endo_state.position[2] - (9 / 14) * endo_state.position[4],
		endo_pos[3] = endo_pos[2];

		endo_eff[0] = endo_state.effort[0],
		endo_eff[1] = endo_state.effort[1],
		endo_eff[2] = endo_state.effort[2],
		endo_eff[3] = endo_state.effort[3];
	}

	// Estimate forces on Endowrist gripper
	void endowristForceEstimation()
	{
		double m = 0.0028, b = -0.8259;
		endo_force = m*endo_eff[0] + b;
	}

	// Set feedback for the phantom omni and send to phantom_omni package
	void setFeedback(double x, double y, double z)
	{
		phantom_omni::OmniFeedback feedback;
		geometry_msgs::Vector3 frc, pos;

		frc.x = x * 20;
		pos.x = 0;
		frc.y = -y * 10;
		pos.y = 0.1;
		frc.z = -z * 20;
		pos.z = 0;

		feedback.force = frc;
		feedback.position = pos;

		force_pub.publish(feedback);
	}

private:
	tf::StampedTransform transform_base_stylus;
	sensor_msgs::JointState endo_state;
	std::vector<double> endo_pos, endo_vel, endo_eff;
	double endo_force;

};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "omni_haptic_node");
	ros::NodeHandle n;
	ros::Rate r(100);

	Endohap hap(n);

	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

}
