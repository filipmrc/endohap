#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class Endowrist
{
public:
	double force;

	Endowrist(ros::NodeHandle n)
	{
		joint_sub = n.subscribe("/joint_states", 1, &Endowrist::callback,
				this);

		pos.resize(4);
		eff.resize(4);
		vel.resize(4);
		force = 0;
	}
	void callback(sensor_msgs::JointState st)
	{
		state = st;

		updateStates();

		forceEstimation();
	}

	void updateStates()
	{
		// Physical motor angles Endowrist used in the dynamic model
		pos[0] = (-11 / 8) * state.position[3],
		pos[1] = (-13/14) * state.position[4],
        pos[2] = state.position[2] - (9 / 14) * state.position[4],
		pos[3] = pos[2];

		eff[0] = state.effort[0],
		eff[1] = state.effort[1],
		eff[2] = state.effort[2],
		eff[3] = state.effort[3];
	}

	// Estimate forces on Endowrist gripper
	void forceEstimation()
	{
		double m = 0.0028, b = -0.8259;
		force = m*eff[0] + b;
	}

private:
	sensor_msgs::JointState state;
	std::vector<double> pos, vel, eff;
	ros::Subscriber joint_sub;
};


class PhantomOmni
{
public:
	ros::Publisher force_pub;
	tf::TransformListener listener;

	PhantomOmni(ros::NodeHandle n)
	{
		force_pub = n.advertise<phantom_omni::OmniFeedback>(
				"omni1_force_feedback", 1);
	}

	// Set feedback for the phantom omni and send to phantom_omni package
	void setFeedback(double force)
	{
		updateStates();

		phantom_omni::OmniFeedback feedback;
		geometry_msgs::Vector3 frc, pos;

		frc.x = 0, pos.x = 0;
		frc.y = 0, pos.y = 0;
		frc.z = force, pos.z = 0;

		feedback.force = frc;
		feedback.position = pos;

		force_pub.publish(feedback);
	}

private:

	// Update states for both the endowrist and phantom omni
	void updateStates()
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

	tf::StampedTransform transform_base_stylus;

};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "omni_haptic_node");
	ros::NodeHandle n;
	ros::Rate r(100);

	PhantomOmni omni(n);
	Endowrist endowrist(n);

	while (ros::ok())
	{
		omni.setFeedback(endowrist.force);
		ros::spinOnce();
		r.sleep();
	}

}
