#include <endohap/Endowrist.h>

Endowrist::Endowrist(ros::NodeHandle n)
{
	joint_sub = n.subscribe("/joint_states", 1, &Endowrist::callback, this);

	joint_pub = n.advertise<trajectory_msgs::JointTrajectory>(
			"p4_hand_controller/follow_joint_trajectory/command", 1);

	pos.resize(4);
	eff.resize(4);
	vel.resize(4);
	force = 0;
}

void Endowrist::callback(sensor_msgs::JointState st)
{
	state = st;

	updateStates();

	forceEstimation();
}

void Endowrist::updateStates()
{
	// Physical motor angles Endowrist used in the dynamic model
	pos[0] = (-11 / 8) * state.position[3], pos[1] = (-13 / 14)
			* state.position[4], pos[2] = state.position[2]
			- (9 / 14) * state.position[4], pos[3] = pos[2];

	// Efforts //TODO Possibly conversion from current to amps
	eff[0] = state.effort[0], eff[1] = state.effort[1], eff[2] =
			state.effort[2], eff[3] = state.effort[3];
}

void Endowrist::forceEstimation()
{
	double m = 0.0028, b = -0.8259;
	force = m * eff[0] + b;
}

void Endowrist::setJoints()
{
	//TODO set this shit up
	trajectory_msgs::JointTrajectory command;
}

