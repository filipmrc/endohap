#include <endohap/Endowrist.h>

Endowrist::Endowrist(ros::NodeHandle n) :
		acTraj("davinci/p4_hand_controller/follow_joint_trajectory", true)
{
	joint_sub = n.subscribe("/joint_states", 1, &Endowrist::callback, this);

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

void Endowrist::setJoints(std::vector<double> cmd)
{
	cmd.resize(5);

	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.points.resize(1);
	goal.trajectory.points[0].positions.resize(5);



	char* args[5] =
	{ "p4_hand_roll", "p4_hand_pitch", "p4_instrument_slide",
			"p4_instrument_roll", "p4_instrument_pitch" };


	for (int i = 0; i < 5; i++)
	{
		goal.trajectory.joint_names.push_back(args[i]);
		goal.trajectory.points[0].positions[i] = cmd[i];
	}

	acTraj.sendGoal(goal);
}

