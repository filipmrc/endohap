#include <endohap/Endowrist.h>

Endowrist::Endowrist(ros::NodeHandle n, ros::Rate r) :
		acTraj("davinci/p4_hand_controller/follow_joint_trajectory", true)
{
	joint_sub = n.subscribe("/joint_states", 1, &Endowrist::callback, this);
	T = r.cycleTime().sec;

	pos.resize(4);
	last_pos.resize(4);
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

	// Endowrist motor angular velocities
	for (int i = 0; i < 4; i++)
	{
		vel[i] = (pos[i] - last_pos[i]) / T;
		last_pos[i] = pos[i];
	}

	// Efforts //TODO Possibly conversion from current to amps
	eff[0] = state.effort[0], eff[1] = state.effort[1], eff[2] =
			state.effort[2], eff[3] = state.effort[3];
}

void Endowrist::forceEstimation()
{
	double m = 0.0028, b = -0.8259;
	force = m * eff[2] + b;
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
		goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
		goal.trajectory.points[0].positions[i] = cmd[i];
	}

	acTraj.sendGoal(goal);

	bool finished_before_timeout = acTraj.waitForResult(ros::Duration(10.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = acTraj.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");
}

