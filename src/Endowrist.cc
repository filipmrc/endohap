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
	force.x = 0; force.y = 0; force.z = 0;

	initializeModels();
}

void Endowrist::initializeModels()
{
	// initialize yaw model
	A_y.resize(6,6), B_y.resize(6,1), C_y.resize(2,6), x_y.resize(6,1), y_y.resize(2,1),
	A_p.resize(4,4), B_p.resize(4,1), C_p.resize(2,4), x_p.resize(4,1), y_p.resize(2,1);
//	MatrixXd A_y(6,6), B_y(6,1), C_y(2,6), A_p(4,4), B_p(4,1), C_p(2,4),
//			 x_y(6,1), x_p(4,1), y_y(2,1), y_p(2,1), Q_y(1,1), Q_p(1,1), R_y(2,2), R_p(2,2);

	A_y <<     0.5509,   -0.0109,    0.0007,   -0.0047,   -0.0010,    0.0016,
			   10.4518,    0.5990,   -0.1192,    0.1213,   -0.0200,   -0.0280,
			   10.0873,   -0.4123,    0.8314,    0.0783,    0.1449,    0.0884,
			  -16.8556,    0.6775,    0.3087,    0.6162,   -0.0598,    0.2676,
			    4.5207,   -0.1856,   -0.0992,    0.2189,    0.0859,   -0.5281,
			   -4.5210,    0.1822,    0.0715,   -0.2109,    0.1712,   -0.3489;

	B_y <<    -0.1910,   16.4717,   29.1913,    4.2352,  -39.4381, -169.9798;

	C_y <<     0.0900,   -0.0624,    0.0619,   -0.0352,   -0.0026,   -0.0016,
		       0.0009,    0.0008,   -0.0005,    0.0001,    0.0000,   -0.0000;

	Q_y << 0.0005*0.0005;

	R_y << 0.00001*0.00001, 0 ,0, (unsigned int)100000*100000;

	x_y << 0, 0, 0, 0, 0, 0;

	f1.initializeFilter(A_y,B_y,C_y,Q_y,R_y,x_y);

	// initialize pitch model
	A_p <<  0.9034,   -0.3638,    0.0805,    0.1389,
		    0.2981,    0.5413,    0.3809,   -0.3989,
		   -0.0503,   -0.4186,    0.3830,    0.1591,
		    0.0531,    0.1722,    0.4541,   -0.0490;

	B_p << -0.4081,    0.7281,   -1.0969,    2.2372;

	C_p <<  1.4682,    0.4904,    0.3858,    0.5332,
		   -0.0081,    0.0013,    0.0056,    0.0011;

	x_p << 0, 0, 0, 0;
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
			state.effort[3], eff[3] = state.effort[4];
}

void Endowrist::forceEstimation()
{
	//	omni	joint		motor		joint_states
	//
	//	x 	pitch		3		eff[4]
	//	y 	clamp		1 & 4		eff[1] & eff[3]
	//	z 	roll		2		eff[0]
	MatrixXd yv(2,1), u(1,1), ye(2,1);

	// yaw
	yv << vel[0], 0;
	u << eff[0];
	ye = f1.estimateOutput(yv,u);
	force.y = ye(2);


	//pitch
	x_p = A_p*x_p + B_p*eff[4];
	y_p = C_p*x_p;
	force.x = y_p(1);


	// roll
	double m = 5;
	force.z = m*eff[0];



	printf("force.x = %f,\tforce.y = %f,\tforce.z = %f\n",force.x,force.y,force.z);

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
		goal.trajectory.points[0].time_from_start = ros::Duration(0.01);
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


