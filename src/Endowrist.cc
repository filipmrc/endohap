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
	A_p.resize(4,4), B_p.resize(4,1), C_p.resize(2,4), x_p.resize(4,1), y_p.resize(2,1);
    MatrixXd A_yaw(6,6), B_yaw(6,1), C_yaw(2,6), x_yaw(6,1), y_yaw(2,1), Q_yaw(1,1), R_yaw(2,2);


	A_yaw <<    0.9804,    0.0851,    0.1636,    0.1836,    0.0759,    0.1819,
			   -0.0119,    0.6809,   -0.2967,    0.4604,   -0.4634,    0.1466,
			   -0.0012,    0.2670,    0.9193,    0.0268,   -0.2378,   -0.1622,
			   -0.0020,   -0.0913,    0.0933,    0.5568,    0.1060,   -0.3030,
			    0.0008,    0.0198,   -0.0275,    0.5082,    0.7115,   -0.3731,
			   -0.0003,   -0.0059,    0.0127,   -0.1328,    0.3285,    0.4622;

	B_yaw <<    -0.2150,   -1.2980,   -1.5127,    2.0888,   -1.2082,    1.7883;

	C_yaw <<     0.0594,   -0.6962,    0.0325,    0.4309,   -0.4352,    0.0929,
				-0.3510,    0.0089,    0.2817,    0.3199,    0.2157,    0.5793;

	Q_yaw << 0.000001*0.000001;

	R_yaw << 0.000001*0.000001/T, 0 ,0, (unsigned int)100000000*100000000/T;

	x_yaw << 0, 0, 0, 0, 0, 0;


	f_yaw.initializeFilter(A_yaw,B_yaw,C_yaw,Q_yaw,R_yaw,x_yaw);

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
//	pos[0] = (-11 / 8) * state.position[3], pos[1] = (-13 / 14)
//			* state.position[4], pos[2] = state.position[2]
//			- (9 / 14) * state.position[4], pos[3] = pos[2];

	double co = 1.953125;
	pos[0] = co*state.position[0], pos[1] = co*state.position[1], pos[2] = co*state.position[3] , pos[3] = co*state.position[4];

	// Endowrist motor angular velocities
	for (int i = 0; i < 4; i++)
	{
		vel[i] = (pos[i] - last_pos[i]) / T;
		if (!std::isfinite(vel[i])) vel[i] = 0.0;
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
	MatrixXd yv(2,1), u(1,1), ye1(2,1), ye2(2,1), ye3(2,1), ye4(2,1);

	// yaw
	yv << vel[1], 0;
	u << eff[1];
	ye1 = f_yaw.estimateOutput(yv,u);
	yv << vel[3], 0;
	u << eff[3];
	ye4 = f_yaw.estimateOutput(yv,u);
	force.y = ye4(1);



	// pitch
	x_p = A_p*x_p + B_p*eff[4];
	y_p = C_p*x_p;
	force.z = 0;//y_p(1);


	// roll
	double m = 1;
	force.x = m*eff[1];

	//printf("force.x = %f,\tforce.y = %f,\tforce.z = %f\n",force.x,force.y,force.z);

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
		goal.trajectory.points[0].time_from_start = ros::Duration(0.0001);
		goal.trajectory.points[0].positions[i] = cmd[i];
	}

	acTraj.sendGoal(goal);

	bool finished_before_timeout = acTraj.waitForResult(ros::Duration(10.0));

	if (finished_before_timeout)
	{
		//actionlib::SimpleClientGoalState state = acTraj.getState();
		//ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");
}


