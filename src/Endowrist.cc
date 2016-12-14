#include <endohap/Endowrist.h>

Endowrist::Endowrist(ros::NodeHandle n, ros::Rate r) :
		acTraj("davinci/p4_hand_controller/follow_joint_trajectory", true)
{
	joint_sub = n.subscribe("/joint_states", 1, &Endowrist::callback, this);
	T = 0.02;

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
//	A_p.resize(4,4), B_p.resize(4,1), C_p.resize(2,4), x_p.resize(4,1), y_p.resize(2,1);
//    MatrixXd A_yaw(6,6), B_yaw(6,1), C_yaw(2,6), x_yaw(6,1), y_yaw(2,1), Q_yaw(1,1), R_yaw(2,2);

	// initialize pitch model
	int order = 6; int inputs = 1; int outputs = 2;
	A_p.resize(order,order), B_p.resize(order,inputs), C_p.resize(outputs,order), x_p.resize(order,1), y_p.resize(outputs,1);


	// initialize yaw model
	order = 6; inputs = 1; outputs = 2;
	A_yaw.resize(order,order), B_yaw.resize(order,inputs), C_yaw.resize(outputs,order), x_yaw.resize(order,1), y_yaw.resize(outputs,1), Q_yaw.resize(inputs,inputs), R_yaw.resize(outputs,outputs);
	std::cout << "A" << std::endl;
	A_yaw <<    0.9804,    0.0851,    0.1636,    0.1836,    0.0759,    0.1819,
			   -0.0119,    0.6809,   -0.2967,    0.4604,   -0.4634,    0.1466,
			   -0.0012,    0.2670,    0.9193,    0.0268,   -0.2378,   -0.1622,
			   -0.0020,   -0.0913,    0.0933,    0.5568,    0.1060,   -0.3030,
			    0.0008,    0.0198,   -0.0275,    0.5082,    0.7115,   -0.3731,
			   -0.0003,   -0.0059,    0.0127,   -0.1328,    0.3285,    0.4622;

	B_yaw <<    -0.2150,   -1.2980,   -1.5127,    2.0888,   -1.2082,    1.7883;

	C_yaw <<     0.0594,   -0.6962,    0.0325,    0.4309,   -0.4352,    0.0929,
				-0.3510,    0.0089,    0.2817,    0.3199,    0.2157,    0.5793;

	Q_yaw << 10*10;

	R_yaw << 10*10/T, 0 ,0, (unsigned int)100000000*100000000/T;

	x_yaw << 0, 0, 0, 0, 0, 0;


	f_yaw.initializeFilter(A_yaw,B_yaw,C_yaw,Q_yaw,R_yaw,x_yaw);

	// initialize pitch model
	A_p <<  0.2553,   -0.6622,    3.2317,  256.5243,   49.3080,  371.6285,
		    0.0485,    0.8377,   -0.2614,  -56.3401,   23.5937,  -93.4333,
		   -0.0782,   -0.0529,    1.7583,   56.6637,   34.0408,  -62.2696,
		    0.0010,    0.0011,   -0.0113,    0.2611,   -0.5256,    1.2069,
		   -0.0000,   -0.0004,   -0.0018,   -0.1151,    0.9082,   -0.2223,
		    0.0000,    0.0002,    0.0010,    0.0624,    0.0627,    0.6219;

	B_p <<  1000000*9.1454,   - 1000000*2.2241,     1000000*0.3202,     1000000*0.0012,    1000000*0.0012,     1000000*0.0010;

	C_p <<  0.0000,    0.0000,   -0.0000,   -0.0000,    0.0011,   -0.0019,
		    0.0000,    0.0000,    0.0001,    0.0082,   -0.0013,   -0.0034;

	x_p << 0, 0, 0, 0, 0, 0;
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

	pos[0] = CONV_POS*state.position[0], pos[1] = CONV_POS*state.position[1],
    pos[2] = CONV_POS*state.position[3] , pos[3] = CONV_POS*state.position[4];

	vel[0] = CONV_POS*state.velocity[0], vel[1] = CONV_POS*state.velocity[1],
	vel[2] = CONV_POS*state.velocity[3], vel[3] = CONV_POS*state.velocity[4];

	// Efforts //TODO Possibly conversion from current to amps
	eff[0] = state.effort[0], eff[1] = state.effort[1], eff[2] =
			state.effort[3], eff[3] = state.effort[4];

	// Endowrist motor angular velocities
//	for (int i = 0; i < 4; i++)
//	{
//		vel[i] = (pos[i] - last_pos[i]) / T;
//		if (!std::isfinite(vel[i])) vel[i] = 0.0;
//		last_pos[i] = pos[i];
//	}
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
	u << -eff[1];
	ye1 = f_yaw.estimateOutput(yv,u);
	yv << vel[3], 0;
	u << eff[3];
	ye4 = f_yaw.estimateOutput(yv,u);
	if ((double(ye1(1)) > 0) && (double(ye4(1) > 0)))
	{
		force.y = deadzone(double (ye4(1) + ye1(1))/2,0.4);
	}
	else if ((double(ye1(1)) < 0) && (double(ye4(1) < 0)))
	{
		force.y = deadzone(double (ye4(1) + ye1(1))/2,0.4);
	}else
		force.y = 0;




	// pitch
	x_p = A_p*x_p + B_p*eff[4];
	y_p = C_p*x_p;
	force.z = y_p(1);
    std::cout << force.z << std::endl;

	// roll
	double m = 1;
	if (vel[0] > 0)
	{
		force.x = m*eff[0];
	}
	else
		force.x = -m*eff[0];

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


