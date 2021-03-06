#include <endohap/Endowrist.h>
#include <endohap/PhantomOmni.h>
#include <endohap/utils.h>

#include <iostream>
#include <fstream>

class Endohap
{
public:
	Endohap(ros::NodeHandle n, ros::Rate rate);

	void calculateFeedback(geometry_msgs::Vector3 force, geometry_msgs::Vector3 pos);

	void loop();

private:
	PhantomOmni omni;
	Endowrist endowrist;
	geometry_msgs::Vector3 feedback;
	double r, theta;
	std::ofstream _data_log;
};
