#include <endohap/Endohap.h>

Endohap::Endohap(ros::NodeHandle n, ros::Rate r) :
		omni(n), endowrist(n, r)
{
	feedback.x = 0, feedback.y = 0, feedback.z = 0;
}

void Endohap::calculateFeedback(geometry_msgs::Vector3 force, geometry_msgs::Vector3 pos)
{
	double x_p, z_p;

	double previous_x = feedback.x;

	z_p = std::sqrt(1 / ((pos.z * pos.z) / (pos.x * pos.x) + 1));
	x_p = -(pos.z / pos.x) * z_p;

	feedback.x = force.x*x_p, feedback.y = force.z, feedback.z = force.x*z_p + force.y;
	
	//feedback.x = previous_x + (feedback.x - previous_x);


	saturation(&feedback.x,2.0);
	saturation(&feedback.y,2.0);
	saturation(&feedback.z,2.0);

	printf("%f\t%f\t%f\t\n", feedback.x,feedback.y,feedback.z);
}

void Endohap::loop()
{
	calculateFeedback(endowrist.force,omni.pos);
	std::vector<double> pos(5);

	for (int i = 0; i < 5; i++)
		pos.push_back(1);
	omni.state.position.resize(6);

	omni.setFeedback(feedback);
	pos[2] = omni.state.position[0];
	pos[0] = -omni.pos.z;
	pos[3] = omni.pos.z;
	pos[1] = omni.pos.y;
	endowrist.setJoints(pos);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "omni_haptic_node");
	ros::NodeHandle n;
	ros::Rate r(10000);

	Endohap endohap(n, r);

	while (ros::ok())
	{
		endohap.loop();
		ros::spinOnce();
		r.sleep();
	}

}

void Endohap::saturation(double* force, double bound)
{
	if (*force > bound) *force = bound;
	if (*force < -bound) *force = -bound;
	if (*force != *force) *force = 0.0;
}

void Endohap::deadzone(double* value, double bound)
{
	if (abs(*value) < bound)
		*value = 0.0;
	else
		*value = *value + (signum(value)*bound); 
}

int Endohap::signum(double* d)
{
	return (*d > 0) - (*d < 0);
}

