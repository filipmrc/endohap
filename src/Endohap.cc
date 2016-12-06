#include <endohap/Endohap.h>

Endohap::Endohap(ros::NodeHandle n, ros::Rate r) :
		omni(n), endowrist(n, r)
{
	feedback.x = 0, feedback.y = 0, feedback.z = 0;
}

void Endohap::calculateFeedback(geometry_msgs::Vector3 force, geometry_msgs::Vector3 pos)
{
	double x_p, z_p;

	z_p = std::sqrt(1 / ((pos.z * pos.z) / (pos.x * pos.x) + 1));
	x_p = -(pos.z / pos.x) * z_p;

	feedback.x = force.x*x_p, feedback.y = 0, feedback.z = force.x*z_p;
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