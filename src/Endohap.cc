#include <endohap/Endohap.h>

Endohap::Endohap(ros::NodeHandle n, ros::Rate rate) :
		omni(n), endowrist(n, rate)
{
	feedback.x = 0, feedback.y = 0, feedback.z = 0; r = 0; theta= 0;
}

void Endohap::calculateFeedback(geometry_msgs::Vector3 force, geometry_msgs::Vector3 pos)
{
	double x_r, y_r, x_y, y_y, m;

	r = std::sqrt((pos.y * pos.y)  + (pos.x * pos.x));
    theta = 2*atan(pos.y /(pos.x + r));

	feedback.x = force.x*sin(theta) + 2*force.y*cos(theta);
    feedback.y = force.x*cos(theta) - 2*force.y*sin(theta);
    feedback.z = 0*force.z;

	//printf("feedback: %f\t%f\t%f\t\n", feedback.x,feedback.y,feedback.z);

	saturation(&feedback.x,2.5);
	saturation(&feedback.y,2.5);
	saturation(&feedback.z,2.5);

	//printf("velocities: %f\t%f\t%f\t\n", omni.velocities.x,omni.velocities.y,omni.velocities.z);
}

void Endohap::loop()
{
	calculateFeedback(endowrist.force,omni.pos);
	std::vector<double> pos(5);

	for (int i = 0; i < 5; i++)
		pos.push_back(1);
	omni.state.position.resize(6);

	omni.setFeedback(feedback);
	pos[1] = 0.7096*theta-1.108;
	pos[2] = 3.93*omni.pos.z - 0.54;
	pos[3] = -5.384*r + 0.7;
	pos[0] = 5.384*r - 0.7;

	std::cout << pos[0] << " " << pos[1] << std::endl;
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



