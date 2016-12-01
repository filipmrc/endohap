#include <ros/ros.h>
#include <endohap/Endowrist.h>
#include <endohap/PhantomOmni.h>

int main(int argc, char** argv)
{

	ros::init(argc, argv, "omni_haptic_node");
	ros::NodeHandle n;
	ros::Rate r(600);

	PhantomOmni omni(n);
	Endowrist endowrist(n,r);

	std::vector<double> pos (5);

	for(int i= 0;i<5;i++)
		pos.push_back(1);
	omni.state.position.resize(6);

	while (ros::ok())
	{
		omni.setFeedback(endowrist.force);
		pos[2] = omni.state.position[0];
		pos[0] = -omni.z;
		pos[3] = omni.z;
		pos[1] = omni.y;
		endowrist.setJoints(pos);
		ros::spinOnce();
		r.sleep();
	}

}
