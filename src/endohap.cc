#include <ros/ros.h>
#include <endohap/Endowrist.h>
#include <endohap/PhantomOmni.h>

int main(int argc, char** argv)
{

	ros::init(argc, argv, "omni_haptic_node");
	ros::NodeHandle n;
	ros::Rate r(100);

	PhantomOmni omni(n);
	Endowrist endowrist(n);

	while (ros::ok())
	{
		omni.setFeedback(endowrist.force);
		ros::spinOnce();
		r.sleep();
	}

}
