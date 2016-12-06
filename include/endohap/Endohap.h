#include <endohap/Endowrist.h>
#include <endohap/PhantomOmni.h>

class Endohap
{
public:
	Endohap(ros::NodeHandle n, ros::Rate r);

	void calculateFeedback(double force, geometry_msgs::Vector3 pos);

	void loop();

private:
	PhantomOmni omni;
	Endowrist endowrist;
	geometry_msgs::Vector3 feedback;
};
