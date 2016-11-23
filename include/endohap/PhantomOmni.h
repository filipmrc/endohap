#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class PhantomOmni
{
public:
	ros::Publisher force_pub;
	tf::TransformListener listener;

	PhantomOmni(ros::NodeHandle n);

	// Set feedback for the phantom omni and send to phantom_omni package
	void setFeedback(double force);

private:

	// Update states for both the endowrist and phantom omni
	void updateStates();

	tf::StampedTransform transform_base_stylus;

};
