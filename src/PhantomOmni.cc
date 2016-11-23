#include <endohap/PhantomOmni.h>
PhantomOmni::PhantomOmni(ros::NodeHandle n)
{
	force_pub = n.advertise<phantom_omni::OmniFeedback>("omni1_force_feedback",
			1);
}

void PhantomOmni::setFeedback(double force)
{
	updateStates();

	phantom_omni::OmniFeedback feedback;
	geometry_msgs::Vector3 frc, pos;

	frc.x = 0, pos.x = 0;
	frc.y = 0, pos.y = 0;
	frc.z = force, pos.z = 0;

	feedback.force = frc;
	feedback.position = pos;

	force_pub.publish(feedback);
}

void PhantomOmni::updateStates()
{
	// Check tf stream for current phantom omni eef position
	try
	{
		listener.lookupTransform("/base", "/tip", ros::Time(0),
				transform_base_stylus);
	} catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::StampedTransform transform_base_stylus;

}
;
