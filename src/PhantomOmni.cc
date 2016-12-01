#include <endohap/PhantomOmni.h>
PhantomOmni::PhantomOmni(ros::NodeHandle n)
{
	force_pub = n.advertise<phantom_omni::OmniFeedback>("omni1_force_feedback",
			1);
	diag = n.advertise<geometry_msgs::Vector3>("diagnosis",1),
	joint_sub = n.subscribe("/omni1_joint_states", 1, &PhantomOmni::callback, this);

	x = 0, y = 0, z = 0;

}

void PhantomOmni::callback(sensor_msgs::JointState st)
{
	state = st;

	updateStates();
}

void PhantomOmni::setFeedback(double force)
{
	phantom_omni::OmniFeedback feedback;
	geometry_msgs::Vector3 frc, blk_pos, pos;
	
	double x_p , z_p;

	z_p = std::sqrt(1/((z*z)/(x*x) + 1));
	x_p = -(z/x)*z_p;

	frc.x = force*x_p, blk_pos.x = 0;
	frc.y = 0, blk_pos.y = 0;
	frc.z = force*z_p, blk_pos.z = 0;

	feedback.force = frc;
	feedback.position = pos;

	force_pub.publish(feedback);
	diag.publish(frc);
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

	x = transform_base_stylus.getOrigin().x();
	y = transform_base_stylus.getOrigin().y();
	z = transform_base_stylus.getOrigin().z();
}
;
