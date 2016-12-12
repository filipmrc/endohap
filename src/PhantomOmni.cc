#include <endohap/PhantomOmni.h>
PhantomOmni::PhantomOmni(ros::NodeHandle n)
{
	force_pub = n.advertise<phantom_omni::OmniFeedback>("omni1_force_feedback",
			1);
	diag = n.advertise<geometry_msgs::Vector3>("diagnosis",1),
	joint_sub = n.subscribe("/omni1_joint_states", 1, &PhantomOmni::callback, this);

	pos.x = 0, pos.y = 0, pos.z = 0;

}

void PhantomOmni::callback(sensor_msgs::JointState st)
{
	state = st;

	updateStates();
}

void PhantomOmni::setFeedback(geometry_msgs::Vector3 fdbk)
{
	phantom_omni::OmniFeedback feedback;
	geometry_msgs::Vector3 frc, blk_pos;

	frc.x = fdbk.x, blk_pos.x = 0;
	frc.y = fdbk.y, blk_pos.y = 0;
	frc.z = fdbk.z, blk_pos.z = 0;

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
	geometry_msgs::Vector3 previous_position = pos;

	pos.x = transform_base_stylus.getOrigin().x();
	pos.y = transform_base_stylus.getOrigin().y();
	pos.z = transform_base_stylus.getOrigin().z();

	velocities.x = pos.x - previous_position.x;
	velocities.y = pos.y - previous_position.y;
	velocities.z = pos.z - previous_position.z;
}
;
