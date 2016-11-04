#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <phantom_omni/OmniFeedback.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class Endohap {
public:
	ros::Publisher force_pub;
	ros::Subscriber joint_sub;
	tf::TransformListener listener;

	Endohap(ros::NodeHandle n)
	{
	  force_pub = n.advertise<phantom_omni::OmniFeedback>("omni1_force_feedback",1);
	  joint_sub = n.subscribe("/joint_states", 1 , &Endohap::endowristCallback, this);
	};

	// Get current endowrist joint states and calculate feedback
	void endowristCallback(sensor_msgs::JointState state)
	{
	  updateStates();
	  this->setFeedback(1,1,1);
	}

	// Set feedback for the phantom omni and send to phantom_omni package
	void setFeedback(double x, double y, double z)
	{
	  phantom_omni::OmniFeedback feedback;
	  geometry_msgs::Vector3 frc, pos;

	  frc.x = x*20; pos.x = 0;
	  frc.y = -y*10; pos.y = 0.1;
	  frc.z = -z*20; pos.z = 0;

	  feedback.force = frc;
	  feedback.position = pos;

	  force_pub.publish(feedback);
	}

	// Update states for both the endowrist and phantom omni
	void updateStates()
	{
	      // Check tf stream for current phantom omni eef position
	      try{
	        listener.lookupTransform("/base", "/tip",
	                                 ros::Time(0), transform_base_stylus);
	      }
	      catch (tf::TransformException &ex){
	        ROS_ERROR("%s",ex.what());
	        ros::Duration(1.0).sleep();
	      }
	}

private:
	  tf::StampedTransform transform_base_stylus;

};
int main(int argc, char** argv) {

  ros::init(argc, argv, "omni_haptic_node");
  ros::NodeHandle n;
  ros::Rate r(100);

  Endohap hap(n);

  while(ros::ok()){
      ros::spinOnce();
      r.sleep();
  }

}
