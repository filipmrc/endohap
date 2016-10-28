#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <phantom_omni/OmniFeedback.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class Endohap {
public:
	ros::Publisher force_pub;
	//ros::Subscriber joint_sub;

	Endohap(ros::NodeHandle n)
	{
	  force_pub = n.advertise<phantom_omni::OmniFeedback>("omni1_force_feedback",1);
	};

	void endowristCallback()
	{

	}

	void setForces()
	{
	  phantom_omni::OmniFeedback feedback;
	  geometry_msgs::Vector3 frc, pos;

	  frc.x = 0; pos.x = 0;
	  frc.y = 0; pos.y = 0;
	  frc.z = 0; pos.z = 0;

	  feedback.force = frc;
	  feedback.position = pos;

	  force_pub.publish(feedback);
	}

};
int main(int argc, char** argv) {

  ros::init(argc, argv, "omni_haptic_node");
  ros::NodeHandle n;
  tf::TransformListener listener;
  tf::StampedTransform transform_base_stylus;
  geometry_msgs::Transform t;

  Endohap hap(n);

  while(ros::ok()){
      hap.setForces();

      try{
        listener.lookupTransform("/lower_arm", "/stylus",
                                 ros::Time(0), transform_base_stylus);
      }
      catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      tf::transformTFToMsg(transform_base_stylus,t);
      ROS_INFO_STREAM(t);
  }

}
