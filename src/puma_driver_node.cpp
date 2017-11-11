/*
 *  Copyright (C) 2017 Vision-Guided and Intelligent Robotics Lab
 *  Written by Ali Shafiekhani <Ashafiekhani@mail.missouri.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *          keep this copyright notice,
 *          do  not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *          yada yada yada...
 *
 *  You can get a copy of the GNU General Public License by writing to
 *  the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <leap_motion/leapros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <angles/angles.h>
 
#include "lab5/Puma_OP.h"

class Puma260{
private:
	tf::TransformBroadcaster br_;
	tf::TransformListener ls_;

	tf::Transform rb_to_origin_;
	tf::Transform lm_to_origin_;
	tf::Transform hand_to_lm_;
	tf::Transform link1_to_rb_;
	tf::Transform link2_to_link1_;
	tf::Transform link3_to_link2_;
	tf::Transform link4_to_link3_;
	tf::Transform link5_to_link4_;
	tf::Transform gripper_to_link5_;

	ros::Subscriber lm_sub_;

	geometry_msgs::Vector3 ypr_;
	geometry_msgs::Vector3 normal_;
	geometry_msgs::Vector3 direction_;
	geometry_msgs::Point xyz_;
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

public:
	Puma260(ros::NodeHandle&);
	~Puma260();
	void leapMotionCallback(const leap_motion::leapros::ConstPtr& msg);
	void updateTF();

};

Puma260::Puma260(ros::NodeHandle& nh)
: nh_(nh),
priv_nh_("~")
{
	lm_sub_ = nh_.subscribe("/leapmotion/data", 1, &Puma260::leapMotionCallback, this);

}
Puma260::~Puma260(){}

void Puma260::leapMotionCallback(const leap_motion::leapros::ConstPtr& msg)
{
	ypr_ = msg->ypr;
	normal_ = msg->normal;
	direction_ = msg->direction;
	xyz_ = msg->palmpos;
	//ROS_INFO_STREAM("YPR = " << ypr_.x << ", " << ypr_.y << ", " << ypr_.z);
	//ROS_INFO_STREAM("XYZ = " << xyz_.x << ", " << xyz_.y << ", " << xyz_.z);
} 

void Puma260::updateTF()
{
	ros::Time NOW = ros::Time::now();

	// Robot Base wrt Origin
	rb_to_origin_.setOrigin(tf::Vector3(0, 0, 0.185));
	tf::Quaternion q1;
	q1.setRPY(0, 0, 0);
	rb_to_origin_.setRotation(q1);
	
	// Leap Motion wrt Robot Base
	lm_to_origin_.setOrigin(tf::Vector3(-0.215, 0.305, -0.200));
	tf::Quaternion q2;
	q2.setRPY(M_PI/2 , 0, 0);
	lm_to_origin_.setRotation(q2);
	
	// Hand wrt Leap Motion
	hand_to_lm_.setOrigin(tf::Vector3(xyz_.x / 1000, xyz_.y / 1000, xyz_.z / 1000));
	tf::Quaternion q3;
	q3.setRPY(angles::normalize_angle_positive(ypr_.z * M_PI / 180), angles::normalize_angle_positive(ypr_.y * M_PI / 180), angles::normalize_angle_positive(ypr_.x * M_PI / 180));
	
	hand_to_lm_.setRotation(q3);

	br_.sendTransform(tf::StampedTransform(rb_to_origin_, NOW, "map", "rb"));
	br_.sendTransform(tf::StampedTransform(lm_to_origin_, NOW, "map", "lm"));
	br_.sendTransform(tf::StampedTransform(hand_to_lm_, NOW, "lm", "hand"));

	tf::Transform hand_wrt_rb =  rb_to_origin_.inverse() * lm_to_origin_ * hand_to_lm_;
	float X, Y, Z;
	X = hand_wrt_rb.getOrigin().getX() * 1000;
	Y = hand_wrt_rb.getOrigin().getY() * 1000;
	Z = hand_wrt_rb.getOrigin().getZ() * 1000;

	ROS_INFO_STREAM("XYZ = " << X << ", " << Y << ", " << Z);
	
	if(X>-380 && X < -170 && Y>180 && Y<350 && Z>-195)
		MOVETO_XYZOAT_WO_CHECK(X, Y, Z, -90, 90, 0);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "tf_broadcaster_node");
	ros::NodeHandle nh;
	ros::Duration(1).sleep();

	Puma260 myRobot(nh);
	ros::Rate rate(10);
	setSpeed(10);
	while(ros::ok()){
		ros::spinOnce();
		myRobot.updateTF();
		rate.sleep();

	}

	return 0;
}