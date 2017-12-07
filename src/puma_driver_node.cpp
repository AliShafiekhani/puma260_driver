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
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <angles/angles.h>
 
#include "puma260_driver/Puma_OP.h"
#include "puma260_driver/XYZOAT.h"
#include "puma260_driver/JointAngles.h"

class Puma260{
private:
	ros::Publisher pos_pub_;
	ros::Publisher joints_pub_;
	tf::TransformBroadcaster br_;

	puma260_driver::XYZOAT end_effector_pose_;
	puma260_driver::JointAngles joint_angles_;

	ros::ServiceServer ready_srv_;
	ros::ServiceServer nest_srv_;

protected:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

public:
	Puma260(ros::NodeHandle&);
	~Puma260();
	void readPoseAngles();
	bool pumaReady(puma260_driver::ReadyPuma::Request &req, puma260_driver::ReadyPuma::Response &res);
	bool pumaNest(puma260_driver::NestPuma::Request &req, puma260_driver::NestPuma::Response &res);
};

Puma260::Puma260(ros::NodeHandle& nh)
: nh_(nh),
priv_nh_("~")

{
	pos_pub_ = nh_.advertise<puma260_driver::XYZOAT>("/puma260/xyz_oat",1);
	joints_pub_ = nh_.advertise<puma260_driver::JointAngles>("/puma260/joint_angles",1);
	ready_srv_ = nh_.advertiseService("/puma260/puma_ready",&Puma260::pumaReady, this);
	nest_srv_ = nh_.advertiseService("/puma260/puma_nest", &Puma260::pumaNest, this);

}

Puma260::~Puma260(){}

void Puma260::readPoseAngles(){
	POSITION pos;
	JOINTS jots;

	if(READ_POSITION(&pos, &jots))
		ROS_INFO_STREAM("Not able to read pose from puma");
	else{
		end_effector_pose_.x = pos.X;
		end_effector_pose_.y = pos.Y;
		end_effector_pose_.z = pos.Z;
		end_effector_pose_.O = pos.O;
		end_effector_pose_.A = pos.A;
		end_effector_pose_.T = pos.T;
		pos_pub_.publish(end_effector_pose_);

		joint_angles_.j1 = jots.j1;
		joint_angles_.j2 = jots.j2;
		joint_angles_.j3 = jots.j3;
		joint_angles_.j4 = jots.j4;
		joint_angles_.j5 = jots.j5;
		joint_angles_.j6 = jots.j6;
		joints_pub_.publish(joint_angles_);
	}
}

bool Puma260::pumaReady(puma260_driver::ReadyPuma::Request &req, puma260_driver::ReadyPuma::Response &res){
	if (doReady() == 1)
	{
		res.readyarm_result = "PUMA ARM HAS BEEN RETURNED TO READY POS";
		return true;
	}
	else{
		res.readyarm_result = "PUMA ARM UNABLE TO RETURN TO READY POS";
		return false;
	}
}

bool Puma260::pumaNest(puma260_driver::NestPuma::Request &req, puma260_driver::NestPuma::Response &res){
	if (doReady() == -1){
		ROS_INFO_STREAM("PUMA ARM UNABLE TO RETURN TO READY POS");
		res.nestarm_result = "PUMA UNABLE TO RETURN TO NEST, MOVE ARM TO READY FIRST";
		return false;
	}
	else{
		if(doNest() == 1){
			res.nestarm_result = "PUMA ARM HAS BEEN RETURNED TO NEST POS";
			return true;
		}
		else{
			res.nestarm_result = "PUMA UNABLE TO RETURN TO NEST, MOVE ARM TO READY FIRST";
			return false;
		}
	}
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "tf_broadcaster_node");
	ros::NodeHandle nh;
	ros::Duration(1).sleep();

	Puma260 myRobot(nh);
	ros::Rate rate(10);
	while(ros::ok()){
		ros::spinOnce();
		myRobot.readPoseAngles();
		rate.sleep();

	}

	return 0;
}
