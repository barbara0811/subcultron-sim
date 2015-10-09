/*
 * mzos_test.cpp
 *
 *  Created on: Jun 30, 2014
 *      Author: filip
 */


#include <labust_mission/labustMission.hpp>
#include <geometry_msgs/PoseStamped.h>

class QuadMeasSim{

public:
	QuadMeasSim():relDistX(0.0), relDistY(0.0), targetPosX(10.0), targetPosY(10.0), FOV(5.0){

		ros::NodeHandle nh;
		subMeasIdeal = nh.subscribe<auv_msgs::NavSts>("meas_ideal",1,&QuadMeasSim::onStateHat, this);
		pubRelativeDistance = nh.advertise<geometry_msgs::PoseStamped>("target_pose",1);

	}

	void onStateHat(const auv_msgs::NavSts::ConstPtr& data){
		if(abs(data->position.north-targetPosX)<FOV && abs(data->position.east-targetPosY)<FOV){

			geometry_msgs::PoseStamped relDist;
			relDist.pose.position.x = -(data->position.north-targetPosX);
			relDist.pose.position.y = -(data->position.east-targetPosY);
			pubRelativeDistance.publish(relDist);
		}
	}

	double relDistX, relDistY;
	double targetPosX, targetPosY;

	double FOV;

	ros::Subscriber subMeasIdeal;
	ros::Publisher pubRelativeDistance;

	//%subRelativeDistance = nh.subscribe<geometry_msgs::PoseStamped>("target_pose", 1, &MZOS::onRelativeDistance, this);

};

int main(int argc, char **argv){

	ros::init(argc,argv,"quad_meas");
	ros::NodeHandle nh, ph("~");

	QuadMeasSim qms;
	ros::spin();
	return 0;
}




