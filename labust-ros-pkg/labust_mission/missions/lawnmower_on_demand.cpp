/*
 * lawnmower_on_demand.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Filip Mandic
 */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the LABUST nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//#ifndef EVENTEVALUATION_HPP_
//#define EVENTEVALUATION_HPP_
//
#include <labust_mission/labustMission.hpp>
#include <sensor_msgs/Joy.h>
////#include <exprtk/exprtk.hpp>
//
namespace labust {
	namespace data {

		class DataManager{

		public:

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			DataManager();

			void updateData(const auv_msgs::NavSts::ConstPtr& data);

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

			//ros::Subscriber subExternalEvents;
			//ros::Subscriber subStateHatAbs;

			/* State Hat variables */
			enum {u=0, v, w, r, x, y, z, psi, x_var, y_var, z_var, psi_var, alt, stateHatNum};
			std::vector<double> stateHatVar;

			/* Events data variables */
			std::vector<double> eventsVar;

			/* Mission specific variables */ // Svki specificni projekt ima klasu koja extenda DataManager klasu
			std::vector<double> missionVar;
		};

		DataManager::DataManager(){

			ros::NodeHandle nh;
			//subExternalEvents= nh.subscribe<misc_msgs::ExternalEvent>("externalEvent",1, &EventEvaluation::onReceiveExternalEvent, this);
			//subStateHatAbs= nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1, &DataManager::onStateHat, this);


			//externalEventContainer.resize(5); // 5 eksternih evenata
			stateHatVar.resize(stateHatNum);
		}

		void DataManager::updateData(const auv_msgs::NavSts::ConstPtr& data){

			stateHatVar[u] = data->body_velocity.x;
			stateHatVar[v] = data->body_velocity.y;
			stateHatVar[w] = data->body_velocity.z;
			stateHatVar[r] = data->orientation_rate.yaw;

			stateHatVar[x] = data->position.north;
			stateHatVar[y] = data->position.east;
			stateHatVar[z] = data->position.depth;
			stateHatVar[psi] = data->orientation.yaw;

			stateHatVar[x_var] = data->position_variance.north;
			stateHatVar[y_var] = data->position_variance.east;
			stateHatVar[z_var] = data->position_variance.depth;
			stateHatVar[psi_var] = data->orientation_variance.yaw;

			stateHatVar[alt] = data->altitude;

		}
	}
}

//using namespace labust::data;

class LOD : public labust::data::DataManager{

public:

	ros::Subscriber subStateHatAbs, subRequestLawn, subMissionOffset, subJoy;
	ros::Publisher pubEvent;

	ros::Timer timer;

	bool requestLawnFlag;

	int counter;

	auv_msgs::NED offset;

	//enum {u=0, v, w, r, x, y, z, psi, x_var, y_var, z_var, psi_var, alt, stateHatNum};

	double startLawnX, startLawnY;


	LOD():startLawnX(0.0),startLawnY(0.0), requestLawnFlag(false), counter(0){

		ros::NodeHandle nh;

		/* Subscribers */
		subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",5, &LOD::onStateHat, this);
		subRequestLawn = nh.subscribe<std_msgs::Bool>("requestLawn",1, &LOD::onRequestLawn, this);
		subMissionOffset = nh.subscribe<auv_msgs::NED>("missionOffset",1, &LOD::onMissionOffset, this);
		subJoy = nh.subscribe<sensor_msgs::Joy>("joy", 1, &LOD::onJoy,this);



		/* Publishers */
		pubEvent = nh.advertise<misc_msgs::ExternalEvent>("externalEvent",3);

		//missionLoop();


	}

	void onStateHat(const auv_msgs::NavSts::ConstPtr& data){

		updateData(data);
		//ROS_ERROR("UPDATE");
		ros::NodeHandle nh;
		if(requestLawnFlag == true){
			timer = nh.createTimer(ros::Duration(2.0), &LOD::onTimeout, this, true);
			requestLawnFlag = false;
		}

	}

	void onTimeout(const ros::TimerEvent& timer){

		//ROS_ERROR("Timeout");
		//mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
		//checkEventFlag = false;
		misc_msgs::ExternalEvent sendEvent;

		sendEvent.id = 1;
		sendEvent.value = 0;
		pubEvent.publish(sendEvent);


	}

	void onRequestLawn(const std_msgs::Bool::ConstPtr& req){

		if(req->data){
			//requestLawnFlag = true;

			misc_msgs::ExternalEvent sendEvent;
			sendEvent.id = 2;
			sendEvent.value = startLawnX = stateHatVar[x] + offset.north;
			pubEvent.publish(sendEvent);

			sendEvent.id = 3;
			sendEvent.value = startLawnY = stateHatVar[y] + offset.east;
			pubEvent.publish(sendEvent);

			sendEvent.id = 1;
			sendEvent.value = 1;
			pubEvent.publish(sendEvent);

			requestLawnFlag = true;
		}
	}

	void onMissionOffset(const auv_msgs::NED::ConstPtr& data){

		offset.north = data->north;
		offset.east = data->east;
	}

	void onJoy(const sensor_msgs::Joy::ConstPtr& data){



		if(data->buttons[1]){

			ROS_ERROR("pritisak");

			misc_msgs::ExternalEvent sendEvent;
			sendEvent.id = 2;
			sendEvent.value = startLawnX = stateHatVar[x] + offset.north;
			pubEvent.publish(sendEvent);

			sendEvent.id = 3;
			sendEvent.value = startLawnY = stateHatVar[y] + offset.east;
			pubEvent.publish(sendEvent);

			sendEvent.id = 1;
			sendEvent.value = 1;
			pubEvent.publish(sendEvent);

			requestLawnFlag = true;
		}
	}


};


int main(int argc, char** argv){

	ros::init(argc, argv, "LOD");
	ros::NodeHandle nh;

	LOD lawnmowerOnDemand;
	//lawnmowerOnDemand.missionLoop();

	ros::spin();
	return 0;
}



