/*
 * course_keeping_turns.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: Filip Mandic
 */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER#include <sensor_msgs/Joy.h>
*
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
*  THIS SOFTWARE IS   seq: 230
*  PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
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
////#include <exprtk/exprtk.hpp>
#include <sensor_msgs/Joy.h>

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


class CKT : public labust::data::DataManager{

public:

	ros::Subscriber subStateHatAbs, subRequestLawn, subMissionOffset, subJoy;
	ros::Publisher pubEvent;

	ros::Timer timer;

	bool requestTurn;

	double course;



	auv_msgs::NED offset;

	//enum {u=0, v, w, r, x, y, z, psi, x_var, y_var, z_var, psi_var, alt, stateHatNum};

	double startLawnX, startLawnY;


	CKT():startLawnX(0.0),startLawnY(0.0), requestTurn(false), course(0){

		ros::NodeHandle nh;

		/* Subscribers */
		subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbsSlow",1, &CKT::onStateHat, this);
		//subRequestLawn = nh.subscribe<std_msgs::Bool>("requestLawn",1, &CKT::onRequestLawn, this);
		//subMissionOffset = nh.subscribe<auv_msgs::NED>("missionOffset",1, &CKT::onMissionOffset, this);
		subJoy = nh.subscribe<sensor_msgs::Joy>("joy", 1, &CKT::onJoy,this);



		/* Publishers */
		pubEvent = nh.advertise<misc_msgs::ExternalEvent>("externalEvent",1);

		//missionLoop();


	}

	void onStateHat(const auv_msgs::NavSts::ConstPtr& data){

		updateData(data);
		//ROS_ERROR("UPDATE");
		ros::NodeHandle nh;

		misc_msgs::ExternalEvent sendEvent;


		if(requestTurn == true){

			//misc_msgs::ExternalEvent sendEvent;


			timer = nh.createTimer(ros::Duration(0.5), &CKT::onTimeout, this, true);
			requestTurn = false;

			ROS_ERROR("KURS: %f", course*180/M_PI);
		}



	}


	void onTimeout(const ros::TimerEvent& timer){

		//ROS_ERROR("Timeout");
		//mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
		//checkEventFlag = false;
		misc_msgs::ExternalEvent sendEvent;

		sendEvent.id = 2;
		sendEvent.value = 0;
		pubEvent.publish(sendEvent);





//		sendEvent.id = 2;
//		sendEvent.value = 0;
//		pubEvent.publish(sendEvent);



	}


	void onJoy(const sensor_msgs::Joy::ConstPtr& data){

		misc_msgs::ExternalEvent sendEvent;
			if(data->buttons[2]){

				ROS_ERROR("lijevo");

				course -= M_PI/2;

				if(course>M_PI){
					course -= 2*M_PI;
				}else if(course<-M_PI){
					course += 2*M_PI;
				}


					sendEvent.id = 5;
					sendEvent.value = course;
					pubEvent.publish(sendEvent);

				requestTurn = true;
			} else if(data->buttons[3]){

				ROS_ERROR("desno");

				course += M_PI/2;

				if(course>M_PI){
					course -= 2*M_PI;
				}else if(course<-M_PI){
					course += 2*M_PI;
				}


								sendEvent.id = 5;
								sendEvent.value = course;
								pubEvent.publish(sendEvent);
				requestTurn = true;

			}

			if(requestTurn){
				sendEvent.id = 2;
				sendEvent.value = 1;
				pubEvent.publish(sendEvent);
			}
		}


};










int main(int argc, char** argv){

	ros::init(argc, argv, "LOD");
	ros::NodeHandle nh;

	CKT courseKeepingTurns;
	//lawnmowerOnDemand.missionLoop();

	ros::spin();
	return 0;
}







