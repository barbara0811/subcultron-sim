/*********************************************************************
 * Battery simulator. It reads data published to "battery_charging" topic.
 * Current value is published to the agent's "battery_level" topic.
 * Publishes battery alert level message if battery level is below threshold 
 * and agent is on surface.
 *********************************************************************/


#include <auv_msgs/NavSts.h>
#include <auv_msgs/NED.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
 #include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <string>

struct BatterySensorSim
{
	BatterySensorSim()
	{
		start = false;
		sleepMode = false;

		ros::NodeHandle nh, ph("~");
		
		std::string ns = ros::this_node::getNamespace();

		batteryPercentage = std_msgs::Int32();
		batteryPercentage.data=50;

		// subscribers
		powerSleepMode = nh.subscribe<std_msgs::Bool>("power_sleep_mode", 1, &BatterySensorSim::onPowerSleep, this);
		chargeSub = nh.subscribe<std_msgs::Float64>("charging", 1, &BatterySensorSim::onCharge, this);
		// publishers
		batteryPub = nh.advertise<std_msgs::Int32>("battery_level", 1);
		//surfaceReq = nh.advertise<std_msgs::Bool>("goto_surface", 1);
		//alertPub = nh.advertise<auv_msgs::NavSts>("/battery_alert", 1);
		
		startSub = nh.subscribe<std_msgs::Bool>("start_sim", 1, &BatterySensorSim::onStart, this);
		timer = nh.createTimer(ros::Duration(1), &BatterySensorSim::timerCallback, this);
	}
	
	void timerCallback(const ros::TimerEvent& event)
	{
		if ((not start) or sleepMode)
		    return;
		
		if (batteryPercentage.data>0) batteryPercentage.data-=1;
        batteryPub.publish(batteryPercentage);
	}

	void onStart(const typename std_msgs::Bool::ConstPtr& msg)
	{
		start = true;
	}

	void onPowerSleep(const typename std_msgs::Bool::ConstPtr& msg)
	{
		sleepMode = msg->data;
	}

	/*
	void onPosition(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		// set position
		position.position.north = msg->position.north;
		position.position.east = msg->position.east;
		position.position.depth = msg->position.depth;
		position.header.frame_id=agentId;
		gotPosition = true;
	}
	*/

	

private:
	// flags
	bool start;
	bool sleepMode;
	//bool gotPosition;
	//bool batteryPublished;

	ros::Subscriber startSub;
	ros::Subscriber powerSleepMode;
	//ros::Subscriber positionSub;
	//ros::Subscriber chargeSub;
	ros::Publisher batteryPub;
	//ros::Publisher alertPub;
	//ros::Publisher surfaceReq;

	// battery level range
	//int batteryThresh;
	std_msgs::Int32 batteryPercentage;
	//std::string agentId;
	ros::Timer timer;

	//auv_msgs::NavSts position;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"battery_sensor_sim");
	ros::NodeHandle nh;
	BatterySensorSim batterySensorSim;
	ros::spin();
	return 0;
}


