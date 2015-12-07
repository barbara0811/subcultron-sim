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
#include <ros/ros.h>

struct BatterySensorSim
{
	BatterySensorSim():
		batteryThresh(5) // default value
	{
		start = false;
		ros::NodeHandle nh, ph("~");
		ph.getParam("battery_threshold", batteryThresh);

		batteryPublished = false;
		gotPosition = false;

		position = auv_msgs::NED();
		batteryPercentage = std_msgs::Int32();
		batteryPercentage.data=100;

		// subscribers
		positionSub = nh.subscribe<auv_msgs::NavSts>("position", 1, &BatterySensorSim::onPosition, this);
		//chargeSub = nh.subscribe<auv_msgs::NavSts>("charging", 1, &BatterySensorSim::onPosition, this);
		// publishers
		batteryPub = nh.advertise<std_msgs::Int32>("battery_level", 1);
		surfaceReq = nh.advertise<std_msgs::Bool>("goto_surface", 1);
		alertPub = nh.advertise<auv_msgs::NED>("/battery_alert", 1);
		
		startSub = nh.subscribe<std_msgs::Bool>("start_sim", 1, &BatterySensorSim::onStart, this);
		timer = nh.createTimer(ros::Duration(1), &BatterySensorSim::timerCallback, this);
	}
	
	void timerCallback(const ros::TimerEvent& event)
	{
		if ((not gotPosition) or (not start))
		    return;
		
		if (batteryPercentage.data>0) batteryPercentage.data-=1;
        batteryPub.publish(batteryPercentage);
        std_msgs::Bool flag;
        flag.data=true;
        if (batteryPercentage.data<batteryThresh) surfaceReq.publish(flag);
        if ((batteryPercentage.data<batteryThresh) && (position.depth<1.0)) alertPub.publish(position); 
	}

	void onStart(const typename std_msgs::Bool::ConstPtr& msg)
	{
		start = true;
	}

	void onPosition(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		// set position
		position.north = msg->position.north;
		position.east = msg->position.east;
		position.depth = msg->position.depth;
		gotPosition = true;
	}

	

private:
	// flags
	bool start;
	bool gotPosition;
	bool batteryPublished;

	ros::Subscriber startSub;
	ros::Subscriber positionSub;
	ros::Subscriber chargeSub;
	ros::Publisher batteryPub;
	ros::Publisher alertPub;
	ros::Publisher surfaceReq;

	// battery level range
	int batteryThresh;
	std_msgs::Int32 batteryPercentage;
	int agentId;
	ros::Timer timer;

	auv_msgs::NED position;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"battery_sensor_sim");
	ros::NodeHandle nh;
	BatterySensorSim batterySensorSim;
	ros::spin();
	return 0;
}


