/*********************************************************************
 * Docking simulator. Counts number of docked agents.
 *********************************************************************/


#include <auv_msgs/NavSts.h>
#include <auv_msgs/NED.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

struct DockingSim
{
	DockingSim():
		maxDocked(4) // default value
	{
		start = false;
		ros::NodeHandle nh, ph("~");
		ph.getParam("maximum_docked", maxDocked);

		gotPosition = false;

		position = auv_msgs::NED();

		// subscribers
		positionSub = nh.subscribe<auv_msgs::NavSts>("position", 1, &DockingSim::onPosition, this);
		newDockedSub = nh.subscribe<auv_msgs::NavSts>("docked", 1, &DockingSim::onDocked, this);
		
		// publishers
        //chargePub = nh.subscribe<auv_msgs::NavSts>("charging", 1, &DockingSim::onPosition, this);
        maxReachedPub = nh.advertise<std_msgs::Int32>("maximum_docked_reached", 1);
		
		startSub = nh.subscribe<std_msgs::Bool>("start_sim", 1, &DockingSim::onStart, this);
		timer = nh.createTimer(ros::Duration(2), &DockingSim::timerCallback, this);
	}
	
	void timerCallback(const ros::TimerEvent& event)
	{
		if ((not gotPosition) or (not start))
		    return;
		    
		if (numberDocked.data==maxDocked) maxReachedPub.publish(numberDocked); 
		
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
	
	void onDocked(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		if (numberDocked.data<maxDocked) numberDocked.data+=1;
	}
	
	void onReleased(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		if (numberDocked.data>0) numberDocked.data-=1;
	}

	

private:
	// flags
	bool start;
	bool gotPosition;

	ros::Subscriber startSub;
	ros::Subscriber positionSub;
	ros::Subscriber newDockedSub;
	ros::Publisher chargePub;
	ros::Publisher maxReachedPub;

	// battery level range
	int maxDocked;
	std_msgs::Int32 numberDocked;
	int agentId;
	ros::Timer timer;

	auv_msgs::NED position;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"dockking_sim");
	ros::NodeHandle nh;
	DockingSim dockingSim;
	ros::spin();
	return 0;
}


