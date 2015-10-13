/*Implementirati samo ping_sensor u zasebnom nodu. Ping_sensor slusa /ping topic i ako je primio poruku od musule udaljene <3m (d_max), 
racuna heading i prosljeduje ga na topic /amusselx/ping_sensor. 
Scenario node onda slusa /amusselx/ping_sensor, puni listu C, racuna prosjek i radi pinganje (slanje na topic /ping) 
jer je to sve dio scenarija tj. ta logika ce se cesto mijenjati.
*/
/*********************************************************************
 * Ping simulator.
 *********************************************************************/

#include <auv_msgs/NavSts.h>
#include <auv_msgs/NED.h>
#include <ros/ros.h>
#include <math.h>

struct PingSim
{
	PingSim():
		pingRange(20),
		pingRate(0.2)
	{ 
		ros::NodeHandle nh, ph("~");
		ph.getParam("ping_range", pingRange);
		position = auv_msgs::NED();

		positionOK = false;
		positionSub = nh.subscribe<auv_msgs::NavSts>("position", 1, &PingSim::onPosition, this);
		pingSub = nh.subscribe<auv_msgs::NED>("/ping", 1000, &PingSim::onPing, this);
		pingPub = nh.advertise<auv_msgs::NED>("/ping", 1000);
		pingSensorPub = nh.advertise<auv_msgs::NED>("ping_sensor", 100);

		timer = nh.createTimer(ros::Duration(0.2), &PingSim::onTimer, this);
	}

	void onPosition(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		position.north = msg->position.north;
		position.east = msg->position.east;
		position.depth = msg->position.depth;
	
		if (not positionOK)
	 	    positionOK = true;
	}

	void onPing(const typename auv_msgs::NED::ConstPtr& msg)
	{
		double d = sqrt(pow(msg->north - position.north, 2) + pow(msg->east - position.east, 2));
		if ((d <= pingRange) && (d != 0))
		{
		    auv_msgs::NED heading;
		    heading.north = (msg->north - position.north) / d;
		    heading.east = (msg->east - position.east) / d;
  
		    pingSensorPub.publish(heading);
		    ros::Duration(0.2).sleep();
		    // relaying ping
		    pingPub.publish(position);
		}
	}		

	void onTimer(const ros::TimerEvent& event)
	{
		// intrinsic ping -- periodic
		if (not positionOK)
		    return;
        	pingPub.publish(position);  
	}


private:
        ros::Subscriber positionSub;
	// global topic ping, communication with other agents
	ros::Subscriber pingSub;
	ros::Publisher pingPub;
	// local topic
	ros::Publisher pingSensorPub;

	bool positionOK;
	double pingRange;
	double pingRate;
	ros::Timer timer;

	auv_msgs::NED position;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"ping_sim");
	ros::NodeHandle nh;
	PingSim pingSim;
	ros::spin();
	return 0;
}

