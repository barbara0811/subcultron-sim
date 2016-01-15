/*********************************************************************
 * Docking simulator. Counts number of docked agents.
 *********************************************************************/

#include <auv_msgs/NavSts.h>
#include <auv_msgs/NED.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
 #include <misc_msgs/GetDockingInfo.h>
#include <ros/ros.h>

struct DockingSim
{
	DockingSim():
		maxDocked(4) // default value
	{
		ros::NodeHandle nh, ph("~");
		ph.getParam("maximum_docked", maxDocked);

		slots = std::vector<std::string>(maxDocked, ""); // docking slot occupators (namespaces)

		// subscribers
		newDockedSub = nh.subscribe<auv_msgs::NavSts>("docked", 1, &DockingSim::onDocked, this);

		// services
		checkAvailabilitySrv = nh.advertiseService("check_docking_availability", &DockingSim::checkDockingAvailability, this);
	}

	
	void onDocked(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		if (numberDocked < maxDocked)
		{ 
			// find an empty slot
			for(int i = 0; i < maxDocked; i++)
			{
				if (slots[i].length() == 0)
				{
					ROS_ERROR("empty slot!!");
					slots[i] = msg->header.frame_id; // assign the node to slot
					break;
				}
			}
			ROS_ERROR("!!");
			numberDocked += 1;
		}
		ROS_ERROR("!! %d %d", numberDocked,maxDocked);
	}
	
	void onReleased(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		if (numberDocked > 0) 
		{
			for(int i = 0; i < maxDocked; i++)
			{
				if (slots[i].compare(msg->header.frame_id))
				{
					slots[i] = "";	// remove the node from slot
					break;
				}
			}
			numberDocked -= 1;
		}
	}

	bool checkDockingAvailability(misc_msgs::GetDockingInfo::Request &req, misc_msgs::GetDockingInfo::Response &resp)
	{
		std::vector<int> available;

		for(int i = 0; i < maxDocked; i++)
		{
			if (slots[i].length() == 0)
			{
				available.push_back(i);
			}
		}

		resp.available_slots = available;
		resp.slots = slots;
		return true;
	}

private:

	ros::Subscriber newDockedSub;

	ros::ServiceServer checkAvailabilitySrv;

	std::vector<std::string> slots;
	int maxDocked, numberDocked;
	int agentId;
	ros::Timer timer;

};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"dockking_sim");
	ros::NodeHandle nh;
	DockingSim dockingSim;
	ros::spin();
	return 0;
}


