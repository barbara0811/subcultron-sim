
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/NED.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

class CurrentRegion {
    geometry_msgs::Point topLeft, bottomRight;
    geometry_msgs::TwistStamped current; 
  public:
    CurrentRegion (geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::TwistStamped c)
    {
	topLeft = p1;
	bottomRight = p2;
	current = c;
    }

    bool pointInRegion (geometry_msgs::Point point)
    {

	if ((point.x > topLeft.x) && (point.x <= bottomRight.x) && (point.y <= topLeft.y) && (point.y > bottomRight.y))
		return true;
	else
		return false;
    }

    geometry_msgs::TwistStamped getCurrent()
    {
	return current;
    }
};

struct CurrentSim
{
	CurrentSim():
		currentDepth(0.5)
	{
		ros::NodeHandle nh; 
		nh.getParam("current_depth", currentDepth);

		position = geometry_msgs::Point();
		current = geometry_msgs::TwistStamped();
	
		loadCurrentInfo(ros::package::getPath("labust_sim") + "/data/current/currentInfo.txt");
		myRegion = -1;

		positionSub = nh.subscribe<auv_msgs::NavSts>("position", 1, &CurrentSim::onPosition, this);
		currentSensorPub = nh.advertise<geometry_msgs::TwistStamped>("current_sensor", 1);
		currentPub = nh.advertise<geometry_msgs::TwistStamped>("currents", 1);
	}

	void onPosition(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		while (not currentInfoLoaded)
		{
		    ros::Duration(0.2).sleep();
		}
		position.x = msg->position.north;
		position.y = msg->position.east;
		position.z = msg->position.depth;
		
		int newRegion = -1; 
		// get current info for new position
		// check if region remains the same
		if (myRegion != -1)
		{			    
		    if (currentRegions[myRegion].pointInRegion(position))
			newRegion = myRegion;
		}
		// region changed, go through all regions to find the right one
		if (newRegion == -1)
		{
		    for (int i = 0; i < currentRegions.size(); i++)
                    {
			if (currentRegions[i].pointInRegion(position))
			{
			    newRegion = i;
			    break;
			}
		    }
		}

		myRegion = newRegion;

		// update current value
		geometry_msgs::TwistStamped newCurrent;
		
		// no region --> no current defined		
		if (myRegion == -1)
		{
		    newCurrent.twist.linear.x = 0;
		    newCurrent.twist.linear.y = 0;
		    newCurrent.twist.linear.z = 0;
		}
		else
		{
		    newCurrent = currentRegions[myRegion].getCurrent();
		}

		// publish only when current changes
		if ((newCurrent.twist.linear.x != current.twist.linear.x) || (newCurrent.twist.linear.y != current.twist.linear.y) 
			|| (newCurrent.twist.linear.z != current.twist.linear.z) || not currentPublished)
		{
		    ROS_INFO("CURRENT CHANGE ... %f %f %f -- %f %f", newCurrent.twist.linear.x, newCurrent.twist.linear.y, newCurrent.twist.linear.z, position.x, position.y);
		    current = newCurrent;
		    // set current sensor info
    		    currentSensorPub.publish(newCurrent);

  		    // check depth and set current accordingly
		    if (position.z < currentDepth) 
		    {
		    	currentPub.publish(newCurrent);
			ROS_ERROR("Publishing current %s -- %f %f", ros::this_node::getNamespace().c_str(), newCurrent.twist.linear.x, newCurrent.twist.linear.y);
		    }

		    currentPublished = true;
		}
	}

	void loadCurrentInfo(std::string filename)
	{
		std::ifstream file (filename);
		if (file.is_open())
  		{		    
		    std::string line;
    		    float a, b, c;
		    int err = 0;
		    while (true)
		    {
			geometry_msgs::Point p1, p2;
     		        geometry_msgs::TwistStamped curr;

			// upper left point
			if ((err = readLine(&file, &line)) != 0) {break;}
    			if (sscanf(line.c_str(), "%f %f\n", &a, &b) != 2) {err = -2; break;}
			p1.x = a;
			p1.y = b;

			// lower right point
			if ((err = readLine(&file, &line)) != 0) {err = -2; break;}
    			if (sscanf(line.c_str(), "%f %f\n", &a, &b) != 2) {err = -2; break;}
			p2.x = a;
			p2.y = b;

			// current value
			if ((err = readLine(&file, &line)) != 0) {err = -2; break;}
    			if (sscanf(line.c_str(), "%f %f %f\n", &a, &b, &c) != 3) {err = -2; break;}
			curr.twist.linear.x = a;
			curr.twist.linear.y = b;
			curr.twist.linear.z = c;

			currentRegions.push_back(CurrentRegion(p1, p2, curr));
		    }
		    // err (-2: file format error, -1: end of file (no error))
		    if (err == -2) 
			ROS_ERROR("ERROR current file format: %s", filename.c_str());
		    else
			currentInfoLoaded = true;

		    file.close();
		}
		else ROS_ERROR("Unable to open file %s", filename.c_str());
	}

	int readLine(std::ifstream *file, std::string *line)
	{	
	    bool ok = false;
	    std::string l;
	    while (std::getline(*file, l))
	    {	 
  	    	// comment or empty line
		if (l.length() == 0)
		    continue;
		else if (l.at(0) == '#')
		    continue; 
		else
		{
		    ok = true;
		    break;	
		}
	    }
	    // no error
	    if (ok)
	    {
		*line = l;
	    	return 0;
	    }
	    // end of file
	    else if ((*file).eof()) 
		return -1;
	    // error
	    else 
		return -2;
	}

private:
	ros::Subscriber positionSub;
	// current sensor info
	ros::Publisher currentSensorPub;
	// current value affecting agent
	ros::Publisher currentPub;
	bool currentInfoLoaded = false;
	bool currentPublished = false;

	double currentDepth;

	geometry_msgs::Point position;
	geometry_msgs::TwistStamped current;

	std::vector<CurrentRegion> currentRegions;
	int myRegion;	
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"current_sim");
	ros::NodeHandle nh;
	CurrentSim currentSim;
	ros::spin();
	return 0;
}


