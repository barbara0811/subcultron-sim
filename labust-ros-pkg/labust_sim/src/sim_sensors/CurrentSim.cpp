/*********************************************************************
 * Current simulator. Node reads static current data from currentInfo.txt file 
 * located in labust_sim/data/current/. Depending on agent's position, current
 * value is being published to "current_sensor" topic (and "currents" topic if
 * agent position's depth component is smaller than param current_depth).
 *********************************************************************/

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/NED.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <math.h>

#define PI 3.14159265

class CurrentRegion {
    geometry_msgs::Point topLeft, bottomRight;
    geometry_msgs::TwistStamped current; 
    std::string mode;
    int time_sec;
    int curr_change;
  public:
    CurrentRegion (geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::TwistStamped c, std::string m)
    {
	mode = m;
	ROS_ERROR("%s", mode.c_str());
	topLeft = p1;
	bottomRight = p2;
	current = c;
	time_sec = 0;
        curr_change = ros::Time::now().sec + ros::Duration(10).sec;
    }

    bool pointInRegion (geometry_msgs::Point point)
    {
	// x --> north, y --> east
	if ((point.y > topLeft.y) && (point.y <= bottomRight.y) && (point.x <= topLeft.x) && (point.x > bottomRight.x))
		return true;
	else
		return false;
    }

    geometry_msgs::TwistStamped getCurrent()
    {
	geometry_msgs::TwistStamped curr;
	if (mode == "const")
		return current;	
	else if (mode == "periodic")
	{
		int t = ros::Time::now().sec;
		if (ros::Time::now().sec >= curr_change)
		{
		    time_sec += 10;
		    curr_change += ros::Duration(5, 0).sec;
		}
		double angle = (double)(time_sec % 360) * PI / 180;
		curr.twist.linear.x = cos(angle);
		curr.twist.linear.y = sin(angle);
	        return curr;
	}
    }
};

struct CurrentSim
{
	CurrentSim():
		currentDepth(0.5),
		currentMode("const")
	{
		start = false;
		currentInfoLoaded = false;
		currentPublished = false;
		ros::NodeHandle nh, ph("~");
		ph.getParam("current_depth", currentDepth);
		ph.getParam("current_mode", currentMode);

		position = geometry_msgs::Point();
		current = geometry_msgs::TwistStamped();
	
		myRegion = -1;

		positionSub = nh.subscribe<auv_msgs::NavSts>("position", 1, &CurrentSim::onPosition, this);
		currentSensorPub = nh.advertise<geometry_msgs::TwistStamped>("current_sensor", 1);
		currentPub = nh.advertise<geometry_msgs::TwistStamped>("currents", 1);
		
		currPubTimeout = ros::Time::now() + ros::Duration(0.2);
		startSub = nh.subscribe<std_msgs::Bool>("start_curr_sim", 1, &CurrentSim::onStart, this);
	}

	void onStart(const typename std_msgs::Bool::ConstPtr& msg)
	{
		loadCurrentInfo(ros::package::getPath("labust_sim") + "/data/current/currentInfo.txt");
		start = true;
	}

	void onPosition(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		if ((not currentInfoLoaded) or (not start))
		{
		    return;
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

		// publish only when current changes or current was not published in some time
		if ((newCurrent.twist.linear.x != current.twist.linear.x) || (newCurrent.twist.linear.y != current.twist.linear.y) 
			|| (newCurrent.twist.linear.z != current.twist.linear.z) || (not currentPublished) || (currPubTimeout > ros::Time::now()))
		{
		    ROS_INFO("CURRENT CHANGE ... %f %f %f -- %f %f", newCurrent.twist.linear.x, newCurrent.twist.linear.y, newCurrent.twist.linear.z, position.x, position.y);
		    current = newCurrent;
		    // set current sensor info
    		    currentSensorPub.publish(newCurrent);

  		    // check depth and set current accordingly
		    if (position.z <= currentDepth) 
		    {
		    	currentPub.publish(newCurrent);
		    }
		    else
		    {
			newCurrent.twist.linear.x = 0;
		    	newCurrent.twist.linear.y = 0;
		    	newCurrent.twist.linear.z = 0;
			currentPub.publish(newCurrent);
		    }

		    currPubTimeout = ros::Time::now() + ros::Duration(0.2);
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
			p1.x = a;  // north
			p1.y = b;  // east

			// lower right point
			if ((err = readLine(&file, &line)) != 0) {err = -2; break;}
    			if (sscanf(line.c_str(), "%f %f\n", &a, &b) != 2) {err = -2; break;}
			p2.x = a;  // north
			p2.y = b;  // east

			// current value
			if ((err = readLine(&file, &line)) != 0) {err = -2; break;}
    			if (sscanf(line.c_str(), "%f %f %f\n", &a, &b, &c) != 3) {err = -2; break;}
			curr.twist.linear.x = a;  // north
			curr.twist.linear.y = b;  // east
			curr.twist.linear.z = c;  // depth

			currentRegions.push_back(CurrentRegion(p1, p2, curr, currentMode));
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
	bool start;
	ros::Subscriber startSub;
	ros::Subscriber positionSub;
	// current sensor info
	ros::Publisher currentSensorPub;
	// current value affecting agent
	ros::Publisher currentPub;
	bool currentInfoLoaded;
	bool currentPublished;

	double currentDepth;
	std::string currentMode;
	ros::Time currPubTimeout;

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


