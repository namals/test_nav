#include <test_nav/NavManager.h>
#include <ogmapper/ResetOgMap.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <fstream>

NavManager::NavManager()
{
	n = new ros::NodeHandle();

	ros::NodeHandle private_n("~");
	if( !private_n.getParam("initial_bearing", initial_bearing) )
	{
		ROS_ERROR("Initial bearing is not provided, exiting... ");
		exit(0);
	}
	if( !private_n.getParam("waypoint_fname", wp_fname) )
	{		
		ROS_ERROR("Waypoint file name is not provided, exiting...");
		exit(0);
	}
	if( !private_n.getParam("neighborhood_radius", neighborhood_radius) )
    {
        neighborhood_radius = 3;
    }
    if( !private_n.getParam("target_epsilon", target_epsilon) )
    {
        target_epsilon = 1.5;
    }
    if( !private_n.getParam("neighborhood_timeout_threshold", neighborhood_timeout_th) )
    {
        neighborhood_timeout_th = 60;   // timeout of 10 seconds 
	}	
	if( !private_n.getParam("gps_topic", gps_topic) )
	{
		gps_topic = "/drrobot_gps_info";
	}
	if( !private_n.getParam("use_ogmapper", use_ogmapper) )
	{
		use_ogmapper = false;
	}
	if( !private_n.getParam("inter_beacon_dist", inter_beacon_dist) )
	{
		inter_beacon_dist = 2.0;
	}
	if( !private_n.getParam("min_dist_from_beacon_to_wp", min_dist_from_beacon_to_wp) )
	{
		min_dist_from_beacon_to_wp = 1.0;
	}

	has_gps = false;
	de_reached_wp = false;
	moved_to_beacon = false;
	beta_map = 0.0;
	listener = new tf::TransformListener();

	ReadWayPoints();   // temporary way to get way points

	do
	{
		ROS_INFO("Subscribing to gps messages");
		gps_sub = n->subscribe<GPS::gps>(gps_topic, 1, &NavManager::GPS_cb, this);
	}while(!gps_sub);

	do
	{
		ROS_INFO("Subscribing to de_node's done signal");
		denode_sub = n->subscribe<std_msgs::Empty>("reached_map_goal", 1, &NavManager::DENode_cb, this);
	}while(!denode_sub);

	wp_pub = n->advertise<test_nav::NextWayPoint>("next_wp", 1); // topic on which way points are sent to the de_planner
	reached_goal_signal_pub = n->advertise<std_msgs::Empty>("reached_gps_goal", 1);
	odom_reset_pub = n->advertise<std_msgs::Empty>("/reset_odom", 1);

	// create threads
	controller_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&NavManager::CheckStatus, this)) );
	pos_check_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&NavManager::CheckPos, this)));
}

NavManager::~NavManager()
{
	delete waypoints;
	delete n;
}

void NavManager::ReadWayPoints()
{
	ROS_INFO("Reading way point information from file");
	typedef std::vector<std::string> split_vector_type;	
	waypoints = new std::queue<GPSPoint>();
	std::ifstream ifile(wp_fname.c_str(), std::ifstream::in);	
	if( ifile.is_open() )
	{
		do
		{
			std::string latlon;
			std::getline(ifile,latlon);						
			if( latlon.length() <= 0 )
                continue;

			ROS_INFO("%s", latlon.c_str());
			// assuming lat lon are separated by a tab in a single line
            split_vector_type split_vec;
			boost::algorithm::split( split_vec, latlon, boost::algorithm::is_any_of("\t") );   

            if( split_vec.size() == 2 )
            {
                GPSPoint p;
                //p.latitude = boost::lexical_cast<double>(split_vec[1]);
				p.latitude = atof(split_vec[1].c_str());
                //p.longitude = boost::lexical_cast<double>(split_vec[0]);
				p.longitude = atof(split_vec[0].c_str());
                ROS_INFO("%f, %f", p.latitude, p.longitude);
                waypoints->push(p);
            }
		}while(!ifile.eof());
	}
}

void NavManager::GPS_cb(const GPS::gps::ConstPtr & gps_msg)
{
	if( gps_msg->No_Sat > 3 )	// discard gps fixes with less than 4 sattelites
	{		
		has_gps = true;
		double lat = (gps_msg->Latitude - ((int)(gps_msg->Latitude/100))*100)/60 + ((int)(gps_msg->Latitude/100));
		double lon = (gps_msg->Longitude - ((int)(gps_msg->Longitude/100))*100)/60 + ((int)(gps_msg->Longitude/100));
		gps_mutex.lock();
		pos_gps.latitude = lat;
		pos_gps.longitude = lon;		
		gps_ts = ros::Time::now();
		gps_mutex.unlock();		
	}
}

/*
 * Callback function for messages from de_node signalling reaching assigned way points 
 */
void NavManager::DENode_cb(const std_msgs::Empty::ConstPtr & msg)
{
	ROS_INFO("DE_Node signals that robot reached goal according to map coordinate");
	de_reached_wp = true;
}

void NavManager::WaitForInitialGPSFix()
{
	while( !has_gps )
	{
		ROS_INFO("Waiting for initial GPS fix......................");
		usleep(50000);
		continue;
	}
	ROS_INFO("Initial GPS received\n------------------------------------------");
	gps_mutex.lock();
	//cur_origin_gps = pos_gps;  // initialize the gps point value of the current coordinate frame's origin
	local_frame.offset = pos_gps;
	//local_frame.bearing = initial_bearing;
	gps_mutex.unlock();
}

void NavManager::CheckStatus()
{
	ros::Rate rate(10);
	WaitForInitialGPSFix();      // should this be changed? should it start even if GPS is not available
	sleep(5);                    // wait for 5 seconds to read yaw and close laptop lid

	local_frame.bearing = compass_yaw;
	bool running = true;
	while( running )
	{
		if( !waypoints->empty() )
		{
			if( !moved_to_beacon )
				wp_gps = waypoints->front(); waypoints->pop();	        // retrieve next way point in the high level plan			
			double r, theta;
			wp_utm = ConvertGPSToLocalFrame(wp_gps, r, theta);			
			SendNextWayPoint(r, theta);                                 // send next way point to local navigator, theta in clockwise

			de_reached_wp = false; bool reached_wp = false;	is_in_nbhd = false;	
			bool wp_timedout = false; bool done = false; moved_to_beacon = false;
			while(!done)                                                // wait for local navigator to be done
			{			
				GPSPoint tgt;
				if( GetCurrentGPSPoint(tgt) )
				{					
					pos_utm = ConvertGPSToLocalFrame(tgt, r, theta);
					ROS_INFO("Current position from GPS : %f, %f  | Waypoint position : %f, %f", pos_utm.x, pos_utm.y, wp_utm.x, wp_utm.y);
					if( Euclidean_Dist(pos_utm, wp_utm) <= target_epsilon )
					{
						ROS_INFO("Reached waypoint");
						reached_wp = true;				
					}
					else 
					{
						if( is_in_nbhd )                                           // IF robot is already in neighborhood of GPS way point
						{
							if( IsNeighborhoodTimedOut() )                         // THEN, IF robot spent too much time inside neighborhood
							{
								wp_timedout = true;                                
								ROS_INFO("Neighborhood timedout");
							}
						}
						else if( Euclidean_Dist(pos_utm, wp_utm) <= neighborhood_radius )  // Deciding robot entered waypoint neighborhood
						{
							is_in_nbhd = true;
							ResetNeighborhoodTime();
							ROS_INFO("In neighborhood : flag set");
						}
					}				
				}  // ~ if GPS point is valid
				done = de_reached_wp || reached_wp || wp_timedout || moved_to_beacon;
				rate.sleep();
			} // reached current waypoint at the end of this while loop, or reached the intermediate beacon point
			
			GPSPoint new_offset;                                 // UPDATE Local frame if GPS is available, else use old frame
			if( GetCurrentGPSPoint(new_offset) )
			{
				local_frame.offset = new_offset;
				//beta_mutex.lock();
				//local_frame.bearing = fmod(local_frame.bearing+(-beta_map), 2*PI);   // beta_map is anti-clockwise direction, hence -ve sign
				local_frame.bearing = compass_yaw; // directly using compass's yaw information
				//beta_mutex.unlock();								
				ResetLocalMapping();     // reset localization and mapping
			}
		}
		else
		{
			running = false;
		}		
	}	
	controller_thread->join();
}

void NavManager::CheckPos()
{
	Point2D origin; origin.x = 0.0; origin.y = 0.0;
	ros::Rate r(10);	
	while( n->ok() )
	{
		try
		{
			// get robot_pose
			tf::StampedTransform transform;
			listener->lookupTransform("/map", "/base_link", ros::Time(0), transform);
			pos_map.x = transform.getOrigin().x();
			pos_map.y = transform.getOrigin().y();
			if( Euclidean_Dist(origin, pos_map) >= inter_beacon_dist  && 
				Euclidean_Dist(wp_utm, pos_map) >= min_dist_from_beacon_to_wp )
				moved_to_beacon = true;
			// record beta_map
			beta_mutex.lock();
			beta_map = tf::getYaw( transform.getRotation() );
			beta_mutex.unlock();			
		}
		catch(tf::TransformException & ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		r.sleep();
	}
	pos_check_thread->join();
}

Point2D NavManager::ConvertGPSToLocalFrame(GPSPoint target, double& r, double& theta)
{	
	GPSPoint origin = local_frame.offset;

	origin.latitude = origin.latitude*PI/180.0;
	origin.longitude = origin.longitude*PI/180.0;	
	target.latitude = target.latitude*PI/180.0;
	target.longitude = target.longitude*PI/180.0;
	r = acos( sin(origin.latitude)*sin(target.latitude) + cos(origin.latitude)*cos(target.latitude)*cos(target.longitude-origin.longitude) )*6371*1000;    // in meters
	double alpha = atan2( sin(target.longitude-origin.longitude)*cos(target.latitude), cos(origin.latitude)*sin(target.latitude)-sin(origin.latitude)*cos(target.latitude)*cos(target.longitude-origin.longitude) );  // in radians, clockwise w.r.t North
	//theta = alpha-beta_i;   // offset angle in clockwise direction w.r.t to robot's heading
	theta = alpha-local_frame.bearing; // bearing of the target point w.r.t to robot's heading in clockwise direction

	Point2D tp;
	tp.x = r*cos(theta);
	tp.y = -r*sin(theta);
	return tp;
}

bool NavManager::ResetOgMapping()
{
	ogmapper::ResetOgMap srv;
	ros::service::waitForService("/ogmapper/reset_ogmap");
	ros::ServiceClient ogmapperclient = n->serviceClient<ogmapper::ResetOgMap>("/ogmapper/reset_ogmap");	
	if( !ogmapperclient.call(srv) )
	{
		ROS_ERROR("Error in resetting ogmap");
		return false;
	}
	return true;
}

/*
 * This function waits until initialization of gmapping is complete
 * At the moment this is done by waiting for the service dynamic_map 
 */
void NavManager::WaitForGmappingToStabilize()
{
	ROS_INFO("Waiting for GMapping to fully initialize..........");
	ros::service::waitForService("dynamic_map");
}

void NavManager::ResetLocalMapping()
{
	if( !de_reached_wp )                                
	{
		ROS_INFO("Signaling de_planner to stop the robot");
		std_msgs::Empty reached_goal;
		reached_goal_signal_pub.publish(reached_goal);
		usleep(300000);                                                      // wait sometime for de_planner to stop the robot
	}

	ROS_INFO("Stopping current GMapping instance");
	if( system("bash killgmap.sh") == -1 )
		ROS_ERROR("Error in stopping current GMapping instance");    
	sleep(1);  // wait for 2 seconds
	
	ROS_INFO("Resetting the TF tree so that /map, /odom, /base_link all are in (0,0) position");  
	std_msgs::Empty reset;
	odom_reset_pub.publish(reset);
	usleep(200000);  // wait for 200 miliseconds
	
	if( use_ogmapper )
	{
		ROS_INFO("Resetting the Ogmapper");
		while( !ResetOgMapping() );   
	}

	ROS_INFO("Starting a new GMapping instance");
	if( system("bash startgmap.sh") == -1 )
	{
		ROS_ERROR("Error in starting a new GMapping instance");
		exit(1);
	}
	WaitForGmappingToStabilize();	
	sleep(2);
}

void NavManager::Compass_cb(const compass::compass::ConstPtr& compass_msg)
{
	compass_yaw = (compass_msg->Yval/10.0)*PI/180.0;
}
