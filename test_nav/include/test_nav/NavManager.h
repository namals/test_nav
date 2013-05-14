/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Nanyang Technological University.
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
*   * Neither the name of the Nanyang Technological University nor the names of its
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
*
* Author: P.G. Chaminda Namal Senarathne
*********************************************************************/
#ifndef _TEST_NAV_NAV_MANAGER_H_
#define _TEST_NAV_NAV_MANAGER_H_

#include <cstring>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <GPS/gps.h>
#include <compass/compass.h>
#include <test_nav/NextWayPoint.h>
#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#define PI 3.14159265359

struct GPSPoint
{
	double latitude;
	double longitude;
};

struct Point2D
{
	double x;
	double y;
};

/**
 * @struct CoordinateFrame
 * @brief A structure that stores the transformation of the origin of a new coordinate frame with respect to the global frame
 */
struct CoordinateFrame
{
	GPSPoint offset;			/**< Latitude, Longitude values of origin (0,0) of coordinate frame */
	double bearing;				/**< The bearing of the x-axis with respect to north in clockwise direction */
};

/**
 * @class NavManager
 * @brief A class that provides a higher level controller for GPS based way point navigation
 *
 * Given a sequence of way points (specified in GPS coordinates), this class provides a higher
 * controller to move through the sequence of way points. This is done by issuing next waypoint
 * in (x,y) coordinates of the current coordinate frame. The target waypoint in (x,y) is reached
 * via a standard navigation controller such as move_base. However, this controller keeps track
 * of the robot's position using GPS measurements and decides whether robot has reached the current
 * waypoint independently of the lower level planner. 
 */
class NavManager
{
public:
	NavManager();
	virtual ~NavManager();

private:
	ros::NodeHandle* n;
	ros::Subscriber gps_sub;
	ros::Subscriber denode_sub;	            /**< wait for empty signal */
	ros::Publisher wp_pub;					/**< waypoint publisher */
	ros::Publisher reached_goal_signal_pub; /**< publisher to signal goal reached */
	ros::Publisher odom_reset_pub;          /**< publisher to signal a resetting of TF tree */
	tf::TransformListener* listener;        /**< listener used to get position */
	std::string gps_topic;					/**< stores the gps topic name */

	bool use_ogmapper;             /**< Flag to indicate whether to use ogmapper or not */
	CoordinateFrame local_frame;   /**< Stores offset and bearing information of the local coordinate frame in use */
	volatile bool has_gps;		   /**< Flag to indicate whether we have GPS or not */
	volatile bool de_reached_wp;   /**< Flag to indicate whether to the underlying lower level controller drove the robot to the waypoint  */
	volatile bool moved_to_beacon; /**< Flag to indicate that the robot moved to a virtual beacon along the path towards next way point */
	volatile double compass_yaw;   /**< Stoes the the most recent yaw information from compass */
	GPSPoint pos_gps;              /**< stores the most recent GPS value received */
	Point2D pos_utm;               /**< position of the robot in the current coordinate frame according to GPS  */
	Point2D pos_map;			   /**< position of the robot in the current coordinate frame according to SLAM  */
	GPSPoint cur_origin_gps;	   /**< GPS value of the origin of the current coordinate frame */
	GPSPoint wp_gps;			   /**< Current GPS way point */
	Point2D wp_utm;				   /**< Current way point converted to the x,y values of current coordinate frame  */	
	double beta_map;               // current bearing in current /map coordinate frame
	Point2D wp_map;				   /**< current way point in map coordinates */
	std::queue<GPSPoint>* waypoints; /**< Stores the current sequence of waypoints for the robot to visit */
	double initial_bearing;			 /**< initial robot orientation w.r.t. north */
	std::string wp_fname;			 /**< full qualified name of the file storing the waypoints  */
	
	double neighborhood_radius;	       /**< Radius of the circle that includes the neighborhood surrouding a waypoint */
	double target_epsilon;			   /**< The tolerance used by the high level controller when reaching a waypoint */
	double neighborhood_timeout_th;    /**< neighbourhood timeout threshold */
	double inter_beacon_dist;		   /**< Inter beacon distance */
	double min_dist_from_beacon_to_wp; /**< The minimum distance a virtual beacon should have with the final waypoint target */
	bool is_in_nbhd;				   /**< Flag to indicate if the robot is in the neighborhood of current waypoint */
	ros::Time neighborhood_time;	   /**< Timer used during the movements inside the neighborhood of waypoint */
	ros::Time gps_ts;				   /**< Timestamp of the most recent GPS measurement */

	boost::mutex gps_mutex; 
	boost::mutex beta_mutex;
	
	boost::shared_ptr<boost::thread> controller_thread;
	boost::shared_ptr<boost::thread> pos_check_thread;

	/**
	 * @brief Reads the sequence of waypoints from a file and stores them in a queue
	 */
	void ReadWayPoints();

	/**
	 * @brief Waits until the initial GPS measurement is received
	 */	
	void WaitForInitialGPSFix();

	/**
	 * @brief Function used by the main thread of the controller
	 */
	void CheckStatus();

	/**
	 * @brief Periodically checks the position of the robot in (x,y) in the current coordinate frame
	 */
	void CheckPos();

	/**
	 * @brief Converts a GPS poit to the range and bearing from the origin of the current local frame
	 * @param[in] point GPS Point
	 * @param[out] r Range of the GPS point from the origin of the local coordinate frame
	 * @param[out] theta Bearing of the GPS point from the origin of the local coordinat frame with repsect to north in clockwise direction
	 * @return A Point2D representation of the GPS in the local frame
	 */
	Point2D ConvertGPSToLocalFrame(GPSPoint point, double& r, double& theta);

	/**
	 * @brief Reset ogmapper
	 */
	bool ResetOgMapping();

	/**
	 * @brief Reset the local SLAM instance
	 */
	void ResetLocalMapping();

	/**
	 * @brief Wait for local SLAM (GMapping) to finish initialization
	 */
	void WaitForGmappingToStabilize();	
	
	// callbacks
	void GPS_cb(const GPS::gps::ConstPtr& gps_msg);
	void DENode_cb(const std_msgs::Empty::ConstPtr& msg);
	void Compass_cb(const compass::compass::ConstPtr& compass_msg);

	inline double Euclidean_Dist(Point2D& p1, Point2D& p2)	
	{
		return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
	}

	inline bool IsNeighborhoodTimedOut()
	{
		ros::Time cur_time = ros::Time::now();
		ros::Duration d = cur_time - neighborhood_time;
		return d.toSec() >= neighborhood_timeout_th;
	}

	inline void ResetNeighborhoodTime()
	{
		neighborhood_time = ros::Time::now();
	}   	

	inline void SendNextWayPoint(double r, double theta)
	{
		test_nav::NextWayPoint wp;
		wp.range = r;
		wp.bearing = theta;   
		ROS_INFO("Sending next way point with range, bearing : %f, %f", wp.range, wp.bearing*180/PI);
		wp_pub.publish(wp);			
	}

	inline bool GetCurrentGPSPoint(GPSPoint & gps_point)
	{
		bool result;
		gps_mutex.lock();
		ros::Duration d = ros::Time::now() - gps_ts;
		result = (d.toSec() <= 3.0)? true : false;
		gps_point = pos_gps;
		gps_mutex.unlock();
		return result;
	}
};

#endif

