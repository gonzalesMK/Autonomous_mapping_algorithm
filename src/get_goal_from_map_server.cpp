#include <cmath>
#include <stdio.h>
#include <stdlib.h>

/** include ros libraries**********************/
#include <ros/ros.h>
#include "mapping/get_goal_from_map.h"

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>

#include <map>

struct point{
    int x;
    int y;
    double distance;
};


bool operator==( point A, point B) {
    return  A.x == B.x && A.y == B.y ?  true : false ;
}


bool less_then( point &pointA, point &pointB );
bool greater_then( point &pointA, point &pointB );

bool get_goal(mapping::get_goal_from_map::Request  &req, mapping::get_goal_from_map::Response &res)
{
    system( "clear");

    // Get all the information about the map
    double y_size = req.costmap.info.height;              // cells
    double x_size = req.costmap.info.width;               // cells
    double resolution = req.costmap.info.resolution;      // meters / cell
    geometry_msgs::Pose origin = req.costmap.info.origin;

    // Get the pose of the robot
    geometry_msgs::PoseStamped w_odom = req.odom; /// world_odom in meters
    point m_c_odom;  /// map_cells_odometry
    m_c_odom.x = (w_odom.pose.position.x - origin.position.x) / resolution;
    m_c_odom.y = (w_odom.pose.position.y - origin.position.y) / resolution;
    m_c_odom.distance = 0;

    /**  FOR DEBUGG
    ROS_INFO( " ******* MAP DEBUG **********");
    ROS_INFO_STREAM( "Origin: (" << origin.position.x << "," << origin.position.y <<")" );
    ROS_INFO_STREAM( "Resolution: " << resolution);
    ROS_INFO( " ******* ROBOT POSE  DEBUG *********** " );
    ROS_INFO_STREAM( "World: \t (" << w_odom.pose.position.x << "," << w_odom.pose.position.y << ")");
    ROS_INFO_STREAM( "Costmap: \t (" << m_c_odom.x << "," << m_c_odom.y << ")");
    ROS_INFO_STREAM( " ********* SEARCHING UNKOWN CELLS ************");
    */

    std::map<int, double> distance_map;
    std::list<point> to_visit;

    // Initialize the map
    for( int iy = 0; iy < y_size; iy++){
        for( int ix =0; ix < x_size; ix++ ){
         distance_map.emplace( iy * x_size + ix, std::numeric_limits<double>::infinity() );
        }
    }
    distance_map.at( m_c_odom.y * x_size + m_c_odom.x ) = 0;

    to_visit.push_back( m_c_odom );


   // Start digging for unknow points
    while(to_visit.empty() == false){
        to_visit.sort(less_then);
        point current = to_visit.front();
        to_visit.pop_front();
    //    ROS_INFO_STREAM("CURRENT: (" << current.x <<"," << current.y << ") - DISTANCE " << current.distance );
        if (current.distance == std::numeric_limits<double>::infinity()){
            res.is_finished = true;
            return true;
        }

        for( register int dx = - 1 ; dx <=1 ; dx++){
            for( register int dy = -1; dy <=1 ; dy++){

                point temp;
                temp.x = current.x + dx;
                temp.y = current.y + dy;
                try{
                    temp.distance = distance_map.at( temp.y * x_size + temp.x);
                }catch(const std::out_of_range& oor){
                    continue;
                }
                // Se é o mesmo ponto, o ponto já foi catalogado ou não pertence ao mapa
                if( dx == 0 && dy == 0){
                    continue;
                } else if( temp.distance != std::numeric_limits<double>::infinity() ){
                    continue;
                } else if( temp.x < 0 || temp.x >= x_size || temp.y < 0 || temp.y > y_size){
                    continue;
                }

        //        ROS_INFO_STREAM( "(" << temp.x << "," << temp.y << ")" << " - VALUE " << (int)req.costmap.data.at(temp.x*y_size + temp.y));

                if( (int) req.costmap.data.at(temp.x * y_size + temp.y) == -1 && current.distance > 2 / resolution ) {
                    // YOU MAY CHECK IF THE UNKNOW SPACES HAS A NEIGHBORHOOD OF MORE UNKONW SPACE OR IS A OBSTACLE TO PREVENT OSCILLATION IN SINGLE CELLS
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    res.is_finished = false;
                    res.goal.pose.position.x = temp.x * resolution + origin.position.x ;
                    res.goal.pose.position.y = temp.y * resolution + origin.position.y ;
                    res.goal.header.frame_id = req.odom.header.frame_id;
                    res.goal.header.stamp = ros::Time::now();
                    res.goal.pose.orientation.w = 1;
                    res.goal.pose.orientation.x = 0;
                    res.goal.pose.orientation.y = 0;
                    res.goal.pose.orientation.z = 0;

                    ROS_INFO_STREAM("******  GOAL SENT ****** ");
                    ROS_INFO_STREAM( "WORLD: (" << res.goal.pose.position.x << "," << res.goal.pose.position.y <<")");
                    ROS_INFO_STREAM( "MAP: (" << temp.x << "," << temp.y <<")" );
                    return true;

                } else if ( (int) req.costmap.data.at(temp.x * y_size + temp.y)  < 5) {
                    if( temp.distance > current.distance + std::sqrt( (double) dx*dx + dy*dy)){
                        temp.distance =  current.distance + std::sqrt((double) dx*dx + dy*dy) ;
                        distance_map.at(temp.y*x_size + temp.x) = temp.distance;
                        to_visit.push_back(temp);
                    }
                }
            }
        }
    }


    ROS_ERROR( "NO GOAL SENT");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_goal");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_goal_from_map", get_goal);
  ROS_INFO("Ready to get Goals.");
  ros::spin();

  return 0;
}


/*
 * Name: compare_points
 *
 * Input: the point A is a struct point to be compared
 *        the point B is a struct point to be compared
 *
 * Output: true if pointA less_then point B, false otherwise
 *
 * Description:
 * To be used with list.sort
 * */
bool less_then( point &pointA, point &pointB ){
    if (pointA.distance < pointB.distance)  return true;
    return false;
}

bool greater_then( point &pointA, point &pointB ){
    if( pointA.distance > pointB.distance) return true ;
    return false;
}
