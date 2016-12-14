#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <map>
#include <list>

/** include ros libraries**********************/
#include <ros/ros.h>
#include "mapping/get_goal_from_map.h"

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>



struct point{
    int x;
    int y;
    double distance;
};

int OBSTACLE_LIMIT = 5;
double DISTANCE_TO_OBSTACLE = 0.1;

bool operator==( point A, point B) {
    return  A.x == B.x && A.y == B.y ?  true : false ;
}


bool less_then( point &pointA, point &pointB );
bool greater_then( point &pointA, point &pointB );
bool isFreeWithin( int x, int y, nav_msgs::OccupancyGrid costmap, int distance_free);
bool isNotGhost( int mx, int my, nav_msgs::OccupancyGrid costmap, int distance_free);
int get_index( int mx, int my);
void write_costmap( nav_msgs::OccupancyGrid);

double x_size=0,y_size=0,resolution=0;

bool get_goal(mapping::get_goal_from_map::Request  &req, mapping::get_goal_from_map::Response &res)
{
    system( "clear");

    // Get all the information about the map
    y_size = req.costmap.info.height;              // cells
    x_size = req.costmap.info.width;               // cells
    resolution = req.costmap.info.resolution;      // meters / cell
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
         distance_map.emplace( get_index(ix,iy), std::numeric_limits<double>::infinity() );
        }
    }

    distance_map.at( get_index(m_c_odom.x,m_c_odom.y)) = 0;

    to_visit.push_back( m_c_odom );

    to_visit.sort(less_then);
   // Start digging for unknow points
    while(to_visit.empty() == false){
        point current = to_visit.front();

        if ( current.distance == std::numeric_limits<double>::infinity()){
            to_visit.sort(less_then); // just sort the list when there is no more good points to use
            current = to_visit.front();
            if ( current.distance == std::numeric_limits<double>::infinity()){
                res.is_finished = true;
                return true;
            }
        }
        to_visit.pop_front();

        //      ROS_INFO_STREAM("CURRENT: (" << current.x <<"," << current.y << ") - DISTANCE " << current.distance  << " - VALUE " << (int)req.costmap.data.at(current.y *x_size + current.x));
        //      ROS_INFO_STREAM("DISTANCE " << current.distance);

        for( register int dx = - 1 ; dx <=1 ; dx++){
            for( register int dy = -1; dy <=1 ; dy++){
                point temp;
                temp.x = current.x + dx;
                temp.y = current.y + dy;
                double index = get_index(temp.x,temp.y);

                try{
                    temp.distance = distance_map.at( get_index( temp.x, temp.y));
                }catch(const std::out_of_range& oor){
                    continue;
                }

                // Se é o mesmo ponto, o ponto já foi catalogado ou não pertence ao mapa
                if( dx == 0 && dy == 0){
                    continue;
                }
                if( temp.distance != std::numeric_limits<double>::infinity() ){
                    continue;
                }
                if( temp.x < 0 || temp.x >= x_size || temp.y < 0 || temp.y > y_size){
                    ROS_INFO("DESNECESSARIO ?");
                    continue;
                }

                // ROS_INFO_STREAM( "(" << temp.x << "," << temp.y << ")" << " - VALUE " << (int)req.costmap.data.at(index));

                if( (int) req.costmap.data.at(index) == -1 && current.distance > 2 / resolution ) {
                        // At least one is a unknown and the and none is an obstacle
                        if ( isFreeWithin( temp.x, temp.y , req.costmap , DISTANCE_TO_OBSTACLE / resolution ) && isNotGhost( temp.x, temp.y ,req.costmap,1) ){
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
                            ROS_INFO_STREAM( "VALUE: " << (int) req.costmap.data.at(index));
                            return true;

                        } else {
                            temp.distance =  current.distance + std::sqrt((double) dx*dx + dy*dy) ;
                            distance_map.at(index) = temp.distance;
                            to_visit.push_back(temp);
                       }
                } else if ( (int) req.costmap.data.at(index)  == 0 ) {
                        temp.distance =  current.distance + std::sqrt((double) dx*dx + dy*dy) ;
                        distance_map.at(index) = temp.distance;
                        to_visit.push_front(temp);
               }

            }
        }
    }

    ROS_ERROR( " ******* MAP FINISHED ******* ");
    res.is_finished = true;
///    ros::Duration stop(1);
//    stop.sleep();
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_goal");
  ros::NodeHandle n;

  ros::ServiceServer serrvice = n.advertiseService("get_goal_from_map", get_goal);

  n.param( "obstacle_distance", DISTANCE_TO_OBSTACLE, 0.1);

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

bool isNotGhost( int mx, int my, nav_msgs::OccupancyGrid costmap, int distance_free){
    int contador = 0;
    for( int dx = - distance_free ; dx <= distance_free ; dx++ ){
        for ( int dy = -(distance_free - abs(dx) ) ; dy <= (distance_free- abs(dx) ) ; dy ++){
            if( dx == 0 && dy == 0){
            } else if ( costmap.data.at( dx + mx + (dy+my)* costmap.info.width ) == 0 ) {
                //ROS_INFO_STREAM("POINT: (" << x+dx << "," << y+dx << ")" );
                contador ++;
            }
        }
    }
    if( contador >= 2) return true;
//    ROS_INFO("IT'S A GHOST");
    return false;

}

bool isFreeWithin( int mx, int my , nav_msgs::OccupancyGrid costmap, int distance_free){
    for( int x = - distance_free ; x < distance_free; x++){
        for( int y = -(distance_free - abs(x) ) ; y < (distance_free- abs(x) ) ; y ++){
            if ( (int) costmap.data.at(get_index(mx+x, my+y)) > OBSTACLE_LIMIT ) {
//                  ROS_INFO("CLOSE TO OBSTACLE");
                return false;
            }
        }
    }
    return true;
}
int get_index( int mx, int my){
    // return mx * y_size + my ;
    return my * x_size + mx ;
}

/*
void write_costmap( nav_msgs::OccupancyGrid){
    std::ofstream ExcelFile("Costmap_with_index.xlsx", std::ios::trunc);
    ROS_INFO( "WRITING COSTMAP");
    for( int index = 1; index <= y_size * x_size ; index++){
        ExcelFile << (int)costmap.data[index - 1] << '\t';
        if( index % x_size == 0 ) ExcelFile << std::endl;
    }

    ROS_INFO( "COSTMAP WROTE");
}
*/
