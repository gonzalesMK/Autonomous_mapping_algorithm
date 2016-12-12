#include <stdio.h>
/** include ros libraries**********************/
#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "mapping/get_goal_from_map.h"


#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <costmap_2d/costmap_2d.h>

#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


/** GLOBAL VARIABLES ********************************************/

double distance_to_goal;
nav_msgs::OccupancyGrid my_costmap;
geometry_msgs::PoseStamped goal_pose;
double MAX_ERROR = 0.8;

/** DECLARATIONS ********************************************/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleClientGoalState::StateEnum states;

geometry_msgs::PoseStamped getPose(tf::TransformListener& listener,std::string frame);


/** ************************************************/



void get_costmap( const nav_msgs::OccupancyGrid &costmap ){
    my_costmap = costmap;
}

/*
 * Actually, move_base never sends that the goal is completed
 * */
void done_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result){
     if (state == state.SUCCEEDED){
        ROS_INFO( " GOAL IS DONE ");
     }else{
         ROS_ERROR_STREAM( state.getText() );
      }

     distance_to_goal = 0;
}


void active_cb(){
    ROS_INFO( " Sent Goal!!");
}

/*
 * The feedback in the action lib returns the position of the robot
 */

void feedback_cb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
    double dx = feedback.get()->base_position.pose.position.x - goal_pose.pose.position.x;
    double dy = feedback.get()->base_position.pose.position.y - goal_pose.pose.position.y;
    distance_to_goal = std::sqrt( dx*dx + dy*dy);
    system("clear");
    ROS_INFO_STREAM( " GOAL: (" <<goal_pose.pose.position.x << "," << goal_pose.pose.position.y <<") | POSE: (" <<
                       feedback.get()->base_position.pose.position.x << "," << feedback.get()->base_position.pose.position.y <<
                       ") | Distance: " << distance_to_goal << std::endl);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "automapping");
  ros::NodeHandle node;

  // Clients for the services
  ros::ServiceClient client_slam =  node.serviceClient<nav_msgs::GetMap>("dynamic_map");
  ros::ServiceClient client_goal =  node.serviceClient<mapping::get_goal_from_map>("get_goal_from_map");

  ros::Subscriber global_costmap_sub = node.subscribe( "/move_base/global_costmap/costmap",2, get_costmap);
  MoveBaseClient client_move_base("move_base",true);

  // Initializing client's services
  while(!client_slam.waitForExistence(ros::Duration(5.0))){
    ROS_INFO("Waiting for slam_server to come up");
  }


  while(!client_goal.waitForExistence(ros::Duration(5.0))){
    ROS_INFO("Waiting get_goal_from_map server to come up");
  }

  while(!client_move_base.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Services msgs
  move_base_msgs::MoveBaseGoal action_goal;
  nav_msgs::GetMap srv_slam;
  mapping::get_goal_from_map srv_goal;

  // Tf
  tf::TransformListener listener;
  std::string robot_frame("AMR");

  ros::Rate rate(1);
  while (ros::ok())
  {
      rate.sleep();
      ros::spinOnce();
      // 1º - Pegar o Mapa
      if (!client_slam.call(srv_slam))
      {
        ROS_ERROR("FAIL TO GET ODOMETRY");
      }
      else
      {
        ROS_INFO("GOT ODOMETRY");
      }
      // 2º  Encontrar uma Goal
      srv_goal.request.costmap = my_costmap;
      srv_goal.request.odom = getPose( listener, robot_frame );

      if (!client_goal.call(srv_goal))
      {
        ROS_ERROR("FAIL TO GET GOAL");
        continue;
      }

      ROS_INFO_STREAM("NEW GOAL: (" << srv_goal.response.goal.pose.position.x << ", " << srv_goal.response.goal.pose.position.y << ")");

      if( srv_goal.response.is_finished == false){
          goal_pose = srv_goal.response.goal ;
      } else {
          ROS_INFO_STREAM ( " MAP IS FINISHED ");
          return 1;
      }

     // 3 - Atingir a Goal
      distance_to_goal = std::numeric_limits<double>::infinity();

      action_goal.target_pose = goal_pose;

      client_move_base.sendGoal( action_goal, done_cb ,active_cb, feedback_cb);

      while( distance_to_goal > MAX_ERROR && ros::ok() ){
          ros::spinOnce();
          rate.sleep();
      }

      ROS_INFO_STREAM("******************" << std::endl);

  }



}



/*
 * Name: getPose
 *
 * Input:  listener is a tf::TransformListener object linked to roscore,
 *         frame is a std::string with the name of the target frame
 *
 * Output: the pose of the robot as a geometry_msgs::Pose2D
 *
 * Description:
 * It makes the conversion between the types following the ROS documentation
 * */
geometry_msgs::PoseStamped getPose(tf::TransformListener& listener,std::string frame)
{
  tf::StampedTransform stamped_transform;
  listener.waitForTransform("/map", frame, ros::Time(0), ros::Duration(10.0));
  try{
    // Get pose
    listener.lookupTransform("/map", frame, ros::Time(0), stamped_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = stamped_transform.getOrigin().getX();
  pose.pose.position.y = stamped_transform.getOrigin().getY();
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/map";
  pose.pose.orientation.w = 1;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.z = 0;
  return pose;
}
