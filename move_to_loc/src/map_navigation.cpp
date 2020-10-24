
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::PoseStamped goalPose;

float x_dist=0;
float y_dist=0;
  double r,p,y; 
  
float x_cur_pose=0;
float y_cur_pose=0;

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   
   //goalPose=msg;
   goalPose.header.seq=msg->header.seq;
   goalPose.header.frame_id=msg->header.frame_id;
   goalPose.header.stamp=msg->header.stamp;
   goalPose.pose.position.x=msg->pose.position.x;
   goalPose.pose.position.y=msg->pose.position.y;
   goalPose.pose.position.z=msg->pose.position.z;
   goalPose.pose.orientation.x=msg->pose.orientation.x;
   goalPose.pose.orientation.y=msg->pose.orientation.y;
   goalPose.pose.orientation.z=msg->pose.orientation.z;
   goalPose.pose.orientation.w=msg->pose.orientation.w;

   tf2::Quaternion q_orig, q_rot, q_new;

// Get the original orientation of 'commanded_pose'
tf2::convert(goalPose.pose.orientation , q_orig);

double r=0, p=0, y=3.14159;  // Rotate the previous pose by 180* about X
q_rot.setRPY(r, p, y);

q_new = q_rot*q_orig;  // Calculate the new orientation
q_new.normalize();

// Stuff the new rotation back into the pose. This requires conversion into a msg type
tf2::convert(q_new, goalPose.pose.orientation);

    
  //  tf::Quaternion q(
  //  msg->pose.orientation.x,
  //  msg->pose.orientation.y,
  //  msg->pose.orientation.z,
  //  msg->pose.orientation.w);
  //  tf::Matrix3x3 m(q);
 
  //  m.getRPY(r,p,y);
  //  ROS_INFO("Roll=%f Pitch=%f Yaw=%f",r,p,y); 
  // //  y=y+3.14159265358979323;
  //  tf::Quaternion quat=tf::createQuaternionFromRPY(r,p,y);
  //  goalPose.pose.orientation.x=quat.x();
  //  goalPose.pose.orientation.y=quat.y();
  //  goalPose.pose.orientation.z=quat.z();
  //  goalPose.pose.orientation.w=quat.w();
   x_dist=1;
}


float round(float var) 
{ 
    float value = (int)(var * 100 + .5); 
    return (float)value / 100; 
} 
 
void initPoseCallback(const geometry_msgs::PoseStamped msg)
{
  //x_cur_pose=msg.pose.pose.position.x;
  //y_cur_pose=msg.pose.pose.position.y;
  // ROS_INFO("Cam_x= %f    Cam_z=%f \n",x_dist,y_dist);
}

void arucoCallback(const geometry_msgs::PointStamped& msg)
{
  x_dist=msg.point.x;
  y_dist=msg.point.z;
  // ROS_INFO("Cam_x= %f    Cam_z=%f \n",x_dist,y_dist);
}
int main(int argc, char** argv){
  ros::init(argc, argv, "map_navigation_node");
  ros::NodeHandle nh;
  //subscribe to Aruco Marker position topic
  
  ros::Subscriber aruco_sub= nh.subscribe("/VIPER/follow_aruco/target_position",100,arucoCallback);
  //ros::Subscriber init_pose_sub= nh.subscribe("/amcl_pose",100,initPoseCallback);
  ros::Subscriber goal_pose_sub= nh.subscribe("/aruco_single/pose",100,goalPoseCallback);

  while(x_dist==0){

  ros::spinOnce();

  }
  
  ROS_INFO("Map_x= %f    Map_y=%f \n",x_dist,y_dist);
  ROS_INFO("Goal position_x= %f    Goal_position_y=%f \n",round(y_dist+x_cur_pose-1.3),round(y_cur_pose-x_dist));
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goalPose.pose.position.x-1;
  goal.target_pose.pose.position.y = goalPose.pose.position.y;
  // goal.target_pose.pose.position.z = 0;
  // goal.target_pose.pose.orientation.x = 0;
  // goal.target_pose.pose.orientation.y = 0;
  //goal.target_pose.pose.orientation.z = goalPose.pose.orientation.z;
  goal.target_pose.pose.orientation.w = 1; //goalPose.pose.orientation.w;

  ROS_INFO("Moving to point A");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("I have reached at point A :)");
    ROS_INFO(" Current position is %f %f",y_cur_pose,x_cur_pose);           
  }
  else{
    ROS_INFO("I am unable to move to point A :(");

}
  return 0;
}


