#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <map>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace Location {
  enum class ID {UNKNOWN = 0, HOME, PICKUP, DROPOFF, DEFAULT = UNKNOWN};
  const std::map<ID, std::string> ID2NAME = {
    {ID::UNKNOWN, "UNKNOWN"},
    {ID::HOME,    "HOME"   }, 
    {ID::PICKUP,  "PICKUP" },
    {ID::DROPOFF, "DROPOFF"}
  };
  const std::map<std::string, ID> NAME2ID = {
    {"UNKNOWN", ID::UNKNOWN},
    {"HOME",    ID::HOME   },
    {"PICKUP",  ID::PICKUP },
    {"DROPOFF", ID::DROPOFF},
  };
}

class LocationPublisher {
public:
  LocationPublisher(
    const std::string &location_pub_topic = "/location_home_service", 
    const int &queue_size = 100) {

    location_pub_ = node_.advertise<std_msgs::String>(
      location_pub_topic, queue_size);
    PublishLocation(Location::ID::HOME);

  }
  void PublishLocation(const Location::ID &location) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << Location::ID2NAME.find(location)->second;
    msg.data = ss.str();
    location_pub_.publish(msg);
    ROS_INFO("Published current location: %s", msg.data.c_str());
  }
private:
  // ROS node
  ros::NodeHandle node_;
  // ROS Publisher 
  ros::Publisher location_pub_;
};

int main(int argc, char** argv) {

  // Initialize pick_objects node
  ros::init(argc, argv, "pick_objects");
  MoveBaseClient mbc("move_base", true);
  while(!mbc.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  } 
  // Goal pose 
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  LocationPublisher lp;

  // Pick up location 
  goal.target_pose.pose.position.x = -4.60;
  goal.target_pose.pose.position.y = -6.60;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.40;
  goal.target_pose.pose.orientation.w = 0.90;
  mbc.sendGoal(goal);
  ROS_INFO("Sent robot goal"); 
  mbc.waitForResult();
  if(mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    lp.PublishLocation(Location::ID::PICKUP);
  } else {
    lp.PublishLocation(Location::ID::UNKNOWN);
  }
  
  // Wait for object to be picked 
  ROS_INFO("Picking object");
  ros::Duration(5, 0).sleep();

  // Drop-off location 
  goal.target_pose.pose.position.x = 3.96;
  goal.target_pose.pose.position.y = -2.00;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.01;
  goal.target_pose.pose.orientation.w = 1.0;
  mbc.sendGoal(goal);
  mbc.waitForResult();
  
  // Check if the robot reached its goal
  if(mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    lp.PublishLocation(Location::ID::DROPOFF);
  } else {
    lp.PublishLocation(Location::ID::UNKNOWN);
  }

  // Home location 
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  mbc.sendGoal(goal);
  mbc.waitForResult();
  if(mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    lp.PublishLocation(Location::ID::HOME);
  } else {
    lp.PublishLocation(Location::ID::UNKNOWN);
  }
  return 0;
}