#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

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

namespace Action {
  enum class ID {UNKNOWN = 0, START, CARRY, FINISH, DEFAULT = UNKNOWN};
  const std::map<ID, std::string> ID2NAME = {
    {ID::UNKNOWN, "UNKNOWN"},
    {ID::START,   "START"  }, 
    {ID::CARRY,   "CARRY"  },
    {ID::FINISH,   "FINISH"  },
  };
  const std::map<std::string, ID> NAME2ID = {
    {"UNKNOWN", ID::UNKNOWN },
    {"START",   ID::START   },
    {"CARRY",   ID::CARRY   },
    {"FINISH",   ID::FINISH },
  };
}

class MarkerPublisher {
public:
  MarkerPublisher(
    const std::string &location_sub_topic = "/location_home_service", 
    const std::string &marker_pub_topic = "visualization_marker",
    const std::string &marker_namespace = "add_markers",
    const int &queue_size = 100) 
  : marker_namespace_(marker_namespace) {
    // Initialize subscriber
    location_sub_ = node_.subscribe<std_msgs::String>(
      location_sub_topic, queue_size, &MarkerPublisher::LocationCallback, this);

    // Initialize publisher
    marker_pub_ = node_.advertise<visualization_msgs::Marker>(
      marker_pub_topic, queue_size);
  }
  bool Initialize() {
    // Wait to get a subscriber to the markers 
    while (marker_pub_.getNumSubscribers() < 1) {
      if(!ros::ok()) {
        return false;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    // Initialize marker
    marker_.header.stamp = ros::Time::now();
    marker_.ns = marker_namespace_;
    marker_.id = 0;
    marker_.lifetime = ros::Duration();
    return true;
  }

  void SetFrameId(const std::string &frame_id) {
    marker_.header.frame_id = frame_id;
  }
  void SetType(const uint32_t &type) {
     marker_.type = type;
  }
  void SetScale(const float &x, const float &y, const float &z) {
    marker_.scale.x = x;
    marker_.scale.y = y;
    marker_.scale.z = z;
  }
  void SetColor(const float &r, const float &g, const float &b, const float &a) {
    marker_.color.r = r;
    marker_.color.g = g;
    marker_.color.b = b;
    marker_.color.a = a;
  }

  void SetStartPose(const geometry_msgs::Pose &start_pose) {
    start_pose_ = start_pose;
  }

  void SetFinishPose(const geometry_msgs::Pose &finish_pose) {
    finish_pose_ = finish_pose;
  }

  bool SetAction(const Action::ID &action) {
    switch(action) {
      case Action::ID::START:
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.pose = start_pose_;
        break;
      case Action::ID::FINISH:
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.pose = finish_pose_;
        break;
      case Action::ID::CARRY:
        marker_.action = visualization_msgs::Marker::DELETE;
        break;
      default:
        ROS_ERROR("Unknown action");
        return false;
    }
    return true;
  }

  void PublishMarker() {
    marker_pub_.publish(marker_);
    ROS_INFO("Published marker");
  }

  void LocationCallback(const std_msgs::String msg) {

    Location::ID location = Location::NAME2ID.find(msg.data.c_str())->second;

    switch(location) {
      case Location::ID::HOME:
        ROS_INFO("Robot reached home");
        break;
      case Location::ID::PICKUP:
        SetAction(Action::ID::CARRY);
        PublishMarker();
        break;
      case Location::ID::DROPOFF:
        SetAction(Action::ID::FINISH);
        PublishMarker();
        break;
      default:
        ROS_WARN("Unknown location");
    }

    ROS_INFO("Heard %s", msg.data.c_str());
  }

private:
  // ROS node
  ros::NodeHandle node_;
  // ROS location subscriber 
  ros::Subscriber location_sub_;
  // ROS marker publisher 
  ros::Publisher marker_pub_;
  // Marker to publish
  visualization_msgs::Marker marker_;
  // Namespace
  std::string marker_namespace_ = "add_markers";
  // Start pose
  geometry_msgs::Pose start_pose_;
  // Goal pose 
  geometry_msgs::Pose finish_pose_;
};


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  MarkerPublisher mp; 
  if(!mp.Initialize()) {
    ROS_ERROR("Could not initialize marker publisher");
    return EXIT_FAILURE;
  }
  mp.SetFrameId("map");
  mp.SetType(visualization_msgs::Marker::CUBE);
  mp.SetScale(0.2, 0.2, 0.2);
  mp.SetColor(1.0, 0.2, 1.0, 1.0);

  geometry_msgs::Pose pose;
  pose.position.x =  6.60;
  pose.position.y = -4.20;
  pose.position.z = 0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.90;
  pose.orientation.w = 0.40;
  mp.SetStartPose(pose);
  mp.SetAction(Action::ID::START);
  mp.PublishMarker();

  pose.position.x = -3.96;
  pose.position.y = -4.00;
  pose.position.z = 0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.72;
  pose.orientation.w = 0.68;
  mp.SetFinishPose(pose);

  ros::spin();
}