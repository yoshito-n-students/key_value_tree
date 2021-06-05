#include <cstdint>
#include <string>
#include <vector>

#include <key_value_tree/FlattenTree.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/range/algorithm/find.hpp>

namespace kvt = key_value_tree;

ros::NodeHandlePtr nh;
std::string path;
ros::Publisher value_pub;
std::string advertised_type;

template <typename Value> struct MessageOf;
template <> struct MessageOf<std::int32_t> { using Type = std_msgs::Int32; };
template <> struct MessageOf<double> { using Type = std_msgs::Float64; };
template <> struct MessageOf<std::string> { using Type = std_msgs::String; };

template <typename Value>
void publishValue(const std::vector<std::string> &paths, const std::vector<Value> &values) {
  using Message = typename MessageOf<Value>::Type;
  static const std::string value_type = ros::message_traits::datatype<Message>();

  // search keys with the query key
  const std::size_t id = boost::find(paths, path) - paths.begin();
  if (id < 0 || id >= paths.size()) {
    return;
  }

  // setup the publisher if never
  if (!value_pub) {
    value_pub = nh->advertise<Message>("value", 1);
    advertised_type = value_type;
  }
  if (advertised_type != value_type) {
    ROS_ERROR_STREAM_ONCE("Found a value of '" << value_type << "' with the path '" << path << "'. "
                                               << "But the publisher has already advertised with '"
                                               << advertised_type << "'. "
                                               << "Skip publishing.");
    return;
  }

  // publish the value corresponding to the found key
  Message msg;
  msg.data = values[id];
  value_pub.publish(msg);
}

void onFlattenTreeReceived(const kvt::FlattenTreeConstPtr &flatten_tree) {
  publishValue(flatten_tree->int32_paths, flatten_tree->int32_values);
  publishValue(flatten_tree->float64_paths, flatten_tree->float64_values);
  publishValue(flatten_tree->string_paths, flatten_tree->string_values);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "extract_value");
  nh.reset(new ros::NodeHandle());

  path = ros::param::param<std::string>("~path", "");

  const ros::Subscriber flatten_tree_sub = nh->subscribe("flatten_tree", 1, onFlattenTreeReceived);

  ros::spin();
  
  return 0;
}