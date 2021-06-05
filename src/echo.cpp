#include <key_value_tree/FlattenTree.h>
#include <key_value_tree/tree.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>

namespace kvt = key_value_tree;

void echo(const kvt::FlattenTreeConstPtr &msg) {
  ROS_INFO_STREAM("\n" << kvt::Tree::fromFlattenTree(*msg));
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "echo");
  ros::NodeHandle nh;
  const ros::Subscriber sub = nh.subscribe("flatten_tree", 1, echo);
  ros::spin();
  return 0;
}