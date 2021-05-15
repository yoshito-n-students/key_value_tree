#include <string>

#include <key_value_tree/FlattenTree.h>
#include <key_value_tree/tree.hpp>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/bind.hpp>

#include <gtest/gtest.h>

using namespace key_value_tree;

/////////////////
// Test protocol
/////////////////

template <typename Message> void runTest(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  // load test tree & expected value from param
  XmlRpc::XmlRpcValue tree_params;
  ASSERT_TRUE(pnh.getParam("tree", tree_params));
  Tree tree;
  ASSERT_NO_THROW(tree = Tree::fromXmlRpcValue(tree_params));
  decltype(Message::data) expected_value;
  ASSERT_TRUE(pnh.getParam("expected_value", expected_value));

  // setup test node with own callback queue
  ros::CallbackQueue cb_queue;
  nh.setCallbackQueue(&cb_queue);

  // periodic tree publisher
  const ros::Publisher pub = nh.advertise<FlattenTree>("flatten_tree", 1);
  void (ros::Publisher::*const pub_func)(const FlattenTree &) const =
      &ros::Publisher::publish<FlattenTree>;
  const ros::Timer pub_timer = nh.createTimer(
      ros::Duration(0.5), boost::bind(pub_func, &pub, tree.flatten(ros::Time::now())));

  // subscriber just storing received values
  using MessagePtr = typename Message::Ptr;
  MessagePtr msg;
  MessagePtr &(MessagePtr::*const copy_msg_func)(const MessagePtr &) = &MessagePtr::operator=;
  const ros::Subscriber sub = nh.subscribe<Message>(
      "value", 1, boost::function<void(const MessagePtr &)>(boost::bind(copy_msg_func, &msg, _1)));

  // receive the first value
  while (nh.ok() && !msg) {
    cb_queue.callOne(ros::WallDuration(0.1));
  }

  // received == expected ??
  ASSERT_TRUE(msg);
  ASSERT_EQ(expected_value, msg->data);
}

/////////
// Tests
/////////

TEST(ExtractValue, int32) {
  ros::NodeHandle nh("int32_test"), pnh("~int32_test");
  runTest<std_msgs::Int32>(nh, pnh);
}

TEST(ExtractValue, float64) {
  ros::NodeHandle nh("float64_test"), pnh("~float64_test");
  runTest<std_msgs::Float64>(nh, pnh);
}

TEST(ExtractValue, string) {
  ros::NodeHandle nh("string_test"), pnh("~string_test");
  runTest<std_msgs::String>(nh, pnh);
}

////////
// Main
////////

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "rostest_extract_value");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}