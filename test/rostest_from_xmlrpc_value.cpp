#include <key_value_tree/tree.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <gtest/gtest.h>

using namespace key_value_tree;

TEST(FromXmlRpcValue, valid) {
  // set params corresponding a valid tree
  ros::param::set("~valid_tree/number/int32/positive", 42);
  ros::param::set("~valid_tree/number/int32/negative", -42);
  ros::param::set("~valid_tree/number/float64/positive", 42.);
  ros::param::set("~valid_tree/number/float64/negative", -42.);
  ros::param::set("~valid_tree/string/positive", "42");
  ros::param::set("~valid_tree/string/negative", "-42");

  // load params
  XmlRpc::XmlRpcValue params;
  ASSERT_TRUE(ros::param::get("~valid_tree", params));

  // tree from params
  Tree tree;
  ASSERT_NO_THROW(tree = Tree::fromXmlRpcValue(params));

  // check tree contents
  using P = Tree::Parent;
  const Tree expected_tree = P{{"number",                  //
                                P{{"float64",              //
                                   P{{"negative", -42.},   //
                                     {"positive", 42.}}},  //
                                  {"int32",                //
                                   P{{"negative", -42},    //
                                     {"positive", 42}}}}}, //
                               {"string",                  //
                                P{{"negative", "-42"},     //
                                  {"positive", "42"}}}};
  ASSERT_EQ(expected_tree, tree);
}

TEST(FromXmlRpcValue, invalid) {
  // set params corresponding a valid tree
  ros::param::set("~invalid_tree/boolean/true", true);
  ros::param::set("~invalid_tree/boolean/false", false);
  ros::param::set("~invalid_tree/int32/positive", 42);
  ros::param::set("~invalid_tree/int32/negative", -42);
  ros::param::set("~invalid_tree/string/positive", "42");
  ros::param::set("~invalid_tree/string/negative", "-42");

  // load params
  XmlRpc::XmlRpcValue params;
  ASSERT_TRUE(ros::param::get("~invalid_tree", params));

  // tree from params
  Tree tree;
  ASSERT_THROW(tree = Tree::fromXmlRpcValue(params), std::runtime_error);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "rostest_from_xmlrpc_value");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}