#include <key_value_tree/FlattenTree.h>
#include <key_value_tree/tree.hpp>
#include <ros/time.h>

#include <gtest/gtest.h>

using namespace key_value_tree;

TEST(FlattenTree, fromTree) {
  using P = Tree::Parent;
  const Tree tree = P{{"number",             //
                       P{{"int32", 42},      //
                         {"float64", 42.}}}, //
                      {"string", "42"}};
  const ros::Time stamp = ros::Time(1234, 5678);
  const FlattenTree flatten_tree = tree.flatten(stamp);
  // stamp
  ASSERT_EQ(stamp, flatten_tree.header.stamp);
  // int32 leaf
  ASSERT_EQ(1, flatten_tree.int32_keys.size());
  ASSERT_EQ("number/int32", flatten_tree.int32_keys[0]);
  ASSERT_EQ(1, flatten_tree.int32_values.size());
  ASSERT_EQ(42, flatten_tree.int32_values[0]);
  // float64 leaf
  ASSERT_EQ(1, flatten_tree.float64_keys.size());
  ASSERT_EQ("number/float64", flatten_tree.float64_keys[0]);
  ASSERT_EQ(1, flatten_tree.float64_values.size());
  ASSERT_DOUBLE_EQ(42., flatten_tree.float64_values[0]);
  // string leaf
  ASSERT_EQ(1, flatten_tree.string_keys.size());
  ASSERT_EQ("string", flatten_tree.string_keys[0]);
  ASSERT_EQ(1, flatten_tree.string_values.size());
  ASSERT_EQ("42", flatten_tree.string_values[0]);
}

TEST(FlattenTree, toTree) {
  FlattenTree flatten_tree;
  flatten_tree.int32_keys = {"number/int32/positive", "number/int32/negative"};
  flatten_tree.int32_values = {42, -42};
  flatten_tree.float64_keys = {"number/float64/positive", "number/float64/negative"};
  flatten_tree.float64_values = {42., -42.};
  flatten_tree.string_keys = {"string/positive", "string/negative"};
  flatten_tree.string_values = {"42", "-42"};

  using P = Tree::Parent;
  ASSERT_EQ((Tree{P{{"number",                    //
                     P{{"int32",                  //
                        P{{"positive", 42},       //
                          {"negative", -42}}},    //
                       {"float64",                //
                        P{{"positive", 42.},      //
                          {"negative", -42.}}}}}, //
                    {"string",                    //
                     P{{"positive", "42"},        //
                       {"negative", "-42"}}}}}),
            Tree::fromFlattenTree(flatten_tree));
}