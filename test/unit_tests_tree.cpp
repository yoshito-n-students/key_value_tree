#include <iostream>
#include <string>

#include <key_value_tree/tree.hpp>

#include <gtest/gtest.h>

using namespace key_value_tree;

TEST(Tree, creation) {
  Tree tree;
  // as an int32 leaf
  tree = 42;
  ASSERT_TRUE(tree.root().isInt32Leaf());
  ASSERT_EQ(42, tree.root().asInt32Leaf());
  // as a float64 leaf
  tree = 42.;
  ASSERT_TRUE(tree.root().isFloat64Leaf());
  ASSERT_DOUBLE_EQ(42., tree.root().asFloat64Leaf());
  // as a string leaf
  tree = std::string("42");
  ASSERT_TRUE(tree.root().isStringLeaf());
  ASSERT_EQ("42", tree.root().asStringLeaf());
  // as a string leaf 2
  tree = "Answer to the Ultimate Question of Life, the Universe, and Everything";
  ASSERT_TRUE(tree.root().isStringLeaf());
  ASSERT_EQ("Answer to the Ultimate Question of Life, the Universe, and Everything",
            tree.root().asStringLeaf());
  // as a complex tree
  using P = Tree::Parent;
  tree = P{{"number",             //
            P{{"int32", 42},      //
              {"float64", 42.}}}, //
           {"string", "42"}};
  ASSERT_TRUE(tree.root().isParent());
  ASSERT_DOUBLE_EQ(42., tree.root().asParent()[0].node.asParent()[1].node.asFloat64Leaf());
}

TEST(Tree, comparison) {
  // vs int32 tree
  ASSERT_EQ(Tree(42), Tree(42));
  ASSERT_NE(Tree(-42), Tree(42));
  ASSERT_NE(Tree(42.), Tree(42));
  ASSERT_NE(Tree("42"), Tree(42));
  // vs float64 tree
  ASSERT_EQ(Tree(42.), Tree(42.));
  ASSERT_NE(Tree(-42.), Tree(42.));
  ASSERT_NE(Tree("42."), Tree(42.));
  // vs string tree
  ASSERT_EQ(Tree("42"), Tree("42"));
  ASSERT_NE(Tree("-42"), Tree("42"));
  // vs complex tree
  using P = Tree::Parent;
  ASSERT_EQ((Tree{P{{"number",                //
                     P{{"int32", 42},         //
                       {"float64", 42.}}}}}), //
            (Tree{P{{"number",                //
                     P{{"int32", 42},         //
                       {"float64", 42.}}}}}));
  ASSERT_NE((Tree{P{{"number",                //
                     P{{"int32", 42},         //
                       {"float64", 42.}}}}}), //
            (Tree{P{{"number",                //
                     P{{"int32", 42},         //
                       {"float64", -42.}}}}}));
}

TEST(Tree, print) {
  using P = Tree::Parent;
  const Tree tree = P{{"number",             //
                       P{{"int32", 42},      //
                         {"float64", 42.}}}, //
                      {"string", "42"}};
  ASSERT_NO_THROW(std::cout << tree << std::endl);
}