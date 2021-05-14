#ifndef KEY_VALUE_TREE_TREE_HPP
#define KEY_VALUE_TREE_TREE_HPP

#include <cstdint>
#include <iostream>
#include <string>
#include <utility> // for std::forward()
#include <vector>

#include <key_value_tree/FlattenTree.h>
#include <ros/time.h>

#include <boost/operators.hpp>
#include <boost/range/algorithm/equal.hpp>
#include <boost/range/algorithm/find_if.hpp>
#include <boost/range/algorithm_ext/for_each.hpp>
#include <boost/tokenizer.hpp>
#include <boost/variant.hpp>

namespace key_value_tree {

///////////////////////////////////////////////////////
// Tree structure
//   - string keys for parent nodes
//   - integer/floating-point/string values for leaves
///////////////////////////////////////////////////////
class Tree : private boost::equality_comparable<Tree> /* supply != from == */ {
public:
  struct KeyNodePair;
  using Parent = std::vector<KeyNodePair>;
  using Int32Leaf = std::int32_t;
  using Float64Leaf = double;
  using StringLeaf = std::string;
  class Node : private boost::equality_comparable<Node> /* supply != from == */ {
  public:
    template <class... Args> Node(Args &&...args) : node_(std::forward<Args>(args)...) {}
    //
    bool isParent() const { return node_.which() == 0; }
    Parent &asParent() { return boost::get<Parent>(node_); }
    const Parent &asParent() const { return boost::get<Parent>(node_); }
    //
    bool isInt32Leaf() const { return node_.which() == 1; }
    Int32Leaf &asInt32Leaf() { return boost::get<Int32Leaf>(node_); }
    const Int32Leaf &asInt32Leaf() const { return boost::get<Int32Leaf>(node_); }
    //
    bool isFloat64Leaf() const { return node_.which() == 2; }
    Float64Leaf &asFloat64Leaf() { return boost::get<Float64Leaf>(node_); }
    const Float64Leaf &asFloat64Leaf() const { return boost::get<Float64Leaf>(node_); }
    //
    bool isStringLeaf() const { return node_.which() == 3; }
    StringLeaf &asStringLeaf() { return boost::get<StringLeaf>(node_); }
    const StringLeaf &asStringLeaf() const { return boost::get<StringLeaf>(node_); }
    //
    bool operator==(const Node &other) const {
      if (isParent() && other.isParent()) {
        return boost::equal(asParent(), other.asParent());
      } else if (isInt32Leaf() && other.isInt32Leaf()) {
        return asInt32Leaf() == other.asInt32Leaf();
      } else if (isFloat64Leaf() && other.isFloat64Leaf()) {
        return asFloat64Leaf() == other.asFloat64Leaf();
      } else if (isStringLeaf() && other.isStringLeaf()) {
        return asStringLeaf() == other.asStringLeaf();
      } else {
        return false;
      }
    }

  private:
    boost::variant<boost::recursive_wrapper<Parent>, Int32Leaf, Float64Leaf, StringLeaf> node_;
  };
  struct KeyNodePair : private boost::equality_comparable<KeyNodePair> /* supply != from == */ {
    KeyNodePair(const std::string &_key, const Node &_node) : key(_key), node(_node) {}
    std::string key;
    Node node;
    bool operator==(const KeyNodePair &other) const {
      return key == other.key && node == other.node;
    }
  };

public:
  template <class... Args> Tree(Args &&...args) : root_(std::forward<Args>(args)...) {}
  Node &root() { return root_; }
  const Node &root() const { return root_; }
  bool operator==(const Tree &other) const { return root() == other.root(); }

  FlattenTree flatten(const ros::Time &stamp) const {
    struct Impl {
      static void apply(FlattenTree *const flatten_tree, const Node &node,
                        const std::string &path = "") {
        if (node.isParent()) {
          for (const KeyNodePair &child : node.asParent()) {
            apply(flatten_tree, child.node, path.empty() ? child.key : (path + "/" + child.key));
          }
        } else if (node.isInt32Leaf()) {
          flatten_tree->int32_paths.push_back(path);
          flatten_tree->int32_values.push_back(node.asInt32Leaf());
        } else if (node.isFloat64Leaf()) {
          flatten_tree->float64_paths.push_back(path);
          flatten_tree->float64_values.push_back(node.asFloat64Leaf());
        } else if (node.isStringLeaf()) {
          flatten_tree->string_paths.push_back(path);
          flatten_tree->string_values.push_back(node.asStringLeaf());
        }
      }
    };

    FlattenTree flatten_tree;
    flatten_tree.header.stamp = stamp;
    Impl::apply(&flatten_tree, root());
    return flatten_tree;
  }

private:
  // helper class used in fromFlattenTree().
  // this class cannot be declared in fromFlattenTree() because this is a templated class.
  template <typename Value> class MergeTo {
  public:
    MergeTo(Tree *const tree) : tree_(tree) {}
    void operator()(const std::string &path, const Value &value) const {
      const boost::tokenizer<boost::char_separator<char>> keys(path,
                                                               boost::char_separator<char>(R"(/)"));
      Node *node = &tree_->root();
      for (const std::string &key : keys) {
        Parent *const parent = &node->asParent();
        Parent::iterator child =
            boost::find_if(*parent, [&key](const KeyNodePair &child) { return child.key == key; });
        if (child == parent->end()) {
          parent->push_back({key, Parent()});
          child = parent->end() - 1;
        }
        node = &child->node;
      }
      *node = value;
    }

  private:
    Tree *const tree_;
  };

public:
  static Tree fromFlattenTree(const FlattenTree &flatten_tree) {
    Tree tree = Parent();
    boost::for_each(flatten_tree.int32_paths, flatten_tree.int32_values, MergeTo<Int32Leaf>(&tree));
    boost::for_each(flatten_tree.float64_paths, flatten_tree.float64_values,
                    MergeTo<Float64Leaf>(&tree));
    boost::for_each(flatten_tree.string_paths, flatten_tree.string_values,
                    MergeTo<StringLeaf>(&tree));
    return tree;
  }

private:
  Node root_;
};

static inline std::ostream &operator<<(std::ostream &os, const Tree &tree) {
  struct Impl {
    static void apply(std::ostream &os, const Tree::Node &node, const unsigned int n_indent = 0) {
      const std::string indent(n_indent, ' ');
      if (node.isParent()) {
        for (const Tree::KeyNodePair &child : node.asParent()) {
          os << indent << child.key << ":" << std::endl;
          apply(os, child.node, n_indent + 2);
        }
      } else if (node.isInt32Leaf()) {
        os << indent << node.asInt32Leaf() << std::endl;
      } else if (node.isFloat64Leaf()) {
        os << indent << node.asFloat64Leaf() << std::endl;
      } else if (node.isStringLeaf()) {
        os << indent << node.asStringLeaf() << std::endl;
      }
    }
  };

  Impl::apply(os, tree.root());
  return os;
}
} // namespace key_value_tree

#endif