#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "bt_button_tutorial/wait.hpp"
#include "bt_button_tutorial/wait_for_button.hpp"
#include "bt_button_tutorial/print.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS 2. This must happen before creating any ROS node.
  rclcpp::init(argc, argv);

  // This ROS node is used by the behavior tree's ROS action client.
  auto node = std::make_shared<rclcpp::Node>("button_tree");

  // The factory connects the node names used in the XML file (such as
  // "Wait") to the C++ classes that implement those nodes.
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<Wait>("Wait");

  // WaitForButton is a ROS-aware BT node. These parameters give it the ROS
  // node and the default action server name. The server itself is Python.
  BT::RosNodeParams params(node, "/tutorial/wait_for_button");
  factory.registerNodeType<WaitForButton>("WaitForButton", params);

  factory.registerNodeType<Print>("Print");

  // Find the installed package directory and build the tree described by XML.
  // The XML decides the order and branching; the C++ classes provide behavior.
  auto tree = factory.createTreeFromFile(
    ament_index_cpp::get_package_share_directory("bt_button_tutorial") + "/behavior_trees/button.xml");

  // Tick the tree at 50 Hz while it is RUNNING. WallRate uses real clock time
  // and therefore does not require a simulator or a /clock topic.
  rclcpp::WallRate rate(50);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  try {
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      // Process pending ROS callbacks without blocking this loop.
      rclcpp::spin_some(node);

      // One tick lets the tree advance. Long-running nodes return RUNNING,
      // so this loop will tick them again until they return SUCCESS or FAILURE.
      status = tree.tickOnce();
      rate.sleep();
    }
  } catch (const std::exception& error) {
    // For example, this handles a missing/unreachable Python action server.
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    tree.haltTree();
    rclcpp::shutdown();
    return 1;
  }

  // Stop any node still running and release ROS resources cleanly.
  tree.haltTree();
  rclcpp::shutdown();

  // Return a normal process exit code based on the final tree result.
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
