#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "bt_button_tutorial/print.hpp"
#include "bt_button_tutorial/random_odd.hpp"
#include "bt_button_tutorial/wait.hpp"
#include "bt_button_tutorial/wait_for_button.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("random_button_tree");

  // Register every custom node referenced by button_random.xml.
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<Wait>("Wait");
  factory.registerNodeType<Print>("Print");

  BT::RosNodeParams button_params(node, "/tutorial/wait_for_button");
  factory.registerNodeType<WaitForButton>("WaitForButton", button_params);

  BT::RosNodeParams random_params(node, "/tutorial/random_odd");
  factory.registerNodeType<RandomOdd>("RandomOdd", random_params);

  auto tree = factory.createTreeFromFile(
    ament_index_cpp::get_package_share_directory("bt_button_tutorial") +
    "/behavior_trees/button_random.xml");

  rclcpp::WallRate rate(50);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  try {
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      rclcpp::spin_some(node);
      status = tree.tickOnce();
      rate.sleep();
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    tree.haltTree();
    rclcpp::shutdown();
    return 1;
  }

  tree.haltTree();
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
