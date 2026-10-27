#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <bt_button_interfaces/action/wait_for_button.hpp>

// Only the adapter lives in C++; the behavior lives in button_server.py.
class WaitForButton : public BT::RosActionNode<bt_button_interfaces::action::WaitForButton>
{
public:
  WaitForButton(const std::string& name, const BT::NodeConfig& config,
                const BT::RosNodeParams& params);

  // Adds "seconds" to the standard ROS action port "action_name".
  static BT::PortsList providedPorts();

  // Converts BT input ports into a ROS action goal.
  bool setGoal(Goal& goal) override;

  // Converts the Python action server's result into a BT status.
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;

  // Handles failures such as an unavailable or aborted action server.
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
