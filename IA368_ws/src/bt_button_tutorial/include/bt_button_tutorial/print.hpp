#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>

// A synchronous node: it prints once and completes during the same tick.
class Print : public BT::SyncActionNode
{
public:
  Print(const std::string& name, const BT::NodeConfig& config);

  // Declares the XML input: <Print message="Hello"/>.
  static BT::PortsList providedPorts();

  // SyncActionNode implements its complete operation in tick().
  BT::NodeStatus tick() override;
};
