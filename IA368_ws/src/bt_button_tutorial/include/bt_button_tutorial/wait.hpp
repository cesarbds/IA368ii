#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>
#include <chrono>

// A stateful BT action that waits without blocking the tree's tick loop.
class Wait : public BT::StatefulActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfig& config);

  // Declares the XML input: <Wait seconds="5"/>.
  static BT::PortsList providedPorts();

  // Called once when this node changes from IDLE to RUNNING.
  BT::NodeStatus onStart() override;

  // Called on later tree ticks while this node is RUNNING.
  BT::NodeStatus onRunning() override;

  // Called if a parent stops this node before it completes.
  void onHalted() override;

private:
  // Absolute time at which the wait is complete.
  std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<double>> deadline_;
};
