#include "bt_button_tutorial/wait.hpp"
#include <iostream>

using Clock = std::chrono::steady_clock;

Wait::Wait(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList Wait::providedPorts()
{
  return {BT::InputPort<double>("seconds")};
}

BT::NodeStatus Wait::onStart()
{
  // Read the value supplied by the XML input port.
  const double seconds = getInput<double>("seconds").value();

  // Store a deadline instead of sleeping here. Sleeping would block the whole
  // behavior tree and prevent other work from being processed.
  deadline_ = Clock::now() + std::chrono::duration<double>(seconds);
  std::cout << "Waiting " << seconds << " seconds..." << std::endl;

  // Tell the parent that this action has started but is not finished yet.
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::onRunning()
{
  // The tree calls this method repeatedly until the deadline is reached.
  if (Clock::now() >= deadline_) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

// There is no external operation to cancel for this simple timer.
void Wait::onHalted() {}
