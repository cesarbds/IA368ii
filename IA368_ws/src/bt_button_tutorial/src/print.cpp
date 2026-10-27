#include "bt_button_tutorial/print.hpp"
#include <iostream>

Print::Print(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList Print::providedPorts()
{
  return {BT::InputPort<std::string>("message")};
}

BT::NodeStatus Print::tick()
{
  // Read the message input from XML (or from the blackboard) and print it.
  std::cout << getInput<std::string>("message").value() << std::endl;

  // Printing is immediate and cannot remain RUNNING in this example.
  return BT::NodeStatus::SUCCESS;
}
