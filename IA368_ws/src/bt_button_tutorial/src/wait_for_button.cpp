#include "bt_button_tutorial/wait_for_button.hpp"
#include <cmath>
#include <stdexcept>

WaitForButton::WaitForButton(const std::string& name, const BT::NodeConfig& config,
                             const BT::RosNodeParams& params)
  // RosActionNode creates and manages the ROS action client.
  : BT::RosActionNode<bt_button_interfaces::action::WaitForButton>(name, config, params) {}

BT::PortsList WaitForButton::providedPorts()
{
  return providedBasicPorts({BT::InputPort<double>("seconds")});
}

bool WaitForButton::setGoal(Goal& goal)
{
  // Copy <WaitForButton seconds="..."/> from XML into the ROS goal message.
  goal.seconds = getInput<double>("seconds").value();

  // Returning false prevents an invalid goal from being sent.
  return std::isfinite(goal.seconds) && goal.seconds > 0;
}

BT::NodeStatus WaitForButton::onResultReceived(const WrappedResult& result)
{
  // A press is BT SUCCESS. A normal five-second timeout is BT FAILURE, which
  // lets the XML Fallback execute its "No button" branch.
  if (result.result->pressed) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus WaitForButton::onFailure(BT::ActionNodeErrorCode error)
{
  // Infrastructure errors must not be mistaken for a normal button timeout.
  throw std::runtime_error(std::string("Button action failed: ") + BT::toStr(error));
}
