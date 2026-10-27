#include "bt_button_tutorial/random_odd.hpp"

#include <stdexcept>

RandomOdd::RandomOdd(const std::string& name, const BT::NodeConfig& config,
                     const BT::RosNodeParams& params)
  // RosServiceNode creates and manages the ROS service client.
  : BT::RosServiceNode<bt_button_interfaces::srv::RandomOdd>(name, config, params) {}

BT::PortsList RandomOdd::providedPorts()
{
  // The XML may save this value in the blackboard using
  // <RandomOdd number="{random_number}"/>.
  return providedBasicPorts({BT::OutputPort<int>("number")});
}

bool RandomOdd::setRequest(Request::SharedPtr&)
{
  // This service has an empty request: calling it is enough.
  return true;
}

BT::NodeStatus RandomOdd::onResponseReceived(const Response::SharedPtr& response)
{
  // Make the returned number available to other nodes through the blackboard.
  setOutput("number", response->number);

  RCLCPP_INFO(logger(), "Service returned %d (%s)", response->number,
              response->is_odd ? "odd" : "even");

  // The subtree branches on this status: odd follows SUCCESS, while even
  // causes its Fallback to try the next child.
  if (response->is_odd) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RandomOdd::onFailure(BT::ServiceNodeErrorCode error)
{
  // A communication failure is different from a valid even-number result.
  throw std::runtime_error("Random-number service failed with error " +
                           std::to_string(static_cast<int>(error)));
}
