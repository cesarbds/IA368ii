#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <bt_button_interfaces/srv/random_odd.hpp>

// Calls the Python random-number service. An odd result becomes BT SUCCESS;
// an even result becomes BT FAILURE, allowing a Fallback node to branch on it.
class RandomOdd : public BT::RosServiceNode<bt_button_interfaces::srv::RandomOdd>
{
public:
  RandomOdd(const std::string& name, const BT::NodeConfig& config,
            const BT::RosNodeParams& params);

  // Adds the returned number as an output port.
  static BT::PortsList providedPorts();

  // Prepares the service request before it is sent.
  bool setRequest(Request::SharedPtr& request) override;

  // Converts the Python service response into a BT status.
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  // Handles ROS service communication errors.
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
