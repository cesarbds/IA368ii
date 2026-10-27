#!/usr/bin/python3
"""Python implementation of the button behavior, called by the C++ tree."""
import math
import random
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from bt_button_interfaces.action import WaitForButton
from bt_button_interfaces.srv import RandomOdd


class ButtonServer(Node):
    def __init__(self):
        super().__init__('button_server')

        # The executor below uses several threads. The lock protects state that
        # is accessed by both the action callback and the topic callback.
        self._lock = threading.Lock()

        # Only one WaitForButton action goal is allowed at a time.
        self._busy = False

        # None means that the action is not currently listening for a press.
        self._deadline = None

        # A thread-safe flag set by button_callback() and read by execute().
        self._pressed = threading.Event()

        # ReentrantCallbackGroup allows the action, topic and service callbacks
        # to execute concurrently when used with MultiThreadedExecutor.
        group = ReentrantCallbackGroup()

        # This topic simulates a physical button. Only a Bool(data=True)
        # received during the action's listening window counts as a press.
        self._subscription = self.create_subscription(
            Bool, '/tutorial/button', self.button_callback, 10,
            callback_group=group)

        # Long-running interface used by the C++ WaitForButton BT node.
        self._server = ActionServer(
            self, WaitForButton, '/tutorial/wait_for_button',
            execute_callback=self.execute,
            goal_callback=self.accept_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

        # Short request/response interface used by the second behavior tree.
        self._random_service = self.create_service(
            RandomOdd, '/tutorial/random_odd', self.random_odd_callback,
            callback_group=group)
        self.get_logger().info('Python action and service server ready.')

    def random_odd_callback(self, request, response):
        """Generate a number and return both the number and its parity."""
        del request  # The RandomOdd request is intentionally empty.
        response.number = random.randint(1, 100)
        response.is_odd = response.number % 2 == 1
        self.get_logger().info(
            f'Generated {response.number}: '
            f'{"odd" if response.is_odd else "even"}')
        return response

    def accept_goal(self, request):
        """Validate a new action goal before execution starts."""
        with self._lock:
            # Reject concurrent goals and invalid timeout values.
            if self._busy or not math.isfinite(request.seconds) or request.seconds <= 0:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def button_callback(self, msg):
        """Record a true button message only while an action is listening."""
        with self._lock:
            if (msg.data and self._deadline is not None
                    and time.monotonic() < self._deadline):
                self._pressed.set()

    def execute(self, goal_handle):
        """Wait for a press, cancellation, or expiration of the timeout."""
        # pressed defaults to False, which is the result returned on timeout
        # or cancellation.
        result = WaitForButton.Result()
        try:
            with self._lock:
                # Each accepted goal starts with a fresh button state and its
                # own listening deadline.
                self._pressed.clear()
                self._deadline = time.monotonic() + goal_handle.request.seconds
                deadline = self._deadline
            self.get_logger().info(
                f'Press the button now! Listening for {goal_handle.request.seconds:g} seconds...')
            while rclpy.ok():
                # The C++ BT wrapper automatically requests cancellation when
                # its behavior-tree node is halted.
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result

                # A press is the successful outcome of this action.
                if self._pressed.is_set():
                    result.pressed = True
                    goal_handle.succeed()
                    return result

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    # The action completed normally; pressed=False means timeout.
                    goal_handle.succeed()
                    return result

                # Feedback is optional, but lets action clients observe how
                # much time remains without changing the final result.
                feedback = WaitForButton.Feedback()
                feedback.seconds_remaining = remaining
                goal_handle.publish_feedback(feedback)

                # Sleep efficiently, waking immediately if the topic callback
                # sets the event. The short maximum also checks cancellation.
                self._pressed.wait(min(0.05, remaining))

            # ROS shut down while the goal was active.
            goal_handle.abort()
            return result
        finally:
            # Always release the server for another goal, regardless of how
            # this goal ended.
            with self._lock:
                self._deadline = None
                self._busy = False


def main():
    # Initialize ROS and create the node containing both servers.
    rclpy.init()
    node = ButtonServer()

    # execute() waits on one worker; another handles button and cancel callbacks.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down ROS and release node/executor resources on Ctrl+C.
        if rclpy.ok():
            rclpy.shutdown()
        executor.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
