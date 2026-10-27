# Wait, then press a button: C++ tree with Python behavior

This package now contains two independent examples. `button_tree` is the
original button example. `random_button_tree` adds a subtree that calls a
random-number service after the button is pressed.

## Files

| File | Responsibility |
|---|---|
| `behavior_trees/button.xml` | Tree structure, unchanged |
| `src/button_tree.cpp` | C++ registration and execution |
| `behavior_trees/button_random.xml` | Second tree containing the subtree |
| `src/random_button_tree.cpp` | Registration and execution for the second tree |
| `src/wait.cpp` | C++ initial wait |
| `src/print.cpp` | C++ message printing |
| `src/wait_for_button.cpp` | Small C++ ROS action client adapter |
| `src/random_odd.cpp` | Small C++ ROS service client adapter |
| `scripts/button_server.py` | Python action server and random-number service |
| `../bt_button_interfaces/action/WaitForButton.action` | Shared ROS action definition |
| `../bt_button_interfaces/srv/RandomOdd.srv` | Shared ROS service definition |

The C++ adapter sends `seconds` to `/tutorial/wait_for_button`. The Python server returns `pressed`. The adapter maps true to BT SUCCESS and false to BT FAILURE. Transport errors stop the tree with an error, rather than printing a misleading `No button`.

## Build

Use the existing BehaviorTree.ROS2 workspace and dependencies:

```bash
source /opt/ros/humble/setup.bash
cd /home/trossen-ai/Documents/Cesar/IA3682026S2/IA368ii/IA368_ws
source install/setup.bash
colcon build --packages-select bt_button_interfaces bt_button_tutorial --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_RPATH=/usr/lib/x86_64-linux-gnu
source install/setup.bash
```

The Python paths select system Python for ROS build tools. The RPATH selects the system C++ runtime ahead of Conda libraries on this machine. The Python executable uses `/usr/bin/python3` in its shebang for the same reason.

## Run

In each terminal first run:

```bash
source /opt/ros/humble/setup.bash
source /home/trossen-ai/Documents/Cesar/IA3682026S2/IA368ii/IA368_ws/install/setup.bash
```

Terminal 1: start the Python server and leave it running:

```bash
ros2 run bt_button_tutorial button_server.py
```

Terminal 2: run the original tree:

```bash
ros2 run bt_button_tutorial button_tree
```

Or run the second tree:

```bash
ros2 run bt_button_tutorial random_button_tree
```

Terminal 2 prints `Waiting 5 seconds...`. After that initial wait, terminal 1 prints `Press the button now! Listening for 5 seconds...`.

Terminal 3: publish promptly AFTER the prompt in terminal 1:

```bash
ros2 topic pub --once /tutorial/button std_msgs/msg/Bool '{data: true}'
```

Terminal 2 prints `Pressed button`. To test the timeout, rerun terminal 2 and publish nothing: it prints `No button` after about ten seconds total.

Only the second tree continues after a press. The Python server generates an integer from 1 through 100. The
C++ service wrapper returns BT `SUCCESS` for odd and BT `FAILURE` for even. The
`CheckRandomNumber` subtree uses those statuses to print the corresponding
message.

Single early presses and false messages are ignored. Repeated publishing that continues into the window can count. Wall time is used, so no simulator or `/clock` is needed. Callback receipt within the listening window determines whether a press counts.

## Original tree

```text
Sequence
├── Wait (5 s, C++)
└── Fallback
    ├── Sequence
    │   ├── WaitForButton (C++ client → Python action, up to 5 s)
    │   └── Print "Pressed button"
    └── Print "No button"
```

## Second tree

```text
Sequence
├── Wait (5 s, C++)
└── Fallback
    ├── Sequence
    │   ├── WaitForButton (C++ client → Python server, up to 5 s)
    │   ├── Print "Pressed button" (C++)
    │   └── CheckRandomNumber (subtree)
    └── Print "No button" (C++)

CheckRandomNumber
└── Fallback
    ├── Sequence
    │   ├── RandomOdd (C++ client → Python service)
    │   └── Print "Random number is odd"
    └── Print "Random number is even"
```

Sequence stops at the first FAILURE. Fallback tries the next child on FAILURE. WaitForButton stays RUNNING while the Python action is in progress. Both final message paths return SUCCESS: a button timeout is a handled outcome.

## Inspect or call the Python action directly

```bash
ros2 interface show bt_button_interfaces/action/WaitForButton
ros2 action send_goal /tutorial/wait_for_button bt_button_interfaces/action/WaitForButton '{seconds: 5.0}' --feedback
ros2 interface show bt_button_interfaces/srv/RandomOdd
ros2 service call /tutorial/random_odd bt_button_interfaces/srv/RandomOdd '{}'
```

The direct action call opens its listening window immediately. The initial five-second delay belongs to the tree, not the button server. The server accepts one goal at a time and accepts cancellation; halting the BT action client requests cancellation automatically.
