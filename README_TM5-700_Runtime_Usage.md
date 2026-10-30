# TM5-700 Book Retrieval System — Runtime and Operation Guide

This guide explains how to prepare the network connection to the Techman TM5-700 robot, build the application when needed, and start the book-retrieval system using five terminal windows.

It is intended to be used **after** completing the installation guide for Windows/Ubuntu dual boot, Ubuntu 22.04, ROS 2 Humble, the Techman ROS 2 driver, and the project dependencies.

---

## 1. System overview

The runtime system is started through five terminals:

| Terminal | Main function | Command / node |
|---|---|---|
| 1 | Starts the TM5-700 MoveIt 2 move group and robot configuration | `tm5-700_run_move_group.launch.py` |
| 2 | Starts the project's MoveIt bridge | `moveit_bridge_test.launch.py` |
| 3 | Starts the RealSense/YOLO book-perception node | `perception_node` |
| 4 | Starts the robot control user interface | `robot_control_ui` |
| 5 | Starts the Arduino gripper serial node | `gripper_serial` |

The system also requires the robot Ethernet connection to be configured before starting the terminals.

### Important configuration used in this guide

| Item | Value |
|---|---|
| Ubuntu network interface | `enp67s0` |
| Ubuntu robot-network IP | `169.254.167.30/16` |
| TM5-700 robot IP | `169.254.167.105` |
| TM description package | `tm_description` |
| Project workspace | `~/fyp_phase3` |
| TM5 driver workspace | `~/tmdriver_tm5_ws` |
| TM2 driver workspace | `~/tmdriver_tm2_ws` |
| Python virtual environment | `~/fyp_env` |
| ROS distribution | ROS 2 Humble |

> **Network note:** These IP addresses and the interface name are the values used in this setup. If the robot IP, PC network interface, or network configuration changes, update the commands in this guide accordingly. Confirm the interface name with `ip -br link` rather than assuming it is always `enp67s0`.

---

## 2. Before starting: prepare the robot Ethernet connection

The PC needs an address on the same link-local network as the robot.

### Step 1 — Assign the PC's robot-network IP

Open a terminal and run:

```bash
sudo ip addr add 169.254.167.30/16 dev enp67s0
```

This adds `169.254.167.30/16` to the Ethernet interface `enp67s0`.

This is a temporary IP configuration. It may need to be applied again after restarting Ubuntu.

**If the command reports `File exists`:** the address may already be assigned. Check the interface before adding it again.

### Step 2 — Check the Ethernet interface

```bash
ip -br addr show enp67s0
```

Expected output should show that `enp67s0` is `UP` and has both the normal LAN address and the robot-network address. For example:

```text
enp67s0  UP  192.168.1.30/24  169.254.167.30/16
```

The `192.168.1.30/24` address is an example of the normal LAN address; it may differ on another PC or network.

### Step 3 — Confirm the route to the robot

```bash
ip route get 169.254.167.105
```

Expected route:

```text
169.254.167.105 dev enp67s0 src 169.254.167.30
```

The important points are:

- The route uses `enp67s0`.
- The source address is `169.254.167.30`.

### Step 4 — Ping the robot

```bash
ping -c 4 169.254.167.105
```

A successful connection should return replies from `169.254.167.105`.

If the ping fails, do not proceed to robot motion. Check the Ethernet cable, robot power, robot IP configuration, PC interface, and route first.

### Step 5 — Check the neighbour entry

```bash
ip neigh show 169.254.167.105
```

An example of a reachable entry is:

```text
169.254.167.105 dev enp67s0 lladdr 00:10:f3:ab:6e:a4 REACHABLE
```

The MAC address shown here is an example from this setup. The neighbour state may change over time; the key point is that the PC can resolve the robot's link-layer address.

### Step 6 — Final network verification

Run these checks again before launching the system:

```bash
ip -br addr show enp67s0
ip route get 169.254.167.105
ping -c 4 169.254.167.105
```

Do not continue until the PC is using the intended Ethernet interface and can communicate with the robot.

---

## 3. Build the project (only when required)

You do **not** need to rebuild every time you start the robot. Build after modifying source code, changing interfaces/messages, or updating packages.

There are two build procedures below. Use the one that matches the changes you made.

### Option A — Rebuild the main application packages

Use this for ordinary changes to the book-retrieval application or MoveIt bridge when the custom message package does not need to be rebuilt.

```bash
cd ~/fyp_phase3

source /opt/ros/humble/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate

colcon build --packages-select tm5_book_retrieval tm5_moveit_bridge --symlink-install

source ~/fyp_phase3/install/setup.bash
```

### Option B — Rebuild the custom message and application packages

Use this when changes involve `tm5_book_msgs`, or when you want to rebuild all three project packages together.

```bash
cd ~/fyp_phase3

source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_env/bin/activate

colcon build --packages-select tm5_book_msgs tm5_book_retrieval tm5_moveit_bridge
```

After a successful build, source the project workspace in any terminal that will run the updated packages:

```bash
source ~/fyp_phase3/install/setup.bash
```

> **Build notes**
>
> - Run build commands from `~/fyp_phase3`.
> - Make sure the ROS and driver workspaces have already been built successfully.
> - If a package is not found at runtime, check that the correct workspace has been sourced in that terminal.
> - Option A uses `--symlink-install`; Option B follows the full rebuild command used in this setup. Keep the build method consistent with the package changes and your working environment.
> - If you modify message definitions, rebuild `tm5_book_msgs` before launching nodes that depend on those messages.

---

## 4. Start the system using five terminals

Open **five separate terminal windows**. Keep each process running in its own terminal. Do not close a terminal while the system is operating.

The commands below use the current workspace locations and robot IP from this setup.

### Terminal 1 — Start TM5-700 MoveIt 2

**Purpose:** Starts the TM5-700 MoveIt 2 move group using the robot's configuration.

Run:

```bash
cd ~/tmdriver_tm5_ws

source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate

export AMENT_PREFIX_PATH=$HOME/tmdriver_tm5_ws/install/tm_description:$AMENT_PREFIX_PATH

ros2 launch tm5-700_moveit_config tm5-700_run_move_group.launch.py robot_ip:=169.254.167.105
```

Optional package checks:

```bash
ros2 pkg prefix tm_description
ros2 pkg prefix tm_driver
ros2 pkg prefix tm5-700_moveit_config
```

These checks should return package installation paths. If a package cannot be found, verify that the relevant workspace has been built and sourced.

**Leave Terminal 1 running.**

### Terminal 2 — Start the MoveIt bridge

**Purpose:** Starts the project's C++ bridge between the book-retrieval application and MoveIt.

Run:

```bash
cd ~/fyp_phase3

source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate

export AMENT_PREFIX_PATH=$HOME/tmdriver_tm5_ws/install/tm_description:$AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH=$HOME/fyp_phase3/install/tm5_book_msgs:$AMENT_PREFIX_PATH

ros2 launch tm5_moveit_bridge moveit_bridge_test.launch.py
```

Optional package checks:

```bash
ros2 pkg prefix tm5_moveit_bridge
ros2 pkg prefix tm5_book_msgs
```

These checks should return the installed package paths.

**Leave Terminal 2 running.**

### Terminal 3 — Start the perception node

**Purpose:** Starts the camera and book-perception pipeline. This node runs the project's perception code, including the RealSense camera and YOLO-based book detection.

Run:

```bash
cd ~/fyp_phase3

source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate

export AMENT_PREFIX_PATH=$HOME/tmdriver_tm5_ws/install/tm_description:$AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH=$HOME/fyp_phase3/install/tm5_book_msgs:$AMENT_PREFIX_PATH

export PYTHONPATH=~/fyp_phase3/src/tm5_book_retrieval:$PYTHONPATH

python -m tm5_book_retrieval.perception_node
```

Check that the RealSense D435i is connected and that the perception node can access its camera and model files.

**Leave Terminal 3 running.**

### Terminal 4 — Start the robot control UI

**Purpose:** Opens the user interface used to operate the book-retrieval system.

Run:

```bash
cd ~/fyp_phase3

source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate

export AMENT_PREFIX_PATH=$HOME/tmdriver_tm5_ws/install/tm_description:$AMENT_PREFIX_PATH

ros2 run tm5_book_retrieval robot_control_ui
```

The previous direct Python launch method was:

```bash
python3 ~/fyp_phase3/src/tm5_book_retrieval/tm5_book_retrieval/robot_control_ui.py
```

Prefer the installed ROS 2 executable (`ros2 run ...`) when the package has been built and installed correctly. Use the direct Python command only if you intentionally need to run the source file directly.

**Keep the UI open while operating the system.**

### Terminal 5 — Start the Arduino gripper serial node

**Purpose:** Starts the serial communication node that sends gripper commands to the Arduino.

First, ensure the user has serial-port permission. In a new terminal, run:

```bash
newgrp dialout
groups
```

Confirm that `dialout` appears in the group list. `newgrp dialout` opens a new shell with the group applied; continue in that same terminal.

Then run:

```bash
cd ~/fyp_phase3

source /opt/ros/humble/setup.bash
source ~/fyp_phase3/install/setup.bash

ros2 run tm5_book_retrieval gripper_serial
```

If the serial node cannot open the Arduino port:

1. Confirm that the Arduino is connected.
2. Check the detected serial device:

   ```bash
   ls -l /dev/ttyACM*
   ```

3. Check the current user's groups:

   ```bash
   groups
   ```

4. If the user was newly added to `dialout`, log out and log back in before retrying. Alternatively, open a new terminal and apply `newgrp dialout` as shown above.

The serial device may not always be `/dev/ttyACM0`; check the actual device name if the node reports a serial-port error.

**Leave Terminal 5 running.**

---

## 5. Recommended startup order

Use this order when starting the complete system:

1. **Prepare the robot:** Power on the TM5-700, release any emergency stop only when safe, and make sure the robot is in the required ready state.
2. **Prepare the TMflow program:** Confirm that the robot-side TMflow Listen/Ethernet Slave communication setup is running and configured for the expected connection.
3. **Configure and verify the PC Ethernet IP:** Complete Section 2 and confirm that the robot responds to ping.
4. **Terminal 1:** Start the TM5-700 MoveIt 2 launch.
5. **Terminal 2:** Start the MoveIt bridge.
6. **Terminal 5:** Start the Arduino gripper serial node.
7. **Terminal 3:** Start the perception node and confirm that the camera/perception pipeline is running.
8. **Terminal 4:** Start the robot control UI.
9. **Verify the complete system** before issuing a robot command.

The exact order of starting the independent perception and gripper nodes can be adjusted if required by the application, but start the MoveIt components before sending motion commands from the UI.

---

## 6. Pre-operation safety checklist

Before commanding the robot:

- [ ] Robot and PC are powered and connected correctly.
- [ ] Robot Ethernet IP is reachable from the PC.
- [ ] TMflow is in the correct communication/Listen state.
- [ ] Terminal 1 MoveIt launch is running without a fatal error.
- [ ] Terminal 2 MoveIt bridge is running.
- [ ] Terminal 3 perception node is running and the camera is detected.
- [ ] Terminal 4 UI is open and responsive.
- [ ] Terminal 5 gripper serial node is running and the Arduino is connected.
- [ ] The robot workspace is clear of people and obstacles.
- [ ] The book, shelf, gripper, and robot starting position are arranged as expected.
- [ ] The emergency stop is accessible throughout testing.

**Important:** Keep a hand near the emergency stop during initial tests and after any code, calibration, coordinate, or motion-parameter change. Test at reduced speed and with a clear workspace first. Never assume a successful ROS launch means the physical robot is safe to move.

---

## 7. Useful checks while the system is running

### Check the robot connection

```bash
ping -c 4 169.254.167.105
```

### Check ROS 2 topics

```bash
ros2 topic list
```

### Check robot joint-state publication

```bash
ros2 topic echo /joint_states
```

Stop the topic display with `Ctrl+C`.

### Check available project packages

```bash
ros2 pkg list | grep tm5
```

### Check the installed executable names

```bash
ros2 pkg executables tm5_book_retrieval
```

### Check the Arduino serial device

```bash
ls -l /dev/ttyACM*
```

---

## 8. Shutting down

To stop the system safely:

1. Stop issuing commands from the UI.
2. Allow the robot to complete its current action or move it to a safe state using the normal control procedure.
3. Stop the UI and the running ROS nodes with `Ctrl+C`, preferably in this order:
   - Terminal 4 — UI
   - Terminal 3 — perception
   - Terminal 5 — gripper serial node
   - Terminal 2 — MoveIt bridge
   - Terminal 1 — MoveIt 2
4. Power down the robot and other hardware using the normal equipment shutdown procedure.

Do not unplug the robot Ethernet cable, Arduino, or camera while the system is actively executing a task.

---

## 9. Troubleshooting

| Symptom | Checks / actions |
|---|---|
| Robot ping fails | Check robot power, Ethernet cable, PC interface name, PC IP address, robot IP, and `ip route get 169.254.167.105`. |
| Route uses the wrong interface or source IP | Reapply the PC robot-network IP to the correct interface and check the route again. |
| `ros2` cannot find a package | Source ROS 2 Humble and the required driver/project workspaces in that terminal. Confirm the package was built. |
| `colcon: command not found` | Install the ROS 2 colcon extensions package as described in the installation guide. |
| MoveIt launch cannot find `tm_description` | Check the TM5 workspace build and the `AMENT_PREFIX_PATH` export in Terminal 1. |
| Bridge cannot find `tm5_book_msgs` | Rebuild the message package if necessary, source the project workspace, and check the `AMENT_PREFIX_PATH` export in Terminal 2. |
| Perception node cannot import the project module | Check the Python virtual environment, project workspace sourcing, and `PYTHONPATH` export in Terminal 3. |
| Camera is not detected | Check the USB connection, camera power, RealSense installation, and whether another process is already using the camera. |
| UI executable is not found | Rebuild the project package, source `~/fyp_phase3/install/setup.bash`, and check `ros2 pkg executables tm5_book_retrieval`. |
| Arduino serial permission denied | Check `dialout` membership, use `newgrp dialout` in the terminal, and verify the actual `/dev/ttyACM*` device. |
| Robot does not move despite nodes running | Check the MoveIt/driver terminal output, robot state, TMflow communication state, planning result, and safety conditions. Do not repeatedly resend motion commands without identifying the cause. |

---

## 10. Notes for adapting this guide to another computer

Before using this guide on another PC, verify and update:

- The Ethernet interface name (currently `enp67s0`).
- The PC robot-network IP (currently `169.254.167.30/16`).
- The robot IP (currently `169.254.167.105`).
- The workspace folder names and locations.
- The installed ROS package and executable names.
- The Arduino serial device and user permissions.
- The camera device and model-file paths.

The commands in this document reflect the working command sequence used for this project. If the repository structure, package names, launch files, or hardware configuration changes, update the relevant commands and test the complete startup procedure before publishing a revised version.

---

## 11. Quick-start command reference

Use this section as a compact checklist once the full procedure above is understood.

### A. Network setup

```bash
sudo ip addr add 169.254.167.30/16 dev enp67s0
ip -br addr show enp67s0
ip route get 169.254.167.105
ping -c 4 169.254.167.105
ip neigh show 169.254.167.105
```

### B. Terminal 1 — MoveIt 2

```bash
cd ~/tmdriver_tm5_ws
source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate
export AMENT_PREFIX_PATH=$HOME/tmdriver_tm5_ws/install/tm_description:$AMENT_PREFIX_PATH
ros2 launch tm5-700_moveit_config tm5-700_run_move_group.launch.py robot_ip:=169.254.167.105
```

### C. Terminal 2 — MoveIt bridge

```bash
cd ~/fyp_phase3
source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate
export AMENT_PREFIX_PATH=$HOME/tmdriver_tm5_ws/install/tm_description:$AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH=$HOME/fyp_phase3/install/tm5_book_msgs:$AMENT_PREFIX_PATH
ros2 launch tm5_moveit_bridge moveit_bridge_test.launch.py
```

### D. Terminal 3 — Perception

```bash
cd ~/fyp_phase3
source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate
export AMENT_PREFIX_PATH=$HOME/tmdriver_tm5_ws/install/tm_description:$AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH=$HOME/fyp_phase3/install/tm5_book_msgs:$AMENT_PREFIX_PATH
export PYTHONPATH=~/fyp_phase3/src/tm5_book_retrieval:$PYTHONPATH
python -m tm5_book_retrieval.perception_node
```

### E. Terminal 4 — UI

```bash
cd ~/fyp_phase3
source /opt/ros/humble/setup.bash
source ~/tmdriver_tm2_ws/install/setup.bash
source ~/tmdriver_tm5_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
source ~/fyp_env/bin/activate
export AMENT_PREFIX_PATH=$HOME/tmdriver_tm5_ws/install/tm_description:$AMENT_PREFIX_PATH
ros2 run tm5_book_retrieval robot_control_ui
```

### F. Terminal 5 — Gripper serial node

```bash
newgrp dialout
groups

cd ~/fyp_phase3
source /opt/ros/humble/setup.bash
source ~/fyp_phase3/install/setup.bash
ros2 run tm5_book_retrieval gripper_serial
```

---

## References

- Techman ROS 2 driver: https://github.com/TechmanRobotInc/tmr_ros2
- ROS 2 Humble documentation: https://docs.ros.org/en/humble/
