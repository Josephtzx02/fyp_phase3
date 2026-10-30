# fyp_phase3: TM5-700 Robotic Arm --- Windows Dual Boot, Ubuntu 22.04, ROS 2 Humble & Techman Driver Setup

3D scene analysis and robotic vision system for autonomous book retrieval using a Techman TM5-700 robotic arm.

This guide documents a reproducible workstation setup for developing and
operating a **Techman TM5-700** robotic arm with **Ubuntu 22.04 LTS**,
**ROS 2 Humble**, Techman's ROS 2 driver, and this project's
book-retrieval application.

> **Important:** This README is a setup guide, not a guarantee that
> every laptop, TMflow release, robot controller, camera, GPU, or
> project revision will behave identically. Confirm your own hardware,
> IP addresses, software versions, and safety configuration before
> running the robot. Never use `Erase disk and install Ubuntu` when you
> intend to keep Windows.

------------------------------------------------------------------------

## 1. System overview

The intended software stack is:

``` text
Windows 10/11
    └── Ubuntu 22.04 LTS (dual boot)
          └── ROS 2 Humble
                ├── Techman ROS 2 driver (tmr_ros2, humble branch)
                └── TM5-700 book-retrieval application
                      ├── tm5_book_msgs
                      ├── tm5_book_retrieval
                      ├── tm5_moveit_bridge
                      ├── MoveIt 2
                      ├── Intel RealSense D435i
                      ├── YOLO-based book detection
                      └── Arduino-based mechanical gripper
```

The main communication path is:

``` text
TMflow Listen task + Ethernet Slave
                ↕ Ethernet
          ROS 2 Humble
                ↕
          tm_driver
                ↕
        MoveIt 2 / application
                ↕
     TM5-700 + gripper + camera
```

The project-specific perception and manipulation path is:

``` text
Intel RealSense D435i
        ↓
RGB-D acquisition / OpenCV
        ↓
YOLO oriented-book detection
        ↓
Book geometry and grasp target
        ↓
tm5_book_msgs
        ↓
tm5_moveit_bridge + MoveIt 2
        ↓
TM5-700
```

The mechanical gripper is controlled through an Arduino serial
interface. The exact wiring, firmware, model weights, camera
calibration, and robot calibration must be included in this repository
(or linked as versioned release assets) for another user to reproduce
the complete application.

------------------------------------------------------------------------

## 2. Hardware and software requirements

### Hardware

-   Windows laptop/PC capable of running Ubuntu 22.04.
-   USB flash drive (8 GB minimum; 16 GB or larger recommended).
-   Sufficient free SSD space for Ubuntu, ROS 2, build artifacts,
    models, and datasets. Allocate space based on the size of the
    project; 100 GB or more is a practical starting point for a
    development workstation.
-   Ethernet port or USB-to-Ethernet adapter for the TM5-700.
-   Techman TM5-700 robot and controller running a compatible TMflow
    version.
-   Intel RealSense D435i depth camera (for the perception application).
-   Arduino Uno and the project's servo-driven mechanical gripper (for
    gripper control).

### Software versions used by this setup

  Component                  Version / branch
  -------------------------- ---------------------------------------------
  Host OS                    Windows 10/11
  Linux OS                   Ubuntu 22.04 LTS (Jammy, 64-bit)
  ROS                        ROS 2 Humble Hawksbill
  Techman driver             `TechmanRobotInc/tmr_ros2`, branch `humble`
  Build system               `colcon`
  Robot model                TM5-700
  Camera                     Intel RealSense D435i
  Application workspace      `~/fyp_phase3`
  Techman driver workspace   `~/tmdriver_ws`

The Techman repository describes its `humble` branch as the ROS 2 Humble
driver. Always check the upstream repository for updates and
compatibility notes: https://github.com/TechmanRobotInc/tmr_ros2

------------------------------------------------------------------------

# Part A --- Install Ubuntu alongside Windows

## 3. Prepare Windows

**Back up important files before changing disk partitions or installing
another operating system.** Keep a Windows recovery drive and make sure
you can access your BitLocker recovery key if device encryption is
enabled.

1.  In Windows, install pending updates and restart.
2.  Check whether BitLocker / Device Encryption is enabled. If it is,
    save the recovery key and follow Microsoft's and Ubuntu's current
    guidance before changing partitions. Do not proceed if you cannot
    recover the encrypted Windows installation.
3.  Open **Disk Management** (`Win + X` → Disk Management).
4.  Right-click the Windows partition (usually `C:`) and choose **Shrink
    Volume**.
5.  Shrink it to create unallocated space for Ubuntu. Do not format this
    new space as NTFS; leave it unallocated for the Ubuntu installer.
6.  Disable Windows Fast Startup so Windows does not leave shared disks
    in a hibernated state.
7.  If your laptop firmware has a Secure Boot setting, understand the
    implications before changing it. This installation can use Secure
    Boot; if Ubuntu asks to enroll a Machine Owner Key (MOK) for
    third-party drivers, follow the on-screen enrollment process and
    remember the password you created.

> Partition sizes depend on the machine and workload. Do not copy a
> partition size blindly. Ensure Windows retains enough free space for
> updates and normal use.

## 4. Download Ubuntu and create a bootable USB

1.  Download the **Ubuntu 22.04 LTS Desktop 64-bit ISO** from the
    official Ubuntu releases/download page:
    https://releases.ubuntu.com/22.04/
2.  Download Rufus from: https://rufus.ie/
3.  Insert the USB flash drive. **Rufus will erase the USB drive**, so
    copy any files from it first.
4.  In Rufus:
    -   Select the Ubuntu ISO.
    -   Select the correct USB device.
    -   For a typical Windows UEFI installation, use **GPT** and **UEFI
        (non-CSM)**.
    -   Start writing the image and accept the recommended image-writing
        mode if prompted.
5.  Restart the computer and open the one-time boot menu (the key varies
    by manufacturer; common keys include F12, F9, Esc, or F8).
6.  Select the USB entry that starts with `UEFI`.

Use the official Ubuntu installation guide if your firmware or installer
screens differ: https://ubuntu.com/tutorials/install-ubuntu-desktop

## 5. Install Ubuntu in dual-boot mode

1.  Boot the USB and select **Try or Install Ubuntu**.
2.  Start the Ubuntu installer.
3.  Connect to Wi-Fi if available.
4.  Select the normal/default installation (or minimal if you prefer to
    install applications later).
5.  Enable:
    -   **Download updates while installing Ubuntu**
    -   **Install third-party software for graphics and Wi-Fi hardware
        and additional media formats**
6.  If the installer offers **Configure Secure Boot**, follow the
    prompts only if you want to install third-party drivers that require
    MOK enrollment. Create a MOK password you can enter at the next
    boot.
7.  At **Installation type**, choose **Install Ubuntu alongside Windows
    Boot Manager**.
8.  Carefully inspect the disk-change summary before confirming. It
    should show Ubuntu using the free space and should not show Windows
    partitions being erased or formatted.
9.  Do not select **Erase disk and install Ubuntu** if you want to
    preserve Windows.
10. Complete the timezone, username, computer name, and password
    screens.
11. When installation finishes, restart and remove the USB when
    prompted.
12. If the MOK Manager screen appears:
    -   Select **Enroll MOK**.
    -   Select **Continue**.
    -   Confirm **Yes**.
    -   Enter the MOK password created during installation.
    -   Select **Reboot**.

At startup, choose Ubuntu or Windows from the boot menu/GRUB menu.

------------------------------------------------------------------------

# Part B --- Prepare Ubuntu

## 6. Update Ubuntu

Open a terminal with `Ctrl + Alt + T`:

``` bash
sudo apt update
sudo apt upgrade -y
```

Restart if Ubuntu or a driver update requests it.

## 7. Install basic development tools

``` bash
sudo apt install -y \
  git \
  curl \
  wget \
  build-essential \
  cmake \
  python3-pip \
  python3-venv \
  python3-dev \
  software-properties-common \
  openssh-client
```

Confirm the operating system:

``` bash
lsb_release -a
```

The target release should be Ubuntu 22.04 / Jammy.

------------------------------------------------------------------------

# Part C --- Install ROS 2 Humble

## 8. Configure the ROS 2 package repository

The following follows the ROS 2 Humble Debian-package installation
approach for Ubuntu Jammy.

Enable the Ubuntu Universe repository:

``` bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

Install `curl` and obtain the current ROS apt-source package version:

``` bash
sudo apt update
sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
```

Download the Ubuntu Jammy package:

``` bash
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
```

Install it and refresh the package index:

``` bash
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
```

## 9. Install ROS 2 Humble Desktop

``` bash
sudo apt install -y ros-humble-desktop
```

Install ROS development tools and the `colcon` extensions required to
build ROS workspaces:

``` bash
sudo apt install -y \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep
```

Initialize `rosdep` (run `rosdep init` only once on a fresh Ubuntu
installation):

``` bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that it has already been initialized, continue
with `rosdep update`.

## 10. Source ROS 2 Humble

For the current terminal:

``` bash
source /opt/ros/humble/setup.bash
```

Verify:

``` bash
echo $ROS_DISTRO
```

Expected output:

``` text
humble
```

To source Humble automatically in new Bash terminals, add this line to
`~/.bashrc`:

``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Avoid adding multiple duplicate copies of the same line.

## 11. Test ROS 2

Open two terminals.

**Terminal 1:**

``` bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2:**

``` bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

The listener should print messages received from the talker. Stop both
with `Ctrl + C`.

------------------------------------------------------------------------

# Part D --- Install the official Techman ROS 2 driver

## 12. Create the Techman driver workspace

``` bash
mkdir -p ~/tmdriver_ws
cd ~/tmdriver_ws
```

## 13. Clone the Humble branch

``` bash
git clone https://github.com/TechmanRobotInc/tmr_ros2.git -b humble
```

Rename the cloned repository to the ROS workspace source directory, as
described by Techman:

``` bash
mv tmr_ros2 src
```

Check the workspace:

``` bash
ls
```

Expected:

``` text
src
```

The source tree should contain the Techman ROS 2 packages:

``` text
~/tmdriver_ws/
└── src/
    └── ...
```

## 14. Install package dependencies

Source ROS 2 and install dependencies declared by the packages in the
workspace:

``` bash
source /opt/ros/humble/setup.bash
cd ~/tmdriver_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 15. Build the Techman workspace

For the base driver workspace:

``` bash
source /opt/ros/humble/setup.bash
cd ~/tmdriver_ws
colcon build
```

If building the workspace with the MoveIt packages and following
Techman's MoveIt build instructions, use:

``` bash
source /opt/ros/humble/setup.bash
cd ~/tmdriver_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Do not run two builds simultaneously in the same workspace.

After a successful build:

``` bash
source ~/tmdriver_ws/install/setup.bash
```

Verify that ROS can find the driver package:

``` bash
ros2 pkg list | grep '^tm_driver$'
```

Expected: `tm_driver`.

Also check the TM5-700 MoveIt configuration package if it was built:

``` bash
ros2 pkg list | grep 'tm5-700'
```

The exact package list depends on the branch and successful build.

### Optional: make the Techman workspace available in new terminals

Add this line to `~/.bashrc` only after the workspace has built
successfully:

``` bash
echo "source ~/tmdriver_ws/install/setup.bash" >> ~/.bashrc
```

If you maintain multiple ROS workspaces or overlays, it may be safer to
source each workspace explicitly in the correct order rather than
automatically sourcing every workspace in `.bashrc`.

------------------------------------------------------------------------

# Part E --- Configure TMflow and the robot Ethernet connection

> **Do this only after the driver workspace builds.** The IP addresses
> below are examples from Techman's documentation, not guaranteed
> addresses for your robot. Read the actual IP address configured in
> your TMflow project/controller and use a free address in the same
> subnet for the Ubuntu computer.

## 16. Configure the TMflow Listen task

On the TMflow computer/controller:

1.  Open the TMflow project for the TM5-700.
2.  Create or open a flow project that contains a **Listen** node.
3.  Add the Listen node to the flow.
4.  Open **System → Network** and note the robot's wired IP address and
    subnet mask.
5.  In **Setting → Connection → Ethernet Slave**, open **Data Table
    Setting**.
6.  Set the communication mode to **BINARY**.
7.  Select the data items required by the Techman ROS driver. The
    upstream driver README lists the required input data table items,
    including robot error/state, project state, joint angle, flange/tool
    coordinates, TCP force/speed, joint speed/torque, project speed,
    mode, and digital/analog I/O.
8.  Enable Ethernet Slave after configuring its data table.
9.  Run the Listen task using the robot stick's Play/Pause button as
    required by the TMflow workflow.

**Important:** TMflow menus and supported data fields can vary by TMflow
version. Follow the current Techman driver documentation and the
instructions for the installed TMflow release.

Official driver instructions:
https://github.com/TechmanRobotInc/tmr_ros2

## 17. Configure Ubuntu's wired IPv4 address

Connect the Ubuntu computer to the robot controller using Ethernet.

In Ubuntu:

1.  Open **Settings → Network**.
2.  Under **Wired**, open the gear/settings icon.
3.  Select **IPv4**.
4.  Change IPv4 Method to **Manual**.
5.  Enter an unused address in the same subnet as the robot.
6.  Enter the matching subnet mask.
7.  Apply the settings and reconnect the wired connection.

Example only (use the real robot IP/subnet in your installation):

  Device                 Example IPv4 address           Netmask
  -------------------- ---------------------- -----------------
  TM5-700 controller           `192.168.10.2`   `255.255.255.0`
  Ubuntu control PC           `192.168.10.30`   `255.255.255.0`

The PC and robot must be on the same subnet, and their IP addresses must
not conflict.

## 18. Test Ethernet connectivity

Replace the example address with the actual TM5-700 controller IP:

``` bash
ping <ROBOT_IP>
```

For example:

``` bash
ping 192.168.10.2
```

A successful response confirms basic IP connectivity. Stop the ping with
`Ctrl + C`.

If ping fails, check the Ethernet cable, network interface, subnet mask,
IP address, TMflow Listen task, and Ethernet Slave status before
proceeding.

## 19. Launch the Techman driver

Open a terminal:

``` bash
source /opt/ros/humble/setup.bash
source ~/tmdriver_ws/install/setup.bash
ros2 run tm_driver tm_driver robot_ip:=<ROBOT_IP>
```

Example only:

``` bash
ros2 run tm_driver tm_driver robot_ip:=192.168.10.2
```

Keep this terminal running while using ROS nodes that depend on the
robot connection.

In another terminal, source the same environment and inspect ROS topics:

``` bash
source /opt/ros/humble/setup.bash
source ~/tmdriver_ws/install/setup.bash
ros2 topic list
```

Look for the robot state topics, including `/joint_states` where
published by the driver.

To inspect joint states:

``` bash
ros2 topic echo /joint_states
```

Stop the driver with `Ctrl + C` when finished.

------------------------------------------------------------------------

# Part F --- Install and build the book-retrieval application

## 20. Application workspace

The project application workspace is separate from the vendor driver
workspace:

``` text
~/fyp_phase3
```

The project packages used during development include:

``` text
tm5_book_msgs
tm5_book_retrieval
tm5_moveit_bridge
```

The Techman driver workspace provides the vendor driver and robot
description/configuration packages. The FYP workspace contains the
custom application.

> **Repository owner action required:** Replace
> `<YOUR_PROJECT_GITHUB_URL>` below with the actual public GitHub
> repository URL. The application repository must include the final
> versions of all required ROS packages, launch/configuration files,
> Arduino firmware, model assets (or documented release downloads), and
> Python dependency declarations. Do not expect cloning `tmr_ros2` alone
> to download the custom book-retrieval application.

Clone the application repository:

``` bash
git clone <YOUR_PROJECT_GITHUB_URL> ~/fyp_phase3
```

The intended package structure is:

``` text
~/fyp_phase3/
└── src/
    ├── tm5_book_msgs/
    ├── tm5_book_retrieval/
    └── tm5_moveit_bridge/
```

The actual repository may contain additional packages, launch files,
configuration, model assets, and firmware. Use the committed repository
tree as the source of truth.

## 21. Install MoveIt 2 and controller dependencies

The custom MoveIt bridge uses MoveIt 2. Install the Humble binary
packages used by the Techman MoveIt instructions:

``` bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster
```

Techman recommends Cyclone DDS for its MoveIt 2 workflow. Install it if
following that configuration:

``` bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

For terminals running this MoveIt application, select Cyclone DDS:

``` bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

This environment variable must be set in each terminal that needs the
same middleware configuration.

## 22. Install application dependencies

Install dependencies declared by the application packages:

``` bash
source /opt/ros/humble/setup.bash
source ~/tmdriver_ws/install/setup.bash
cd ~/fyp_phase3
rosdep install --from-paths src --ignore-src -r -y
```

The perception and gripper portions of the application also use Python
packages and hardware-specific libraries. The project repository should
provide a maintained `requirements.txt` (or equivalent environment file)
based on the imports in the final source files.

Create and activate a virtual environment if the project uses
pip-managed Python dependencies:

``` bash
cd ~/fyp_phase3
python3 -m venv ~/fyp_env
source ~/fyp_env/bin/activate
python -m pip install --upgrade pip
```

Install the project's pinned dependencies once `requirements.txt` has
been committed:

``` bash
python -m pip install -r requirements.txt
```

**Do not guess or install arbitrary versions of PyTorch, CUDA,
Ultralytics, RealSense, OpenCV, or NumPy.** These packages must be
pinned and tested together for the target Ubuntu, Python, and NVIDIA
driver combination. In particular, PyTorch CUDA compatibility depends on
the installed NVIDIA driver and the selected PyTorch build.

For Intel RealSense, install the appropriate librealsense SDK and Python
bindings using Intel's official instructions for Ubuntu 22.04:
https://github.com/IntelRealSense/librealsense

For Ultralytics YOLO, refer to: https://docs.ultralytics.com/

## 23. Build the application workspace

Build only after the Techman workspace and its required dependencies are
available:

``` bash
source /opt/ros/humble/setup.bash
source ~/tmdriver_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd ~/fyp_phase3
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

After a successful build:

``` bash
source ~/fyp_phase3/install/setup.bash
```

If source files or package manifests change, rebuild the workspace and
source its setup file again.

------------------------------------------------------------------------

# Part G --- Camera, gripper, and application configuration

## 24. Intel RealSense D435i

1.  Connect the D435i to a USB 3.x port.
2.  Install the matching Intel RealSense SDK and Python bindings using
    Intel's official instructions.
3.  Verify that the camera is detected using the RealSense viewer/tool
    supplied by the SDK.
4.  Confirm that the project perception node is configured for the
    correct camera device and stream profiles.
5.  Check camera-to-robot calibration and coordinate transforms before
    using detected 3D positions for robot motion.

Do not assume that a camera detected by Ubuntu is automatically
calibrated to the robot.

## 25. YOLO model and regression assets

The project uses YOLO oriented bounding-box detection for book-spine
detection and additional regression/model assets for estimating book
geometry/weight.

The final repository or release should provide all required model files,
for example:

``` text
best.pt
width_model.pkl
```

These are examples of model assets used during development; verify the
final filenames and paths in the committed source code.

Large model files may need Git LFS or GitHub Releases. Document the
exact download URL, version, checksum, and destination path for every
asset that is not committed directly.

## 26. Arduino mechanical gripper

The mechanical gripper uses an Arduino Uno and a servo. The ROS
application communicates with the Arduino over serial.

Before running the gripper node:

1.  Upload the final project firmware to the Arduino.
2.  Connect the Arduino to the Ubuntu computer.
3.  Identify the serial device:

``` bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

4.  Confirm the device path used by the project configuration.
5.  Ensure the servo uses an appropriate external power supply.
6.  Connect the power supply ground and Arduino ground as required by
    the circuit.
7.  Keep the robot in a safe state while testing the gripper
    independently.

The development configuration used `/dev/ttyACM0` and `115200` baud, but
the serial device name can change between computers. Check the actual
device path rather than assuming it is always `ttyACM0`.

The project ROS interface used these topics:

``` text
/tm5/gripper_command
/tm5/gripper_response
```

The serial protocol used during development included:

``` text
OPEN <mm>
CLOSE
STATUS
```

The final firmware and node source in the repository must be treated as
authoritative.

------------------------------------------------------------------------

# Part H --- Running the system

## 27. Terminal setup

Each new terminal must source the required environments. For the vendor
driver:

``` bash
source /opt/ros/humble/setup.bash
source ~/tmdriver_ws/install/setup.bash
```

For the custom application:

``` bash
source /opt/ros/humble/setup.bash
source ~/tmdriver_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If the Python perception/gripper nodes use the project virtual
environment:

``` bash
source ~/fyp_env/bin/activate
```

Use the launch files and executable names committed in the project
repository. Do not assume an executable name from a source filename;
verify with:

``` bash
ros2 pkg executables tm5_book_retrieval
ros2 pkg executables tm5_moveit_bridge
```

Inspect available launch files in the repository before running the
application:

``` bash
find ~/fyp_phase3/src -type f -name "*.launch.py"
```

The exact launch command for the final integrated application should be
added here after the final launch file and its parameters have been
audited and tested on the physical TM5-700.

------------------------------------------------------------------------

# Part I --- Safety and operational checks

## 28. Before enabling robot motion

-   Confirm the robot model is TM5-700 and the correct robot IP is
    configured.
-   Confirm the robot is in the expected operating mode and no safety
    faults are active.
-   Confirm the TMflow Listen task is running.
-   Confirm Ethernet Slave is enabled and its data table is configured.
-   Confirm `ping <ROBOT_IP>` succeeds.
-   Confirm the ROS driver connects without errors.
-   Confirm the correct MoveIt planning group, end-effector, TCP,
    frames, and robot calibration are loaded.
-   Clear the robot's working area and keep people outside the motion
    envelope.
-   Keep the emergency stop accessible and know how to use it.
-   Test with low speed and conservative motion limits in a controlled
    area.
-   Verify all trajectories and end-effector actions before running an
    automated retrieval cycle.

**A successful ROS connection does not prove that a planned trajectory
is safe.** Never launch a real-robot MoveIt demo or application motion
sequence until its target, frames, velocity/acceleration limits,
collision geometry, and recovery behavior have been checked.

To stop a running ROS node/driver, use `Ctrl + C` in its terminal. Use
the robot's physical emergency stop if an unsafe movement or emergency
occurs.

------------------------------------------------------------------------

# Part J --- Troubleshooting

## 29. `ros2: command not found`

Source ROS Humble:

``` bash
source /opt/ros/humble/setup.bash
```

Check:

``` bash
echo $ROS_DISTRO
```

Expected: `humble`.

## 30. `colcon: command not found`

Install the colcon extensions:

``` bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```

Check:

``` bash
colcon --help
```

## 31. `rosdep: command not found`

Install rosdep:

``` bash
sudo apt install -y python3-rosdep
```

Then:

``` bash
rosdep update
```

## 32. ROS cannot find `tm_driver`

Source both the ROS installation and the built Techman workspace:

``` bash
source /opt/ros/humble/setup.bash
source ~/tmdriver_ws/install/setup.bash
```

Check:

``` bash
ros2 pkg list | grep '^tm_driver$'
```

If the package is absent, inspect the build output and confirm the
workspace completed successfully.

## 33. Robot IP cannot be reached

Check:

-   Ethernet cable and adapter.
-   Ubuntu wired interface is connected.
-   Ubuntu and robot use compatible IP addresses and subnet masks.
-   No duplicate IP addresses exist.
-   TMflow Listen task is running.
-   Ethernet Slave is enabled.
-   The robot controller's configured IP is correct.

Use the actual controller IP:

``` bash
ping <ROBOT_IP>
```

## 34. ROS driver starts but does not connect

Check the TMflow Listen node, Ethernet Slave status, data table
settings, communication mode, and the IP address passed to `robot_ip`.
Compare the setup against the current upstream Techman driver README.

## 35. New terminal cannot find project packages

Source the overlays in order:

``` bash
source /opt/ros/humble/setup.bash
source ~/tmdriver_ws/install/setup.bash
source ~/fyp_phase3/install/setup.bash
```

------------------------------------------------------------------------

# Part K --- Repository contents required for a reproducible setup

## 36. Files that should be included in this GitHub project

To allow another developer to clone and run the complete application,
the repository should contain:

-   `tm5_book_msgs` package and message definitions.
-   `tm5_book_retrieval` package and all runtime Python modules.
-   `tm5_moveit_bridge` package, C++ source, and build metadata.
-   Launch files and robot/application configuration.
-   Robot home and shelf-location configuration.
-   Coordinate transform and grasp-target logic.
-   Final Arduino firmware for the gripper.
-   A pinned `requirements.txt` or equivalent Python environment
    definition.
-   A `.gitignore` for build products, logs, virtual environments,
    caches, and local secrets.
-   Model weights or versioned download instructions with checksums.
-   Camera calibration and robot/tool calibration files required at
    runtime.
-   A documented example configuration file for network settings (never
    commit passwords or private credentials).

Do not commit generated workspace folders such as:

``` text
build/
install/
log/
.venv/
__pycache__/
```

Do not commit personal backups, temporary debug files, local datasets,
or private machine-specific settings unless they are intentionally part
of the released project.

## 37. Suggested repository layout

``` text
TM5-700-Book-Retrieval/
├── README.md
├── requirements.txt
├── .gitignore
├── docs/
│   ├── hardware_setup.md
│   ├── tmflow_network_setup.md
│   ├── calibration.md
│   └── operation.md
├── firmware/
│   └── gripper_arduino/
├── models/
│   └── README.md
└── ros2_ws/
    └── src/
        ├── tm5_book_msgs/
        ├── tm5_book_retrieval/
        └── tm5_moveit_bridge/
```

This is a suggested clean repository layout. If the existing GitHub
repository uses another layout, update the clone/build commands to match
the actual committed tree.

------------------------------------------------------------------------

## 38. Official references

-   Ubuntu Desktop installation:
    https://ubuntu.com/tutorials/install-ubuntu-desktop
-   Ubuntu 22.04 release downloads: https://releases.ubuntu.com/22.04/
-   Rufus: https://rufus.ie/
-   ROS 2 Humble documentation: https://docs.ros.org/en/humble/
-   ROS 2 Humble Ubuntu installation:
    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
-   colcon installation:
    https://colcon.readthedocs.io/en/released/user/installation.html
-   Techman ROS 2 driver (`humble` branch):
    https://github.com/TechmanRobotInc/tmr_ros2
-   Intel RealSense SDK: https://github.com/IntelRealSense/librealsense
-   Ultralytics documentation: https://docs.ultralytics.com/

------------------------------------------------------------------------

## 39. Final verification checklist

Before calling the setup complete, verify each item on the target
computer:

-   [ ] Windows and Ubuntu both boot successfully.
-   [ ] Ubuntu 22.04 is installed.
-   [ ] `echo $ROS_DISTRO` returns `humble`.
-   [ ] ROS talker/listener test works.
-   [ ] `colcon --help` works.
-   [ ] `~/tmdriver_ws` builds successfully.
-   [ ] `ros2 pkg list` contains `tm_driver`.
-   [ ] The TM5-700 Listen task and Ethernet Slave are configured.
-   [ ] Ubuntu can ping the robot controller.
-   [ ] `ros2 run tm_driver tm_driver robot_ip:=<ROBOT_IP>` connects.
-   [ ] `/joint_states` is available when the driver is running.
-   [ ] The FYP application workspace builds successfully.
-   [ ] The RealSense D435i is detected and calibrated.
-   [ ] The YOLO and regression model assets load successfully.
-   [ ] The Arduino gripper responds to its ROS serial node.
-   [ ] The integrated application has been tested safely on the
    physical robot.

**Only mark the complete installation as verified after these checks
have actually passed on a clean installation.**
