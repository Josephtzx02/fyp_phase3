# fyp_phase3
3D scene analysis and robotic vision system for autonomous book retrieval using a Techman TM5-700 robotic arm.

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')


Next step — download the correct .deb
Run this exactly:
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

This is the same command currently specified by the official ROS documentation. GitHub
Then STOP and check the download
Run:
ls -lh /tmp/ros2-apt-source.deb

We want to see a non-zero file size.

Then:
sudo dpkg -i /tmp/ros2-apt-source.deb


ls -lh /tmp/ros2-apt-source.deb

#Expect -rw-r--r-- 1 root root 5.0K ... /tmp/ros2-apt-source.deb
