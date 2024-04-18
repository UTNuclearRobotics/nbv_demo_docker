FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-c"]

# set the ros version
ENV ROS_DISTRO noetic

# install some useful packages and upgrade existing ones
RUN apt update && apt upgrade -y \
    && apt install -y \
    x11-apps \
    python3-pip \
    python3-catkin-tools \
    python-is-python3

# install ros packages
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-python \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros-numpy \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-trac-ik-python \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip install \
    open3d \
    ipdb \
    numba \ 
    transforms3d

RUN pip install --upgrade numpy

# switch to home dir
WORKDIR /root

# setup .bashrc
SHELL ["/bin/bash", "-l", "-c"]
RUN echo -e "\
    source /opt/ros/${ROS_DISTRO}/setup.bash\n\
    export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/amrl_msgs" >> ~/.bashrc \
    && echo -e "\
    source /opt/ros/${ROS_DISTRO}/setup.bash\n\
    export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/amrl_msgs" >> ~/.profile

# Run a command to get the computer's IP address
# RUN IP_ADDRESS=$(ip addr show wlp0s20f3 | awk '$1 == "inet" {gsub(/\/.*$/, "", $2); print $2}')

# Set environment variable with the extracted IP address
# ENV ROS_IP=$IP_ADDRESS
# ENV ROS_MASTER_URI=192.168.11.151:11311

# copy the entrypoint into the image
COPY ./entrypoint.sh /entrypoint.sh
# run this script on startup
ENTRYPOINT /entrypoint.sh