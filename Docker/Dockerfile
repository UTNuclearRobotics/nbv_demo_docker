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

# setup a package from github
# RUN git clone https://github.com/ut-amrl/amrl_msgs.git
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
#     && cd amrl_msgs \
#     && export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH \
#     && make -j

# setup .bashrc
SHELL ["/bin/bash", "-l", "-c"]
RUN echo -e "\
    source /opt/ros/${ROS_DISTRO}/setup.bash\n\
    export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/amrl_msgs" >> ~/.bashrc \
    && echo -e "\
    source /opt/ros/${ROS_DISTRO}/setup.bash\n\
    export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/amrl_msgs" >> ~/.profile

# copy the entrypoint into the image
COPY ./entrypoint.sh /entrypoint.sh

# run this script on startup
ENTRYPOINT /entrypoint.sh