# FROM nvidia/cudagl:11.4.0-base-ubuntu20.04
FROM osrf/ros:noetic-desktop-full

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND=noninteractive

#####################################################
# Install common apt packages
#####################################################
RUN apt-get update && apt-get install -y \
	### utility
	locales \
	xterm \
	dbus-x11 \
	terminator \
	sudo \
	### tools
	unzip \
	lsb-release \
	curl \
	ffmpeg \
	net-tools \
	software-properties-common \
	subversion \
	libssl-dev \
	### Development tools
	build-essential \
	htop \
	git \
	vim \
	gedit \
	gdb \
	valgrind \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*


#####################################################
# Set locale & time zone
#####################################################
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8
ENV TZ=Asia/Tokyo


#####################################################
# Python 3.8
#####################################################
RUN apt-get update && \
	apt-get install -y wget && \
	wget https://bootstrap.pypa.io/get-pip.py && \
	python3.8 get-pip.py


#####################################################
# ROS
#####################################################
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
	libfcl* \
	libglew-dev \
	ros-noetic-desktop-full \
	ros-noetic-joy \
	ros-noetic-gazebo* \
	ros-noetic-moveit* \
	ros-noetic-image-view* \
	ros-noetic-cv-camera* \
	ros-noetic-joint* \
	ros-noetic-graph* \
	ros-noetic-ros-controller* \
	ros-noetic-joy-teleop* \
	ros-noetic-eigen* \
	ros-noetic-rosbridge-server* \
	ros-noetic-geometric* \
	ros-noetic-object-recognition* \
	ros-noetic-map-server* \
	ros-noetic-warehouse-ros* \
	ros-noetic-rosserial \
	ros-noetic-ros-numpy \
	ros-noetic-geodesy && \
	apt-get clean && rm -rf /var/lib/apt/lists/*

RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /home/tomoya-y/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc
RUN echo "export ROS_IP=localhost" >> /root/.bashrc
RUN echo "alias cm='cd /home/tomoya-y/catkin_ws && catkin_make'" >> /root/.bashrc


#####################################################
# camera connection
#####################################################
RUN apt-get update && apt-get install -y v4l-utils


#####################################################
# Run scripts (commands)
#####################################################

### terminator window settings
COPY assets/config /

### user group settings
COPY assets/entrypoint_setup.sh /
ENTRYPOINT ["/entrypoint_setup.sh"] /

# Run terminator
CMD ["terminator"]
