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
RUN apt-get update

RUN apt-get install -y libfcl*
RUN apt-get install -y libglew-dev
RUN apt-get install -y ros-noetic-desktop-full
RUN apt-get install -y ros-noetic-joy
# RUN apt-get install -y ros-noetic-gazebo*
# RUN apt-get install -y ros-noetic-moveit*
RUN apt-get install -y ros-noetic-image-view*
RUN apt-get install -y ros-noetic-cv-camera*
RUN apt-get install -y ros-noetic-joint*
RUN apt-get install -y ros-noetic-graph*
RUN apt-get install -y ros-noetic-ros-controller*
RUN apt-get install -y ros-noetic-joy-teleop*
RUN apt-get install -y ros-noetic-eigen*
RUN apt-get install -y ros-noetic-rosbridge-server*
RUN apt-get install -y ros-noetic-geometric*
RUN apt-get install -y ros-noetic-object-recognition*
RUN apt-get install -y ros-noetic-map-server*
RUN apt-get install -y ros-noetic-warehouse-ros*
RUN apt-get install -y ros-noetic-rosserial
RUN apt-get install -y ros-noetic-ros-numpy
RUN apt-get install -y ros-noetic-geodesy
RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*

RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /home/tomoya-y/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc
RUN echo "export ROS_IP=localhost" >> /root/.bashrc
RUN echo "alias cm='cd /home/tomoya-y/catkin_ws && catkin_make'" >> /root/.bashrc
RUN echo "alias devices='v4l2-ctl --list-devices'" >> /root/.bashrc

RUN echo "alias roscon=' \
		sudo chmod 666 /dev/ttyACM0 && \
		sudo chmod 666 /dev/video0 && \
		roslaunch ~/catkin_ws/src/image_viewers.launch'" >> /root/.bashrc


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
# CMD ["terminator"]
CMD ["/bin/bash"]
