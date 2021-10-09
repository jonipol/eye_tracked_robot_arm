FROM osrf/ros:foxy-desktop

# Fix broken keys https://answers.ros.org/question/379190/apt-update-signatures-were-invalid-f42ed6fbab17c654/
RUN curl http://repo.ros2.org/repos.key | sudo apt-key add -

RUN apt-get update -y && apt-get install -y \
    git \
    nano \
    wget \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update -y && \
    apt-get install -y \
    gazebo11 \
    libgazebo11-dev

RUN apt-get update && apt-get install -y \
    ros-foxy-gazebo-ros \
    ros-foxy-gazebo-plugins \
    ros-foxy-xacro && \
    apt-get dist-upgrade -y && \
    apt-get clean

RUN pip3 install --upgrade pip
RUN pip3 install icecream numpy opencv-python

ENV BASE_WS=/workspace/eye_tracked_arm
ENV PACKAGE_WS=$BASE_WS/package_ws
ENV ROBOT_WS=$BASE_WS/robot_ws
ENV TOBII_SDK=$BASE_WS/tobii_sdk

RUN mkdir -p $PACKAGE_WS/src && \
    mkdir -p $ROBOT_WS/src

ENV UR_WS=$BASE_WS/ur_ws
RUN mkdir -p $UR_WS/src

RUN cd $UR_WS/src  && \
    git clone -b foxy --single-branch https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git Universal_Robots_ROS2_Driver  && \
    git clone https://github.com/ros-controls/control_toolbox.git

COPY ur_ws/src $UR_WS/src
RUN cd $UR_WS && \
    vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos && \
    vcs import src --skip-existing --input $UR_WS/src/moveit_support.repos && \
    vcs import src --skip-existing --input src/moveit2/moveit2.repos && \
    . /opt/ros/foxy/setup.sh && \
    rosdep install --ignore-src --from-paths src -y -r && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


RUN apt-get update -y && apt-get install -y \
    build-essential \
    cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
    python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

RUN mkdir /opencv && \
    cd /opencv && \
    git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git

RUN cd /opencv/opencv && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -OPENCV_EXTRA_MODULES_PATH=/opencv/opencv_contrib/modules/ .. && \
    make -j7 && \
    make install

RUN apt install -y libcanberra-gtk-module libcanberra-gtk3-module

COPY tobii_sdk/ $TOBII_SDK/
RUN cd $TOBII_SDK && \
    make install

COPY package_ws/src $PACKAGE_WS/src
RUN cd $PACKAGE_WS/src && \
    vcs import < eye.repos && \
    cd $PACKAGE_WS && \
    . /opt/ros/foxy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

COPY robot_ws/src $ROBOT_WS/src
RUN cd $ROBOT_WS && \
    . /opt/ros/foxy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

ENV ENTRYPOINT_PATH=docker/ros_entrypoint.sh
COPY $ENTRYPOINT_PATH /ros_entrypoint.sh

# Needed so that exec always sources ros_entrypoint
RUN echo 'source /opt/ros/foxy/setup.sh' >> ~/.bashrc && \
    echo '. $PACKAGE_WS/install/setup.sh' >> ~/.bashrc && \
    echo '. $ROBOT_WS/install/setup.sh' >> ~/.bashrc && \
    echo '. $UR_WS/install/setup.sh' >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
