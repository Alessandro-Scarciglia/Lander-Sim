FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/workspace/touchdown_ws
ENV CAD_DIR=/workspace/CAD

# Aggiornamenti e pacchetti base
RUN apt-get update && apt-get upgrade -y

# Installazione Gazebo e strumenti ROS necessari
RUN apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    git \
    wget \
    build-essential \
    python3-pip \
    python3-matplotlib \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir "trimesh<4" "shapely<2" rtree

# Copia del workspace
COPY touchdown_ws ${ROS_WS}

# Directory di lavoro
WORKDIR ${ROS_WS}

# Build del workspace ROS 2
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Setup automatico
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=${ROS_WS}/src/worlds/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc

# Comando default
CMD ["/bin/bash"]
