# ──────────────────────────────────────────────
# Base image: ROS 2 Humble (Ubuntu 22.04)
# ──────────────────────────────────────────────
FROM ros:humble-ros-base

# Avoid interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# Add metadata
LABEL maintainer="FSLART Team" version="1.0"

# ──────────────────────────────────────────────
# Install git and build tools
# ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    && rm -rf /var/lib/apt/lists/*

# ──────────────────────────────────────────────
# Set up workspace
# ──────────────────────────────────────────────
WORKDIR /ros2_ws/src

# ──────────────────────────────────────────────
# Clone and build lart dependencies first
# ──────────────────────────────────────────────
RUN git clone -b dev https://github.com/FSLART/lart_msgs.git && \
    git clone https://github.com/FSLART/lart_common.git

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select lart_msgs lart_common"

# ──────────────────────────────────────────────
# Now copy p-puma source (after lart_msgs is cached)
# ──────────────────────────────────────────────
COPY . /ros2_ws/src/p-puma

# ──────────────────────────────────────────────
# Install ROS dependencies from package.xml files
# ──────────────────────────────────────────────
RUN /bin/bash -c "apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths /ros2_ws/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*"

# ──────────────────────────────────────────────
# Build p-puma
# ──────────────────────────────────────────────
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    colcon build --packages-select p-puma"

# ──────────────────────────────────────────────
# Clean up build artifacts to reduce image size
# ──────────────────────────────────────────────
# RUN rm -rf /ros2_ws/build /ros2_ws/log

# ──────────────────────────────────────────────
# Source workspace and run on container start
# ──────────────────────────────────────────────
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "p-puma", "puma.launch.py"]