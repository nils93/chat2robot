# Base image
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# TurtleBot3 model
ENV TURTLEBOT3_MODEL=burger

# ---------- User ----------
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME && \
    apt update && apt install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# ============================================================
# === APT-Pakete installieren aus apt-packages.txt ===========
# ============================================================
COPY apt-packages.txt /tmp/apt-packages.txt

RUN sudo apt-get update && \
    grep -Ev '^\s*#' /tmp/apt-packages.txt | grep -Ev '^\s*$' | \
    xargs sudo apt-get install -y --no-install-recommends && \
    sudo rm -rf /var/lib/apt/lists/*

# ============================================================
# === Python-Pakete installieren aus requirements.txt ========
# ============================================================
COPY requirements.txt /tmp/requirements.txt

RUN pip install --no-cache-dir -r /tmp/requirements.txt

# ---------- ROS sourcing ----------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/$USERNAME/ros2_ws/src && \
    echo 'if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi' >> ~/.bashrc

WORKDIR /home/$USERNAME/ros2_ws
CMD ["/bin/bash"]
