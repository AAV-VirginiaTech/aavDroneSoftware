FROM osrf/ros:humble-desktop-full

RUN apt update && apt install -y --no-install-recommends \
    openssh-server \
    xauth \
    snapd \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    git \
    curl \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

RUN echo root:robotics | chpasswd

RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && \
    echo "X11UseLocalhost no" >> /etc/ssh/sshd_config && \
    echo "PermitUserEnvironment yes" >> /etc/ssh/sshd_config && \
    mkdir -p /root/.ssh && \
    touch /root/.ssh/environment

RUN echo "Port 2201" >> /etc/ssh/sshd_config

COPY ssh_entry.sh /root/.bash_profile
COPY ros_bashrc.sh /root/ros_bashrc.sh
RUN echo "source /root/ros_bashrc.sh" >> /root/.bashrc

ENTRYPOINT service ssh restart && bash
