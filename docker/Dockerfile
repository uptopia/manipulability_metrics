
FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04

############################## SYSTEM PARAMETERS ##############################
# * Arguments
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID="${UID}"
ARG SHELL=/bin/bash
ARG HARDWARE=x86_64
ARG ENTRYPOINT_FILE=entrypint.sh

# # * Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# * Setup users and groups
RUN groupadd --gid "${GID}" "${GROUP}" \
    && useradd --gid "${GID}" --uid "${UID}" -ms "${SHELL}" "${USER}" \
    && mkdir -p /etc/sudoers.d \
    && echo "${USER}:x:${UID}:${UID}:${USER},,,:/home/${USER}:${SHELL}" >> /etc/passwd \
    && echo "${USER}:x:${UID}:" >> /etc/group \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${USER}" \
    && chmod 0440 "/etc/sudoers.d/${USER}"

# * Replace apt urls
# ? Change to tku
RUN sed -i 's@archive.ubuntu.com@ftp.tku.edu.tw@g' /etc/apt/sources.list
# ? Change to Taiwan
# RUN sed -i 's@archive.ubuntu.com@tw.archive.ubuntu.com@g' /etc/apt/sources.list

# * Time zone
ENV TZ=Asia/Taipei
RUN ln -snf /usr/share/zoneinfo/"${TZ}" /etc/localtime && echo "${TZ}" > /etc/timezone

# * Copy custom configuration
# ? Requires docker version >= 17.09
COPY --chmod=0775 ./${ENTRYPOINT_FILE} /entrypoint.sh
COPY --chown="${USER}":"${GROUP}" --chmod=0775 config config
# ? docker version < 17.09
# COPY ./${ENTRYPOINT_FILE} /entrypoint.sh
# COPY config config
# RUN sudo chmod 0775 /entrypoint.sh && \
    # sudo chown -R "${USER}":"${GROUP}" config \
    # && sudo chmod -R 0775 config

############################### INSTALL #######################################

# * Install packages
RUN apt update \
    && apt install -y --no-install-recommends \
        sudo \
        vim \
        git \
        htop \
        wget \
        curl \
        psmisc \
        # * Shell
        tmux \
        terminator \
        # * base tools
        python3-pip \
        python3-dev \
        python3-apt \
        python3-setuptools \
        python3-tk \ 
        # * tools
        cmake \
        lsb-release \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

RUN ./config/pip/pip_setup.sh

# * Install ROS noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && apt update \
    && dpkg --configure -a \
    && apt install -y -f --fix-broken --no-install-recommends ros-noetic-desktop-full \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO noetic

RUN mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
        tee /etc/apt/sources.list.d/librealsense.list \
    && apt-get update

RUN apt update \
    && apt install -y --no-install-recommends \
        # * ROS depend tools
        ros-$ROS_DISTRO-moveit \        
        ros-$ROS_DISTRO-moveit-visual-tools \
        ros-$ROS_DISTRO-catkin \
        ros-$ROS_DISTRO-ddynamic-reconfigure \
        ros-$ROS_DISTRO-franka-description \
        ros-$ROS_DISTRO-tf2-sensor-msgs \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>/home/$USER/.bashrc

WORKDIR /

############################## USER CONFIG ####################################
# * Switch user to ${USER}
USER ${USER}

RUN ./config/shell/bash_setup.sh "${USER}" "${GROUP}" \
    && ./config/shell/terminator/terminator_setup.sh "${USER}" "${GROUP}" \
    && ./config/shell/tmux/tmux_setup.sh "${USER}" "${GROUP}" \
    && sudo rm -rf /config

# * Create work folder
RUN sudo mkdir -p /home/"${USER}"/work
WORKDIR /home/"${USER}"/work

# * Make SSH available
EXPOSE 22

ENTRYPOINT [ "/entrypoint.sh", "terminator" ]
# ENTRYPOINT [ "/entrypoint.sh", "tmux" ]
# ENTRYPOINT [ "/entrypoint.sh", "bash" ]
# ENTRYPOINT [ "/entrypoint.sh" ]
