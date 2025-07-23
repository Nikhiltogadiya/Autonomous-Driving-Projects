FROM ubuntu:22.04 

ARG TARGETARCH=amd64

ENV DEBIAN_FRONTEND=noninteractive

USER root

RUN apt-get update && apt-get install -y --no-install-recommends\
        mc \
        vim \
        gdb \
        build-essential \
	psmisc \
     	python-is-python3 \
	python3-pip \
        git \
        zip \
        wget \
 	patch \
        cmake \ 
        gnupg \
	less \
	&& \
    rm -rf /var/lib/apt/lists/*  \
    && apt-get clean

RUN apt-get update && apt-get install -y --no-install-recommends\
         libnvidia-gl-535 \
         libnvidia-compute-535 \
        nvidia-utils-535 \
        pkg-config \
        xorg \
        xcvt \
         mesa-vulkan-drivers \
         libvulkan1 \
         vulkan-tools \
        && \
    rm -rf /var/lib/apt/lists/*  \
    && apt-get clean

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf && \
    # Configure OpenCL manually
    mkdir -pm755 /etc/OpenCL/vendors && echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd && \
    # Configure Vulkan manually
    VULKAN_API_VERSION=$(dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9]+(\.[0-9]+)(\.[0-9]+)') && \
    mkdir -pm755 /etc/vulkan/icd.d/ && echo "{\n\
    \"file_format_version\" : \"1.0.0\",\n\
    \"ICD\": {\n\
        \"library_path\": \"libGLX_nvidia.so.0\",\n\
        \"api_version\" : \"${VULKAN_API_VERSION}\"\n\
    }\n\
}" > /etc/vulkan/icd.d/nvidia_icd.json && \
    # Configure EGL manually
    mkdir -pm755 /usr/share/glvnd/egl_vendor.d/ && echo "{\n\
    \"file_format_version\" : \"1.0.0\",\n\
    \"ICD\": {\n\
        \"library_path\": \"libEGL_nvidia.so.0\"\n\
    }\n\
}" > /usr/share/glvnd/egl_vendor.d/10_nvidia.json


# Make all NVIDIA GPUs visible by default
ENV NVIDIA_VISIBLE_DEVICES all
# All NVIDIA driver capabilities should preferably be used, check `NVIDIA_DRIVER_CAPABILITIES` inside the container if things do not work
ENV NVIDIA_DRIVER_CAPABILITIES all
# Disable VSYNC for NVIDIA GPUs
ENV __GL_SYNC_TO_VBLANK 0


RUN wget -O /etc/apt/trusted.gpg.d/ros2.gpg https://raw.githubusercontent.com/ros/rosdistro/master/ros.key && \
        echo "deb [arch=${TARGETARCH} signed-by=/etc/apt/trusted.gpg.d/ros2.gpg] http://packages.ros.org/ros2/ubuntu jammy main">/etc/apt/sources.list.d/ros2.list && \
   wget -O - https://cyberbotics.com/Cyberbotics.asc |apt-key add - && \
        echo "deb https://cyberbotics.com/debian/ binary-${TARGETARCH}/">/etc/apt/sources.list.d/webots.list 

RUN apt-get update && apt-get install -y --no-install-recommends\
         python3-yapf \
         yapf3 \
         clang-format-14 \
         python3-pycodestyle \
         ros-humble-desktop \
         python3-colcon-* \
         webots=2023b \
         sumo=1.12.0+dfsg1-1 \
         sumo-tools=1.12.0+dfsg1-1 \
         ffmpeg \
         libaribb24-0 \
         ros-humble-webots-ros2 \
	 mesa-utils \
        && \
    rm -rf /var/lib/apt/lists/*  \
    && apt-get clean

ENV WEBOTS_HOME=/usr/local/webots
ENV SUMO_HOME=/usr/share/sumo

RUN echo 'echo "ros2 sourced"; source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc

RUN pip3 install --no-cache-dir geographiclib glfw networkx

RUN groupadd -g 1000 webots && useradd -Ms /bin/bash -u 1000 -g 1000 -d /workspace webots && mkdir /workspace && chown 1000.1000 /workspace

WORKDIR /workspace
USER 1000
