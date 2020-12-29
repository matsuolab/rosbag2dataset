FROM ros:noetic-ros-core

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        libgl1-mesa-dev \
        ros-noetic-cv-bridge \
        ros-noetic-tf* && \
    apt-get clean &&  \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir \
        torch \
        opencv-python \
        tqdm

RUN apt-get update && \
    apt-get install -y \
    wget \
    curl \
    git \
    vim \
    lsb-release \
    gnupg

RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc

WORKDIR /root
CMD ["/bin/bash"]