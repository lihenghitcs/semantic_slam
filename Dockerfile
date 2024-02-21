FROM nvidia/cuda:11.4.3-base-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN useradd -ms /bin/bash hit

WORKDIR /home/hit/liheng

RUN apt-get update &&\
    apt-get install -y curl

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN curl -s https://blog.zireaels.com/static/upload/20240221/ros.asc | apt-key add -
    
RUN apt-get update && \
    apt-get install --fix-missing -y --no-install-recommends \
        cuda-toolkit-11-4 
RUN apt-get update && \
    CUDNN_VERSION=$(apt-cache madison libcudnn8 | grep "cuda11.4" | head -n 1 | awk '{print $3}') && \
    apt-get install --fix-missing -y --no-install-recommends \
        libcudnn8=$CUDNN_VERSION
RUN apt-get update && \
    apt-get install --fix-missing -y --no-install-recommends \
        ros-noetic-desktop-full

USER hit

COPY . /home/hit/liheng/semantic_ws/src/semantic_slam
