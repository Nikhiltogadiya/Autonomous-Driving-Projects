#!/bin/sh


xhost +
docker run --rm --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v `pwd`:/workspace cicd_ros2_webots:latest
