#!/bin/sh


CONTAINER=cicd_ros2_webots:latest

docker build -t ${CONTAINER} -f Dockerfile . 
