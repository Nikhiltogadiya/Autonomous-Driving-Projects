#!/bin/bash

source install/local_setup.sh

ros2 launch robot_launch local_launch.py testdefinition:=src/test.json testsuite:=testsuite-1 testcase:=test-2
