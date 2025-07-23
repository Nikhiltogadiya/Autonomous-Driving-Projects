#!/bin/sh

WEBOTS_HOME=~/.ros/webotsR2022a/webots
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/projects/default/resources/sumo/bin:$WEBOTS_HOME/lib

python3 $WEBOTS_HOME/resources/sumo_exporter/exporter.py --input=tesla_world.wbt --output=.
$WEBOTS_HOME/projects/default/resources/sumo/bin/netconvert --node-files=sumo.nod.xml --edge-files=sumo.edg.xml --output-file=sumo.net.xml