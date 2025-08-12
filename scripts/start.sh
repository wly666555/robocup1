#!/bin/bash

cd `dirname $0`
cd ..

./scripts/stop.sh

source ./install/setup.bash

nohup ros2 launch vision launch.py > vision.log 2>&1 &
nohup ros2 launch brain launch.py "$@" > brain.log 2>&1 &