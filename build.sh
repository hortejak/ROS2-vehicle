#!/bin/bash

echo "Proceeding to build ros2 ws.";

colcon build --symlink-install;

echo "WS built.";

source ./install/setup.bash;

echo "source finished";