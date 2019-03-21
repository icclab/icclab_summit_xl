#!/bin/bash
#
# extract logs of movebase_client_py node to see test results

. /opt/ros/kinetic/setup.bash
roscd log
cat *movebase_client_py*

exit 0
