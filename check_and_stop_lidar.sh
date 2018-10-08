#!/bin/bash
PROCESS="roslaunch.*minimal"
PIDS=`ps -ef | grep -e $PROCESS | grep -v "grep" | awk '{print $2}'`
if [ -z "$PIDS" ]; then
  echo "Minimal not running." 1>&2
  if [ -f /tmp/minimal.pid ]; then 
#   remove pid file
    echo "Waiting another call to shut down lidar motor." 1>&2
    rm -f /tmp/minimal.pid
  else 
#   if file is not there then we can shutdown the motor
    echo "Shutting down motor." 1>&2
    source /home/${USER}/catkin_ws/devel/setup.sh
    rosservice call "/stop_motor"
  fi
  exit 1
else
  echo "Minimal running." 1>&2
  for PID in $PIDS; do
    echo "$PID" > /tmp/minimal.pid
  done
fi
