#!/bin/bash
PIDS=$(cat /sys/fs/cgroup/memory/salt/docking/cgroup.procs)
for pid in $PIDS; do
    [ -d /proc/$pid ] && kill -INT $pid
done
sleep 1
# Make second loop and hard kill any remaining
for pid in $PIDS; do
    [ -d /proc/$pid ] && kill -KILL $pid
done

