#!/bin/bash
PIDS=$(ps -eo pid,command | grep videoStream | grep -v grep | awk '{print $1}')
for pid in $PIDS; do
    [ -d /proc/$pid ] && kill -9 $pid
done

