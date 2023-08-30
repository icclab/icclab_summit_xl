#!/bin/bash
#
# sync clock of robot
#
# requires ssh public key based authenication (https://www.cyberciti.biz/tips/ssh-public-key-based-authentication-how-to.html)

USERNAME=summit
HOSTNAME=SXLS0-180227AA

echo "Checking SSH connection to $USERNAME@$HOSTNAME..."
status=$(ssh -o BatchMode=yes -o ConnectTimeout=5 $USERNAME@$HOSTNAME echo ok 2>&1)

if [[ $status == ok ]] ; then
  echo Authentication ok

  # auth is ok so sync clock on robot
  echo "Type password for $USERNAME robot, followed by [ENTER]:"
  read -s PASSWORD
  time=$(date +%T)
  echo "Time on this machine: $time"
  echo "Syncing time of $USERNAME robot..."
  time=$(date +%T) # get a more recent reading
  ssh -l $USERNAME $HOSTNAME "echo $PASSWORD | sudo -S date -s "$time"" 

elif [[ $status == "Permission denied"* ]] ; then
  echo Permission denied, do you have key based authenication?
else
  echo Error, check network connection
fi

