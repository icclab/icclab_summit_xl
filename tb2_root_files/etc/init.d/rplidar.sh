#!/bin/sh
#
# rplidar          Start/Stop rplidar script.
#
### BEGIN INIT INFO
# Provides:          rplidar
# Required-Start:    $all
# Required-Stop:     $all
# Default-Start:     2 3 4 5 
# Default-Stop:      0 1 6
# Short-Description: Start rplidar ros node
# Description:       Stops motor of lidar and starts rplidar ros node.
### END INIT INFO
#
# chkconfig: 2345 90 60
# description: This is a simple service script that was made to demonstrate \
# how to write SysVinit scripts to 'daemonize' programs.
#
# The daemon's name (to ensure uniqueness and for stop, restart and status)
name="rplidar"  
# The path of the client executable
command="/etc/udev/scripts/rplidar.sh"  
# Any command line arguments for the client executable
command_args=""  
# The path of the daemon executable
daemon="/usr/bin/daemon"

[ -x "$daemon" ] || exit 0
[ -x "$command" ] || exit 0

# Note: The following daemon option arguments could be in /etc/daemon.conf
# instead. That would probably be better because if the command itself were
# there as well then we could just use the name here to start the daemon.
# Here's some code to do it here in case you prefer that.

# Any command line arguments for the daemon executable (when starting)
daemon_start_args="" # e.g. --inherit --env="ENV=VAR" --unsafe  
# The pidfile directory (need to force this so status works for normal users)
pidfiles="/var/run/rplidar"  
# The user[:group] to run as (if not to be run as root)
# user="turtlebot"  
user=`id -nu 1000`
# The path to chroot to (otherwise /)
chroot=""  
# The path to chdir to (otherwise /)
chdir=""  
# The umask to adopt, if any
umask=""  
# The syslog facility or filename for the client's stdout (otherwise discarded)
stdout="daemon.info"  
# The syslog facility or filename for the client's stderr (otherwise discarded)
stderr="daemon.err"

case "$1" in  
    start)
        # This if statement isn't strictly necessary but it's user friendly
        if "$daemon" --running --name "$name" --pidfiles "$pidfiles"
        then
            echo "$name is already running."
        else
            echo -n "Starting $name..."
            mkdir -p /var/run/rplidar
            chown $user:$user /var/run/rplidar
            "$daemon" --respawn $daemon_start_args \
                --name "$name" --pidfiles "$pidfiles" \
                ${user:+--user $user} ${chroot:+--chroot $chroot} \
                ${chdir:+--chdir $chdir} ${umask:+--umask $umask} \
                ${stdout:+--stdout $stdout} ${stderr:+--stderr $stderr} \
                -- \
                "$command" $command_args
            echo done.
        fi
        ;;

    stop)
        # This if statement isn't strictly necessary but it's user friendly
        if "$daemon" --running --name "$name" --pidfiles "$pidfiles"
        then
            echo -n "Stopping $name..."
            "$daemon" --stop --name "$name" --pidfiles "$pidfiles"
            echo done.
        else
            echo "$name is not running."
        fi
        ;;

    restart|reload)
        if "$daemon" --running --name "$name" --pidfiles "$pidfiles"
        then
            echo -n "Restarting $name..."
            "$daemon" --restart --name "$name" --pidfiles "$pidfiles"
            echo done.
        else
            echo "$name is not running."
            exit 1
        fi
        ;;

    status)
        "$daemon" --running --name "$name" --pidfiles "$pidfiles" --verbose
        ;;

    *)
        echo "usage: $0 <start|stop|restart|reload|status>" >&2
        exit 1
esac

exit 0
