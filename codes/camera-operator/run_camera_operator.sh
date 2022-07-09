#!/bin/bash

mkdir -p "$HOME/tmp"
PIDFILE="$HOME/tmp/myprogram.pid"

if [ -e "${PIDFILE}" ] && (ps -u $(whoami) -opid= |
                           grep -P "^\s*$(cat ${PIDFILE})$" &> /dev/null); then
  echo "Already running."
  exit 99
fi

python3 /home/pi/camera-operator/camera_operator.py 921600 1 >> /home/pi/log.txt 2>&1 &

echo $! > "${PIDFILE}"
chmod 644 "${PIDFILE}"
