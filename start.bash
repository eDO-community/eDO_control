#!/bin/bash

master_ip="10.42.0.49"
hostname=`hostname`

vars="export ROS_MASTER_URI=\"http://${master_ip}:11311\";"
vars=$vars" export ROS_HOSTNAME=\"${hostname}.local\";"
vars=$vars" export PS1=\"\[\033[00;33m\][\${ROS_MASTER_URI}]\[\033[00m\] \${PS1}\""

bash --rcfile <(cat ~/.bashrc; echo "${vars}")
