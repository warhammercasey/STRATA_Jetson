#!/bin/bash
cd /home/strata/git/startup/STRATA_Jetson
sleep 10
lanIp=$(ip -f inet addr show wlan0 | grep -oP 'inet \K[\d.]+')
echo ${lanIp} > ip.txt
git add *
git commit -m "Updated ip"
git push -u origin ip
