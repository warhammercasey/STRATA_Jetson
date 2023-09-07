#!/bin/bash
cd /home/strata/git/startup/STRATA_Jetson
lanIp=$(ip -f inet addr show wlan0)
echo ${lanIp} >> ip.txt
git add ip.txt
git commit -m "Updated ip"
git push -u origin ip
