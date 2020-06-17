#!/bin/bash
cd ~
pwd = `pwd`
cat ~/worngDirFiles.list | while read line || [[ -n $line ]];
do
  sed -i 's#/home/fares/src/orbslam2/#$pwd/ORB_SLAM2/#g' $line
done
