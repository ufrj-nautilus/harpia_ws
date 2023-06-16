#!/bin/bash
echo "Choose where you want to store the harpia workspace, leave it blank for $HOME/catkin_ws/src"
echo -n "mkdir -p $HOME/"
read workspace

mkdir -p $HOME/$workspace/catkin_ws/src
cd $HOME/$workspace/catkin_ws/src
git clone git@github.com:ufrj-nautilus/harpia_simulator.git
git clone git@github.com:ufrj-nautilus/harpia_CBR.git
git clone git@github.com:ufrj-nautilus/harpia_mensagens.git
git clone git@github.com:ufrj-nautilus/harpia_localization.git
git clone git@github.com:ufrj-nautilus/harpia_utils.git
git clone git@github.com:ufrj-nautilus/harpia_control.git
