#!/bin/bash
echo "Choose where you want to store the harpia workspace, leave it blank for $HOME/harpia/src"
echo -n "mkdir -p $HOME/"
read workspace

mkdir -p $HOME/$workspace/harpia/src
cd $HOME/$workspace/src
git clone git@github.com:ufrj-nautilus/harpia_ws.git
git clone git@github.com:ufrj-nautilus/harpia_control.git
git clone git@github.com:ufrj-nautilus/harpia_simulator.git
