#!/bin/bash

sudo apt-get update

echo "apt-getting stuff ..."
cat system/apt-installs.txt | xargs sudo apt-get install -y

echo "pipping stuff ..."
cat system/pip-installs.txt | xargs sudo pip install --upgrade
cat system/pip-installs.txt | xargs sudo pip3 install --upgrade

echo "installing other stuff ..."
echo "[ros]"
system/magic-jetson-install ros
echo "[pytorch]"
system/magic-jetson-install pytorch
echo "[tensorflow]"
system/magic-jetson-install tensorflow

