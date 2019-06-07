#!/bin/bash

sudo apt-get update

sudo apt-get install python-pip python3-pip
cat pip-installs.txt | xargs sudo pip install --upgrade
cat pip-installs.txt | xargs sudo pip3 install --upgrade

curl https://raw.githubusercontent.com/dheera/jetson-install-this/master/ros.shit | bash
curl https://raw.githubusercontent.com/dheera/jetson-install-this/master/pytorch.shit | bash
curl https://raw.githubusercontent.com/dheera/jetson-install-this/master/tensorflow.shit | bash

