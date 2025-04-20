#!/bin/bash

mkdir logs

## do a quick update
sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get dist-upgrade -y;

## install python essentials
sudo apt-get install -y python3-numpy python3-scipy python3-matplotlib tmux libatlas-base-dev libgpiod2 autossh rename;
sudo apt-get install -y libv4l-dev libturbojpeg-dev ntpdate;
sudo -H python3 -m pip install cython astropy cherrypy pyephem adafruit-circuitpython-dht;

## install rename
sudo apt-get install rename;

## install ntp
sudo apt-get install -y ntp;

make
