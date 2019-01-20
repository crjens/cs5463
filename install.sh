#!/bin/bash

# require root
if [ $(id -u) -ne 0 ]; then
  printf "Script must be run as root. Try 'sudo $0'\n"
  exit 1
fi

# Install wiring Pi
echo '>>> Install Wiring Pi'
git clone git://git.drogon.net/wiringPi 
cd ./wiringPi 
./build 
cd ../ 
node-gyp rebuild  

# enable SPI
echo '>>> Enable SPI'
sudo raspi-config nonint do_spi 0
