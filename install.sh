#!/bin/bash

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
