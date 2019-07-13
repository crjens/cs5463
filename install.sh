#!/bin/bash

# install wiring pi
cd /tmp
wget https://project-downloads.drogon.net/wiringpi-latest.deb
sudo dpkg -i wiringpi-latest.deb
cd ~

# build
node-gyp rebuild  

# enable SPI
echo '>>> Enable SPI'
sudo raspi-config nonint do_spi 0
