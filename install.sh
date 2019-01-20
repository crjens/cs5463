#!/bin/bash

# Raspberry Pi 
git clone git://git.drogon.net/wiringPi 
cd ./wiringPi 
./build 
cd ../ 
node-gyp rebuild 

# enable SPI
echo '>>> Enable SPI'
sudo raspi-config nonint do_spi 0
#sudo sed -i 's/^#dtparam=spi=on.*/dtparam=spi=on/' /boot/config.txt