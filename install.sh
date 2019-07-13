#!/bin/bash

# build
node-gyp rebuild  

# enable SPI
echo '>>> Enable SPI'
sudo raspi-config nonint do_spi 0
