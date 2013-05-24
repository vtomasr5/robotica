#!/bin/bash

if [ "$1" == "map" ] 
then
	MobileSim -m /usr/local/MobileSim/AMROffice.map &
else
	MobileSim --nomap &
fi
#MobileSim -m prova.map &
sleep 1
make clean
make -j8 && ./practica
