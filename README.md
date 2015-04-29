DriverControlBoxTRI63
=====================

This is the code downloaded from the Tritium website (http://tritium.com.au/products/legacy-products/) on September 17 2014.

We build this on a ubuntu system. The package manager work fine
So far we are using only this

 gcc-msp430                         - GNU C compiler (cross compiler for MSP430) 

sudo apt-get install gcc-msp430

Also grab the memory.x and periph.x files for the processor you have
cp /usr/msp430/lib/ldscripts/msp430f135/periph.x  ~/projects/drivercontrol/build/
