# ZOESoundGenerator

This is a simple sound generator for the Renault ZOE and an Arduino Due which produces these noises: https://youtu.be/zKB3S2EzTBk
It loads three samples directly onto the Due, using 80% of RAM. Future versions will use a SD card.
You need a CAN transceiver like this one: https://www.amazon.de/gp/product/B00KM6XMXO/
Connect it as shown in the schematic.
You need an active speaker to connect the DAC to. It only delivers a small signal in the range from 0.55 V to 2.75 V.
