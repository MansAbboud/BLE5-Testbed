# BLE5-Testbed

This repository contains the source files of the testbed which has been developed for Bluetooth 5.0 Physical Layer Evaluation.
The testbed is composed of three BLE units: Broadcaster which transmit packet, Observer which receives packets, and a Sniffer for measuring the background noise of the test environment (Noise Floor). The units are based on the Nordic nRF52840 development kit https://www.nordicsemi.com/eng/Products/nRF52840 which supports multiple radio standards such as BLE, IEEE 802.15.4, ANT and Nordic 2.4GHz proprietary protocols. 

![drawing](/nrf52840.JPG =200x100)


Additionally,  

The implementation has been carried out during a research project at the Communication and Networked System (ComSys) department at Otto von Guericke University Magdeburg  http://www.comsys.ovgu.de/.

The BLE5-Testbed/code includes a radio_config.h which contains the parameters defines of the packet structure and other required defines.

Please use Issues to report problems for the BLE5-Testbed implementation.

Directory | Content
------------ | -------------
BLE5-Testbed/code/radio_config.h | contains the parameters defines of the implementation
BLE5-Testbed/code/nRF52840_Observer_main.c | dedicated for the Observer  
BLE5-Testbed/code/nRF52840_Broadcaster_main.c | dedicated for the Broadcaster
BLE5-Testbed/code/nRF52840_Sniffer_main.c | dedicated for the Sniffer

# License
BLE5-Testbed is open source and can be freely used by anyone.
