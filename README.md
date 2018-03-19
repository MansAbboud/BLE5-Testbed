# BLE5-Testbed

This repository contains the source files of the BLE testbed which has been developed for the evaluation of Bluetooth 5.0 Physical Layers (BLE PHYs).
The implementation has been carried out during a research project at Communication and Networked Systems (ComSys) department at Otto von Guericke University Magdeburg  http://www.comsys.ovgu.de/.

# Objectives
The main Objective of this testbed is to provide researchers a free access to the BLE radio chip in the 2.4GHz frequency band which is usually not possible when the softwarestack is in use. This enables testing/evaluating the new PHYs of BLE 5 standard such as LE 2M and LE Codded without the need for establishing a connection. The packet structure of all defined PHYs has been defined with a variable payload size up to 255 Bytes. All BLE channels (advertismnet and data) can be accessed and tested with different: packet sizes, transmit power settings and radio modes. Additionally, Frequency Hopping algorithms can be implemented by developers, scanning on a specific advertisment channel is possible, synchronization of nodes can be more accurate after disabling the softwarestack.   



# Structure
The testbed is composed of three BLE units: Broadcaster which transmit packet, Observer which receives packets, and a Sniffer for measuring the background noise of the test environment (Noise Floor). The units are based on the Nordic nRF52840 development kit https://www.nordicsemi.com/eng/Products/nRF52840 which supports multiple radio standards such as BLE, IEEE 802.15.4, ANT and Nordic 2.4GHz proprietary protocols.

![GitHub Logo](/nrf52840.JPG)

The functiionality of the testbed is described in the following figure.

![GitHub Logo](/structure.png)



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
