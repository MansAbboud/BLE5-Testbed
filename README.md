# BLE5-Testbed

This repository contains the source code files of the BLE testbed which has been developed for the evaluation of Bluetooth 5.0 Physical Layers (BLE PHYs).
The implementation has been carried out during a research project at the Communication and Networked Systems (ComSys) department at Otto von Guericke University Magdeburg  http://www.comsys.ovgu.de/.

# Objectives
The main objective is to provide researchers a free access to the BLE radio chip in the 2.4GHz frequency band which is usually not possible when the BLE stack is in use. This enables testing/evaluating the new PHYs of BLE 5 standard such as LE 2M and LE Coded without the need for establishing a connection. For example, LE Coded PHY which uses Forward Error Correction (FEC) can only be tested during a connection which starts first with connection establishment on LE 1M then switching to LE Coded.
Furthermore, the packet structure of all PHYs has been defined with a variable payload size up to 255 Bytes. All BLE channels (advertismnet and data) can be accessed and tested with different: packet sizes, transmit power settings and radio modes. Additionally, Frequency Hopping algorithms can be implemented by developers, scanning on a specific advertisment channel is possible, synchronization of nodes can be more accurate after disabling the stack.   


# Structure
The testbed is composed of three BLE units: Broadcaster which transmit packet, Observer which receives packets, and a Sniffer for measuring the background noise of the test environment (Noise Floor). The units are based on the Nordic nRF52840 development kit https://www.nordicsemi.com/eng/Products/nRF52840 which supports multiple radio standards such as BLE, IEEE 802.15.4, ANT and Nordic 2.4GHz proprietary protocols (See picture below).

![GitHub Logo](/nrf52840.jpg)

The testbed is based on a connectionless one way transmission of a certain number of packets from the broadcaster to the observer without replying any ACK. On the observer side, received packets including their Received Signal Strength (RSS) in addition to the Packet Error Rate (PER) of each transmission are recorded. 

Directory | Content
------------ | -------------
BLE5-Testbed/code/radio_config.h | contains the parameters defines of the implementation
BLE5-Testbed/code/nRF52840_Observer_main.c | dedicated for the Observer  
BLE5-Testbed/code/nRF52840_Broadcaster_main.c | dedicated for the Broadcaster
BLE5-Testbed/code/nRF52840_Sniffer_main.c | dedicated for the Sniffer


Please use Issues to report problems of this implementation.

# License
GNU General Public License v3.0
