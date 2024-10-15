#!/bin/bash

# kernel receive buffer size
sudo sysctl -w net.core.rmem_max=16777216
sudo sysctl -w net.core.rmem_default=16777216

## in addition you can try increasing the receive buffer size
# sudo ethtool -G <sensor_network_interface> rx 2048

## increase the MTU size to 9000 to decrease fragmentation
# sudo ifconfig <sensor_network_interface> mtu 9000
