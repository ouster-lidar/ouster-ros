#!/bin/bash

# NOTE: These settings are not persisted by default during system restarts
# You will have to run these on every startup or consult the OS docs on how
# to persist these settings on your respective OS

# tune kernel receive buffer size
sudo sysctl -w net.core.rmem_max=2147483647         # 2 GiB
sudo sysctl -w net.core.rmem_default=2147483647     # 2 GiB

# tune IP fragmentation settings
sudo sysctl -w net.ipv4.ipfrag_time=3                 # 3s
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB

# optional: might help on certain platforms
## in addition you can try increasing the receive buffer size
# sudo ethtool -G <sensor_network_interface> rx 2048

# optional: might help on certain platforms
## increase the MTU size to 9000 to decrease fragmentation
# sudo ifconfig <sensor_network_interface> mtu 9000
