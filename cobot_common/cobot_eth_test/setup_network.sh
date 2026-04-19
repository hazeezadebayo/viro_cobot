#!/bin/bash

# This script creates a virtual ethernet pair to simulate an EtherCAT link.
# enp3_sim <---> enp3_sim_v

# 1. Cleanup old interfaces if they exist
sudo ip link delete enp3_sim 2>/dev/null
sudo ip link delete enp3_sim_v 2>/dev/null

# 2. Create the veth pair
# We use sudo because network interface creation is a privileged operation.
sudo ip link add enp3_sim type veth peer name enp3_sim_v

# 3. Bring the interfaces UP
sudo ip link set enp3_sim up
sudo ip link set enp3_sim_v up

# 4. Set to promiscuous mode for raw packet capture
sudo ip link set enp3_sim promisc on
sudo ip link set enp3_sim_v promisc on

# 5. Disable IPv6 to keep the link clean from router advertisements
sudo sysctl -w net.ipv6.conf.enp3_sim.disable_ipv6=1 2>/dev/null
sudo sysctl -w net.ipv6.conf.enp3_sim_v.disable_ipv6=1 2>/dev/null

echo "Virtual EtherCAT network created: enp3_sim <--- wire ---> enp3_sim_v"
echo "Your soem_config.yaml should use 'enp3_sim'."
echo "The emulator is running on 'enp3_sim_v'."
