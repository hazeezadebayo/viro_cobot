#!/bin/bash

echo "Stopping EtherCAT Emulator..."

# 1. Stop the docker container
docker stop cobot_emulator_instance 2>/dev/null || true

# 2. Cleanup virtual network interfaces if they exist
if ip link show enp3_sim > /dev/null 2>&1; then
    echo "Cleaning up virtual interfaces (enp3_sim, enp3_sim_v)..."
    sudo ip link delete enp3_sim 2>/dev/null || true
fi

echo "Cleanup complete."
