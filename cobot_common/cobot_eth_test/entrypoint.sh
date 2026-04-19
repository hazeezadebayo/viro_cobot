#!/bin/bash
set -e

echo "Starting Viro Cobot EtherCAT Emulator..."
echo "Binding to interface: ${ETHERCAT_INTERFACE}"

# Run the emulator
python3 robot_slave_emulator.py
