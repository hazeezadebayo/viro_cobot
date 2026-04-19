#!/bin/bash

# Configuration
IMAGE_NAME="cobot-eth-emulator"
DEFAULT_INTERFACE="enp3_sim_v"
INTERFACE=${1:-$DEFAULT_INTERFACE}

# Ensure we are in the script directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

# 1. ALWAYS build the image (Docker cache handles speed)
# This ensures that any changes to robot_slave_emulator.py are picked up.
echo "--------------------------------------------------------"
echo "Ensuring Docker image [$IMAGE_NAME] is up to date..."
docker build -t $IMAGE_NAME "$DIR"
echo "--------------------------------------------------------"

# 2. Setup network if using default virtual interface and it doesn't exist
if [ "$INTERFACE" == "$DEFAULT_INTERFACE" ]; then
    if ! ip link show $INTERFACE > /dev/null 2>&1; then
        echo "Interface $INTERFACE not found. Running setup_network.sh..."
        ./setup_network.sh
    else
        echo "Interface $INTERFACE already exists."
    fi
fi

echo "Running EtherCAT Emulator on $INTERFACE..."

# 3. Run container
# --privileged: needed for scapy to use raw sockets
# --net=host: needed to see host interfaces (veth or physical)
docker run -i --rm \
    --privileged \
    --net=host \
    -e ETHERCAT_INTERFACE=$INTERFACE \
    --name cobot_emulator_instance \
    $IMAGE_NAME
