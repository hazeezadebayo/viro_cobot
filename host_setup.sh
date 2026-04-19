#!/bin/bash

# Viro Cobot - Host Setup Script
# Installs the IgH EtherCAT Master kernel module on the host machine.
# This is required for physical robot interaction via EtherCAT.
#
# IMPORTANT: Run this script with root privileges (e.g., sudo ./host_setup.sh)

set -e

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (use sudo ./host_setup.sh)"
  exit 1
fi

echo "Starting IgH EtherCAT Master installation on Host..."

# Install dependencies
apt-get update
apt-get install -y build-essential git autoconf libtool pkg-config dkms

# Clone and Build
TEMP_DIR=$(mktemp -d)
cd "$TEMP_DIR"
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
./bootstrap
./configure --disable-8139too --disable-e1000 --disable-e1000e --disable-r8169 --enable-generic --prefix=/usr/local/etherlab
make all modules
make modules_install install
depmod -a

# Configuration
mkdir -p /etc/sysconfig
cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
ln -sf /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
systemctl daemon-reload

echo "--------------------------------------------------------"
echo "Installation complete!"
echo "Next steps:"
echo "1. Edit /etc/sysconfig/ethercat to set your MASTER0_DEVICE (NIC MAC address)."
echo "2. Set DEVICE_MODULES=\"generic\"."
echo "3. Run 'systemctl start ethercat' to start the driver."
echo "4. Run './run_viro.sh launch' to start the robot."
echo "--------------------------------------------------------"

# Cleanup
rm -rf "$TEMP_DIR"
