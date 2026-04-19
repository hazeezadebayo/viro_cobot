FROM python:3.10-slim

# Install system dependencies
# iproute2: for interface management if needed
# tcpdump: for debugging packets
RUN apt-get update && apt-get install -y --no-install-recommends \
    iproute2 \
    tcpdump \
    && rm -rf /var/lib/apt/lists/*

# Install Scapy
RUN pip install --no-cache-dir scapy

WORKDIR /app

# Copy emulator scripts
COPY robot_slave_emulator.py .
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh

# Use environment variable ETHERCAT_INTERFACE to specify which interface to bind to
# Default is enp3_sim_v for internal veth setup
ENV ETHERCAT_INTERFACE=enp3_sim_v
ENV PYTHONUNBUFFERED=1

ENTRYPOINT ["./entrypoint.sh"]

