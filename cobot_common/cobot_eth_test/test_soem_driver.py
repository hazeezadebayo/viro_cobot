#!/usr/bin/env python3
import socket
import struct
import time
from scapy.all import Ether, sendp, srp1
from scapy.contrib.ethercat import EtherCat, EtherCatAPRD

# Load EtherCAT
from scapy.all import load_contrib
load_contrib("ethercat")

INTERFACE = "enp3_sim"

def test_discovery():
    print(f"Testing EtherCAT discovery on {INTERFACE}...")
    
    # 1. Create an Auto-Increment Read PDU (APRD)
    # command=1 (APRD), adp=0, ado=0
    pdu = EtherCatAPRD(adp=0, ado=0, len=2)
    
    # 2. Build the frame
    pkt = Ether(dst="ff:ff:ff:ff:ff:ff", type=0x88a4) / EtherCat(type=1) / pdu
    
    # 3. Send and wait for response
    print("Sending discovery packet (APRD)...")
    resp = srp1(pkt, iface=INTERFACE, timeout=2, verbose=False)
    
    if resp and resp.haslayer(EtherCat):
        print("SUCCESS: Received EtherCAT response!")
        # In our mock, the emulator increments ADP by NUM_JOINTS
        ecat = resp[EtherCat]
        pdu_resp = ecat.payload
        if hasattr(pdu_resp, 'adp'):
            num_slaves = pdu_resp.adp
            print(f"Mock Emulator reported {num_slaves} slaves.")
            if num_slaves == 6:
                print("Test PASSED: The emulator is correctly simulating 6 joints.")
            else:
                print(f"Test FAILED: Expected 6 slaves, got {num_slaves}.")
    else:
        print("FAILED: No response from emulator. Is it running?")

if __name__ == "__main__":
    test_discovery()
