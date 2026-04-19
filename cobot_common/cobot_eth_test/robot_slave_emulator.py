#!/usr/bin/env python3
"""
Viro Cobot EtherCAT Emulator — Pure Raw Socket Implementation.

Rationale: Scapy's sniff+sendp pipeline has multi-millisecond latency which
causes SOEM's 2ms per-retry receive window to expire before our response
arrives. This implementation uses Python's built-in AF_PACKET raw sockets
with `select()` for a minimal-latency event loop. No Scapy in the hot path.
"""
import os, sys, time, struct, threading, select, socket

INTERFACE  = os.environ.get("ETHERCAT_INTERFACE", "enp3_sim_v")
NUM_JOINTS = 6
ETH_P_ECAT = 0x88A4  # EtherCAT EtherType

# ── Stats ──────────────────────────────────────────────────────────────────
packet_count    = 0
last_packet_time= 0.0
master_detected = False

# ── CiA 402 Status Word bits ───────────────────────────────────────────────
READY_TO_SWITCH_ON = 0x0001
SWITCHED_ON        = 0x0002
OPERATION_ENABLED  = 0x0004
FAULT              = 0x0008
VOLTAGE_ENABLED    = 0x0010
QUICK_STOP         = 0x0020
SWITCH_ON_DISABLED = 0x0040

class JointState:
    def __init__(self, jid):
        self.id              = jid
        self.al_state        = 0x01  # INIT
        self.configured_addr = 0
        self.status_word     = SWITCH_ON_DISABLED | VOLTAGE_ENABLED | QUICK_STOP
        self.control_word    = 0x0000
        self.target_position = 0
        self.actual_position = 0
        self.actual_velocity = 0
        self.actual_torque   = 0
        self.error_code      = 0
        self.mode_display    = 8   # CSP
        self.digital_inputs  = 0
        self.digital_outputs = 0
        
        # ── EEPROM Simulation ──
        self.eeprom_mem = bytearray(1024)
        # Word 0x0008 (byte 0x10): Vendor ID
        struct.pack_into("<I", self.eeprom_mem, 0x10, 0x0000A13A) # ZeroErr
        # Word 0x000A (byte 0x14): Product ID
        struct.pack_into("<I", self.eeprom_mem, 0x14, 0x00000018) # eRob series
        # Word 0x0018 (byte 0x30): Standard Rx Mbx Offset
        struct.pack_into("<H", self.eeprom_mem, 0x30, 0x1000) # Rx offset
        struct.pack_into("<H", self.eeprom_mem, 0x32, 0x0080) # Rx size
        struct.pack_into("<H", self.eeprom_mem, 0x34, 0x1080) # Tx offset
        struct.pack_into("<H", self.eeprom_mem, 0x36, 0x0080) # Tx size
        struct.pack_into("<H", self.eeprom_mem, 0x38, 0x0001) # mbx_proto: CoE
        
        self.eep_word_addr = 0
        self.eep_control = 0

joints = [JointState(i) for i in range(NUM_JOINTS)]

# ── Open raw socket ────────────────────────────────────────────────────────
def open_raw_socket(iface):
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ECAT))
    s.bind((iface, 0))
    s.setblocking(False)
    return s

sock = open_raw_socket(INTERFACE)

# Get our MAC so we can filter our own echoes
def get_mac(iface):
    import fcntl
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(s.fileno(), 0x8927, struct.pack('256s', iface[:15].encode()))
    return info[18:24]

MY_MAC_BYTES = get_mac(INTERFACE)
MY_MAC_STR   = ':'.join('%02x' % b for b in MY_MAC_BYTES)
print(f"Starting EtherCAT Emulator on {INTERFACE} (MAC={MY_MAC_STR})", flush=True)
print("Waiting for EtherCAT Master traffic...", flush=True)

# ── PCAP Exporter ──
pcap_fd = open('/app/trace.pcap', 'wb')
pcap_fd.write(b'\xd4\xc3\xb2\xa1\x02\x00\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\xff\x00\x00\x01\x00\x00\x00')
pcap_pkts = 0

def pcap_write(pkt):
    global pcap_pkts
    if pcap_pkts > 2000: return
    pcap_pkts += 1
    ts = time.time()
    ts_sec = int(ts)
    ts_usec = int((ts - ts_sec) * 1000000)
    pcap_fd.write(struct.pack('<IIII', ts_sec, ts_usec, len(pkt), len(pkt)))
    pcap_fd.write(pkt)
    if pcap_pkts % 100 == 0:
        pcap_fd.flush()

# ── Heartbeat ──────────────────────────────────────────────────────────────
def heartbeat():
    while True:
        time.sleep(5)
        now = time.time()
        if not master_detected:
            print(f"[{time.strftime('%H:%M:%S')}] [HEARTBEAT] ❤️ Alive, waiting for Master on {INTERFACE}...", flush=True)
        else:
            elapsed = now - last_packet_time
            if elapsed > 2:
                print(f"[{time.strftime('%H:%M:%S')}] [WARNING] ⚠️ Master silent for {elapsed:.1f}s (pkts={packet_count})", flush=True)
            else:
                print(f"[{time.strftime('%H:%M:%S')}] [OK] ✅ Processing traffic (pkts={packet_count})", flush=True)

threading.Thread(target=heartbeat, daemon=True).start()

# ── Packet handler ─────────────────────────────────────────────────────────
def handle_frame(raw_frame):
    global packet_count, last_packet_time, master_detected

    if len(raw_frame) < 14 + 2:
        return

    # ── Ethernet header ──
    dst_mac = raw_frame[0:6]
    src_mac = raw_frame[6:12]
    ethertype = struct.unpack_from(">H", raw_frame, 12)[0]

    if ethertype != ETH_P_ECAT:
        return
    # Filter our own echoes by MAC
    if src_mac == MY_MAC_BYTES:
        return

    pcap_write(raw_frame)

    packet_count    += 1
    last_packet_time = time.time()
    if not master_detected:
        print(f"[{time.strftime('%H:%M:%S')}] [SUCCESS] ✅ EtherCAT Master DETECTED!", flush=True)
        master_detected = True

    # ── EtherCAT frame header (2 bytes after Ethernet header) ──
    raw_ecat = raw_frame[14:]
    if len(raw_ecat) < 2:
        return
    ecat_hdr_val = struct.unpack_from("<H", raw_ecat, 0)[0]
    ecat_len     = ecat_hdr_val & 0x0FFF   # 12-bit length (SOEM uses 0x0fff mask)
    ecat_type    = (ecat_hdr_val >> 12) & 0x0F

    if ecat_type != 1:   # Not a PDU datagram frame
        return

    payload_bytes = raw_ecat[2 : 2 + ecat_len]
    offset        = 0
    responses_raw = b""

    while offset + 12 <= len(payload_bytes):
        # ── PDU header: Cmd(1) Idx(1) ADP(2) ADO(2) Len+flags(2) IRQ(2) ──
        try:
            cmd, idx, adp, ado, v_len, irq = struct.unpack_from("<BBHHHH", payload_bytes, offset)
        except struct.error:
            break

        pdu_len  = v_len & 0x07FF
        has_next = (v_len & 0x8000) != 0   # bit-15 = More PDUs follow

        data_start = offset + 10
        data_end   = data_start + pdu_len
        if data_end + 2 > len(payload_bytes):
            break

        pdu_data = bytearray(payload_bytes[data_start : data_end])
        wkc      = struct.unpack_from("<H", payload_bytes, data_end)[0]

        # ── Command dispatch ──
        p_cmd = {1:"APRD",2:"APWR",3:"APRW",4:"FPRD",5:"FPRD",6:"FPWR",
                 7:"BRD", 8:"BWR", 9:"BRW", 11:"LWR", 12:"LRW"}.get(cmd, f"CMD{cmd}")

        # ── Determine addressed slaves ──
        addressed_slaves = []
        if cmd in (7, 8, 9):  # Broadcast
            addressed_slaves = joints
        elif cmd in (1, 2, 3):  # Auto-Increment
            adp_s = adp if adp < 0x8000 else adp - 0x10000
            if -(NUM_JOINTS - 1) <= adp_s <= 0:
                addressed_slaves = [joints[-adp_s]]
        elif cmd in (4, 5, 6):  # Fixed Physical
            addressed_slaves = [j for j in joints if j.configured_addr == adp]

        # ── Handle EEPROM / Registers ──
        if addressed_slaves:
            # Configured Station Address (0x0010)
            if ado == 0x0010:
                if cmd in (2, 3, 6, 8, 9) and len(pdu_data) >= 2: # Write
                    new_addr = struct.unpack_from("<H", pdu_data, 0)[0]
                    for j in addressed_slaves:
                        j.configured_addr = new_addr
                        print(f"[{time.strftime('%H:%M:%S')}] [CONFIG] Slave {j.id} address set to 0x{new_addr:04x}", flush=True)
                elif cmd in (1, 3, 4, 5, 7, 9) and len(pdu_data) >= 2: # Read
                    struct.pack_into("<H", pdu_data, 0, addressed_slaves[0].configured_addr)

            # Alias Address (0x0012)
            if ado == 0x0012 and cmd in (1, 3, 4, 5, 7, 9) and len(pdu_data) >= 2:
                struct.pack_into("<H", pdu_data, 0, 0) # Alias is zero

            # EEPROM Emulation (0x0500 - 0x050B)
            if 0x0500 <= ado <= 0x050B:
                for j in addressed_slaves:
                    # Write to Control (0x0500)
                    if ado == 0x0500 and cmd in (2, 3, 6, 8, 9) and len(pdu_data) >= 4:
                        ctrl, eep_addr = struct.unpack_from("<HH", pdu_data, 0)
                        if ctrl & 0x0100:  # Read Command
                            j.eep_word_addr = eep_addr
                            j.eep_control = 0  # Clear busy bit immediately (0x0000 implies success)
                            
                    # Read from Control/Data
                    if cmd in (1, 3, 4, 5, 7, 9):
                        # SOEM does an 8-byte FPRD from 0x0500 to read Control(2) + Addr(2) + Data(4)
                        if ado == 0x0500 and len(pdu_data) >= 2:
                            struct.pack_into("<H", pdu_data, 0, j.eep_control)
                            if len(pdu_data) >= 8: # The data chunk is at offset 4
                                byte_addr = j.eep_word_addr * 2
                                val = struct.unpack_from("<I", j.eeprom_mem, byte_addr)[0] if byte_addr + 4 <= len(j.eeprom_mem) else 0
                                struct.pack_into("<I", pdu_data, 4, val)
                        # Explicit read of just the Data register
                        elif ado in (0x0504, 0x0508) and len(pdu_data) >= 4:
                            byte_addr = j.eep_word_addr * 2
                            val = struct.unpack_from("<I", j.eeprom_mem, byte_addr)[0] if byte_addr + 4 <= len(j.eeprom_mem) else 0
                            struct.pack_into("<I", pdu_data, 0, val)

            # AL Control (0x0120) WRITE
            if ado == 0x0120 and cmd in (2, 3, 6, 8, 9) and len(pdu_data) >= 2:
                requested = pdu_data[0] & 0x0F
                has_ack = (pdu_data[0] & 0x10) != 0
                for j in addressed_slaves:
                    if requested in (0x01, 0x02, 0x04, 0x08):
                        if requested != j.al_state:
                            j.al_state = requested
                            print(f"[{time.strftime('%H:%M:%S')}] [STATE] Slave {j.id} (addr=0x{j.configured_addr:04x}) AL transition → 0x{j.al_state:02x} "
                                  f"({'INIT' if j.al_state==1 else 'PRE-OP' if j.al_state==2 else 'SAFE-OP' if j.al_state==4 else 'OP'})", flush=True)
                        if has_ack:
                            print(f"[{time.strftime('%H:%M:%S')}] [STATE] Slave {j.id} Error ACK received.", flush=True)

            # AL Status (0x0130) READ
            if ado == 0x0130 and cmd in (1, 3, 4, 5, 7, 9) and len(pdu_data) >= 2:
                # Return AL state of the first addressed slave
                pdu_data[0:2] = bytes([addressed_slaves[0].al_state, 0x00])

        print(f"[{time.strftime('%H:%M:%S')}] [RX] {p_cmd}(cmd={cmd}) ADP=0x{adp:04x} ADO=0x{ado:04x} "
              f"len={pdu_len} wkc_in={wkc}", flush=True)


        # BRD / BWR — Broadcast: all NUM_JOINTS virtual slaves respond
        if cmd in (7, 8):
            wkc += NUM_JOINTS
            if ado == 0x0000 and len(pdu_data) >= 2:    # ESC Type register
                pdu_data[0:2] = b'\x11\x00'
            print(f"[{time.strftime('%H:%M:%S')}] [TX] {p_cmd} Broadcast ADO=0x{ado:04x} → WKC={wkc}", flush=True)

        # APRD / APWR — Auto-Increment: slave N addressed by ADP=-(N-1) mod 65536
        elif cmd in (1, 2, 3):
            adp_s = adp if adp < 0x8000 else adp - 0x10000
            if -(NUM_JOINTS - 1) <= adp_s <= 0:
                wkc += 1
                if ado == 0x0000 and len(pdu_data) >= 2: pdu_data[0:2] = b'\x11\x00'
                print(f"[{time.strftime('%H:%M:%S')}] [TX] {p_cmd} Auto-Inc ADP_s={adp_s} "
                      f"ADO=0x{ado:04x} → WKC={wkc}", flush=True)

        # FPRD / FPWR — Fixed Physical address (SOEM assigns these after auto-inc scan)
        elif cmd in (4, 5, 6):
            if addressed_slaves:
                wkc += 1
                print(f"[{time.strftime('%H:%M:%S')}] [TX] {p_cmd} Fixed ADP=0x{adp:04x} "
                      f"ADO=0x{ado:04x} → WKC={wkc}", flush=True)

        # LWR / LRW — Logical (PDO process data exchange)
        elif cmd in (11, 12):
            for i in range(NUM_JOINTS):
                rx_off = i * 14
                if len(pdu_data) >= rx_off + 14:
                    t_pos, d_out, t_trq, t_off, c_word = struct.unpack_from("<iiHhH", pdu_data, rx_off)
                    joints[i].control_word    = c_word
                    joints[i].target_position = t_pos
                    new_status = joints[i].status_word
                    if c_word & 0x80:               new_status &= ~FAULT
                    if   (c_word & 0x0F) == 0x06:   new_status = (new_status & ~0x7F) | 0x21
                    elif (c_word & 0x0F) == 0x07:   new_status = (new_status & ~0x7F) | 0x23
                    elif (c_word & 0x0F) == 0x0F:   new_status = (new_status & ~0x7F) | 0x27
                    joints[i].status_word  = new_status
                    joints[i].actual_position += (joints[i].target_position - joints[i].actual_position) // 4

            tx_base = NUM_JOINTS * 14
            for i in range(NUM_JOINTS):
                tx_off = tx_base + i * 20
                if len(pdu_data) >= tx_off + 20:
                    struct.pack_into("<iiHhiHBB", pdu_data, tx_off,
                                    joints[i].actual_position, joints[i].digital_inputs,
                                    joints[i].status_word,     joints[i].actual_torque,
                                    joints[i].actual_velocity, joints[i].error_code,
                                    joints[i].mode_display,    0)
            wkc += NUM_JOINTS * 3
            if packet_count % 100 == 0:
                print(f"[{time.strftime('%H:%M:%S')}] [TX] {p_cmd} PDO → WKC={wkc}", flush=True)

        # ── Rebuild PDU: echo header unchanged, updated data + WKC ──
        responses_raw += struct.pack("<BBHHHH", cmd, idx, adp, ado, v_len, irq)
        responses_raw += bytes(pdu_data)
        responses_raw += struct.pack("<H", wkc)

        offset = data_end + 2
        if not has_next:
            break

    if not responses_raw:
        return

    # ── Build Ethernet frame ──
    # EtherCAT header: same type bits, updated length
    resp_ecat_hdr  = (len(responses_raw) & 0x0FFF) | (1 << 12)
    ecat_payload   = struct.pack("<H", resp_ecat_hdr) + responses_raw

    # Ethernet header: respond to sender's MAC directly
    eth_hdr = dst_mac + MY_MAC_BYTES + struct.pack(">H", ETH_P_ECAT)   # Note: dst=src_mac, but dst_mac of incoming = ff:ff:ff:ff:ff:ff for broadcasts; use src_mac of incoming frame as our destination
    # Actually: we send back TO the master (src of incoming frame)
    eth_hdr = src_mac + MY_MAC_BYTES + struct.pack(">H", ETH_P_ECAT)
    response_frame = eth_hdr + ecat_payload

    try:
        sock.send(response_frame)
        pcap_write(response_frame)
        print(f"[{time.strftime('%H:%M:%S')}] [SENT] {len(response_frame)}B frame → {':'.join('%02x'%b for b in src_mac)}", flush=True)
    except Exception as e:
        print(f"[{time.strftime('%H:%M:%S')}] [ERR] send() failed: {e}", flush=True)

# ── Main event loop using select() ────────────────────────────────────────
print(f"Entering main select() loop on {INTERFACE}...", flush=True)
while True:
    try:
        readable, _, _ = select.select([sock], [], [], 1.0)
        if readable:
            raw_frame = sock.recv(2048)
            handle_frame(raw_frame)
    except KeyboardInterrupt:
        print("\nShutting down emulator.")
        try:
            pcap_fd.close()
            import base64
            with open('/app/trace.pcap', 'rb') as f:
                b = f.read()
                print("\n\n=== PCAP AS BASE64 ===\n")
                print(base64.b64encode(b).decode('utf-8'))
                print("\n======================\n")
        except Exception: pass
        sock.close()
        sys.exit(0)
    except Exception as e:
        print(f"[ERR] Loop error: {e}", flush=True)
