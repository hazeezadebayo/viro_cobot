#!/usr/bin/env python3
import os, pty, struct, time, threading, math

# MODULE 3: Emulator Script
# Simulates Serial Bus + Motor Dynamics + Fake FT Sensor

def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c<<1)^7 if c&128 else c<<1
            c &= 255
    return c

class ViroSim:
    def __init__(self):
        m, s = pty.openpty()
        self.m = m
        self.p = os.ttyname(s)
        print(f"[*] Emulator at: {self.p}")
        if os.path.exists("/tmp/ttyViro"): os.remove("/tmp/ttyViro")
        os.symlink(self.p, "/tmp/ttyViro")
        
        self.states = [[0.0]*4 for _ in range(6)] # p, v, a, t
        self.wrench = [0.0]*6 # fx, fy, fz, tx, ty, tz
        self.cmds = [0.0]*6
        threading.Thread(target=self.step, daemon=True).start()

    def step(self):
        start_time = time.time()
        while True:
            now = time.time() - start_time
            for i in range(6):
                # Simple PD Follower
                err = self.cmds[i] - self.states[i][0]
                self.states[i][1] = err * 5.0
                self.states[i][0] += self.states[i][1] * 0.01
                self.states[i][3] = err * 10.0 # Torque
            
            # Fake Wrench Simulation (Force-Torque)
            # Simulate a 1kg mass oscillating on the flange
            base_force = 9.81
            self.wrench[0] = 0.5 * math.sin(now * 2.0)
            self.wrench[1] = 0.5 * math.cos(now * 2.0)
            self.wrench[2] = base_force + 0.2 * math.sin(now * 5.0)
            self.wrench[3] = 0.1 * math.sin(now)
            self.wrench[4] = 0.1 * math.cos(now)
            self.wrench[5] = 0.05 * math.sin(now * 3.0)

            time.sleep(0.01)

    def run(self):
        while True:
            # Sync to '$'
            while os.read(self.m, 1) != b'$': pass
            cmd_read = os.read(self.m, 1)
            if not cmd_read: continue
            cmd = cmd_read[0]
            
            len_read = os.read(self.m, 1)
            if not len_read: continue
            length = len_read[0]
            
            payload = os.read(self.m, length)
            os.read(self.m, 1) # Skip CRC

            resp = bytearray()
            if cmd == 0x10: # Read All (Joints + FT Sensor)
                # 6 joints * 16 bytes = 96
                for s in self.states: resp += struct.pack('ffff', *s)
                # Wrench * 24 bytes
                resp += struct.pack('ffffff', *self.wrench)
            elif cmd == 0x11: # Write All
                for i in range(6): 
                    self.cmds[i] = struct.unpack('f', payload[1+i*4:5+i*4])[0]
            
            pkt = bytearray([ord('$'), cmd, len(resp)]) + resp
            pkt.append(crc8(pkt))
            os.write(self.m, pkt)

if __name__ == "__main__":
    ViroSim().run()
