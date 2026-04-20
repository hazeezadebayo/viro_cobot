#ifndef COBOT_HARDWARE__VIRO_COMM_HPP_
#define COBOT_HARDWARE__VIRO_COMM_HPP_

#include <libserial/SerialPort.h>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <chrono>

namespace cobot_hardware
{

// Standard CRC8 for robot logic (Polynomial 0x07)
inline uint8_t calculate_crc8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0x00;
    for (uint8_t b : data) {
        crc ^= b;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

struct JointFeedback {
    double pos, vel, accel, torque;
};

struct WrenchFeedback {
    double force_x, force_y, force_z;
    double torque_x, torque_y, torque_z;
};

/**
 * CONSOLIDATED COMMUNICATION MODULE
 * Handles the binary protocol layer and serial lifecycle.
 */
class ViroComm
{
public:
    ViroComm() = default;
    ~ViroComm() { close(); }

    bool open(const std::string& port, uint32_t baud) {
        try {
            if (serial_.IsOpen()) serial_.Close();
            serial_.Open(port);
            serial_.SetBaudRate(static_cast<LibSerial::BaudRate>(baud));
            serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_.SetParity(LibSerial::Parity::PARITY_NONE);
            return true;
        } catch (...) { return false; }
    }

    void close() {
        if (serial_.IsOpen()) serial_.Close();
    }

    bool ping() {
        std::vector<uint8_t> p;
        return transact(0x01, p, p);
    }

    bool read_hardware(std::vector<JointFeedback>& joints, WrenchFeedback& wrench) {
        std::vector<uint8_t> p, resp;
        if (transact(0x10, p, resp) && resp.size() >= 120) {
            // Joints: 6 * 16 bytes = 96
            for (size_t i = 0; i < 6; i++) {
                float f[4];
                memcpy(f, &resp[i * 16], 16);
                joints[i] = { (double)f[0], (double)f[1], (double)f[2], (double)f[3] };
            }
            // Wrench: 6 * 4 bytes = 24
            float w[6];
            memcpy(w, &resp[96], 24);
            wrench = { (double)w[0], (double)w[1], (double)w[2], (double)w[3], (double)w[4], (double)w[5] };
            return true;
        }
        return false;
    }

    bool write_joints(const std::vector<double>& cmds, int mode) {
        // Simple safety clamp
        std::vector<uint8_t> p = { (uint8_t)mode };
        for (double v : cmds) {
            float f = (float)v;
            uint8_t b[4]; memcpy(b, &f, 4);
            for (int i = 0; i < 4; i++) p.push_back(b[i]);
        }
        std::vector<uint8_t> resp;
        return transact(0x11, p, resp);
    }

private:
    LibSerial::SerialPort serial_;

    bool transact(uint8_t cmd, const std::vector<uint8_t>& p, std::vector<uint8_t>& resp) {
        // Send $CMD LEN PAYLOAD CRC
        std::vector<uint8_t> pkt = { 0x24, cmd, (uint8_t)p.size() };
        pkt.insert(pkt.end(), p.begin(), p.end());
        pkt.push_back(calculate_crc8(pkt));
        
        try {
            for (uint8_t b : pkt) serial_.WriteByte(b);
            serial_.DrainWriteBuffer();

            // Receive $CMD LEN PAYLOAD CRC
            uint8_t b; 
            auto start = std::chrono::steady_clock::now();
            // Sync to start byte with 100ms timeout
            while (true) {
                serial_.ReadByte(b, 50);
                if (b == 0x24) break;
                if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(100)) return false;
            }

            uint8_t r_cmd, r_len;
            serial_.ReadByte(r_cmd, 10); 
            serial_.ReadByte(r_len, 10);
            
            resp.clear();
            for (int i = 0; i < r_len; i++) { 
                serial_.ReadByte(b, 10); 
                resp.push_back(b); 
            }
            
            uint8_t r_crc;
            serial_.ReadByte(r_crc, 10);
            
            std::vector<uint8_t> check = { 0x24, r_cmd, r_len };
            check.insert(check.end(), resp.begin(), resp.end());
            
            if (r_crc != calculate_crc8(check)) return false;
            return (r_cmd == cmd);
        } catch (...) { 
            return false; 
        }
    }
};

} // namespace cobot_hardware

#endif
