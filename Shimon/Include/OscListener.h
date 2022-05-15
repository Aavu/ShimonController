//
// Created by Raghavasimhan Sankaranarayanan on 4/9/22.
//

#ifndef SHIMONCONTROLLER_OSCLISTENER_H
#define SHIMONCONTROLLER_OSCLISTENER_H

#include <iostream>
#include <cstdint>
#include <functional>
#include <optional>

#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "ip/UdpSocket.h"

#include "Def.h"

class OscListener: public osc::OscPacketListener {
public:
    explicit OscListener(int port) : m_socket(IpEndpointName(IpEndpointName::ANY_ADDRESS, port), this) {}

    void setSystemMsgCallback(std::function<void(const char*)> callback_fn) {
        m_sysMsgCallback = std::move(callback_fn);
    }
    void setArmMidiCallback(std::function<void(int, int)> callback_fn) {
        m_armMidiCallback = std::move(callback_fn);
    }

    void setArmServoCallback(std::function<void(const char*)> callback_fn) {
        m_armServoCallback = std::move(callback_fn);
    }

    void setArmRawCallback(std::function<void(int, int, float, float)> callback_fn) {
        m_armCallback = std::move(callback_fn);
    }

    void setHeadCallback(std::function<void(const char*, std::optional<float>, std::optional<float>)> callback_fn) {
        m_headCallback = std::move(callback_fn);
    }

    void setStrikerRawCallback(std::function<void(char, uint8_t, int, int, int)> callback_fn) {
        m_strikerCallback = std::move(callback_fn);
    }

    void setStrikerMidiCallback(std::function<void(char, uint8_t, uint8_t)> callback_fn) {
        m_strikerMidiCallback = std::move(callback_fn);
    }

    void start() {
        m_bRunning = true;
        m_socket.Run();
    }

    void stop() {
        if (!m_bRunning) return;
        m_socket.AsynchronousBreak();
        m_bRunning = false;
    }

protected:
    void ProcessMessage( const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint ) override {
        try{
            osc::uint32 count = m.ArgumentCount();
            osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
            const char* typeTags = m.TypeTags();
            if( std::strcmp( m.AddressPattern(), ARM_CMD_OSC_ROUTE) == 0 ) {
                processArmMessage(args, count);
            } else if (std::strcmp( m.AddressPattern(), STRIKER_CMD_OSC_ROUTE) == 0) {
                processStrikerMessage(args, count);
            } else if (std::strcmp( m.AddressPattern(), HEAD_CMD_OSC_ROUTE) == 0) {
                processHeadMessage(args, count, typeTags);
            } else if (std::strcmp( m.AddressPattern(), SHIMON_OSC_ROUTE) == 0) {
                processSysMessage(args, count, typeTags);
            }

        } catch( osc::Exception& e ) {
            // any parsing errors such as unexpected argument types, or
            // missing arguments get thrown as exceptions.
            std::cout << "error while parsing message: "
                      << m.AddressPattern() << ": " << e.what() << "\n";
        }
    }

private:
    UdpListeningReceiveSocket m_socket;
    std::atomic<bool> m_bRunning = false;
    std::function<void(int, int)> m_armMidiCallback = nullptr;
    std::function<void(int, int, float, float)> m_armCallback = nullptr;
    std::function<void(const char*)> m_armServoCallback = nullptr;
    std::function<void(const char*)> m_sysMsgCallback = nullptr;

    /* TODO: Replace with variadic functions */
    std::function<void(const char*, std::optional<float>, std::optional<float>)> m_headCallback = nullptr;

    std::function<void(char, uint8_t, uint8_t)> m_strikerMidiCallback = nullptr;
    std::function<void(char, uint8_t, int, int, int)> m_strikerCallback = nullptr;

    void processArmMessage(osc::ReceivedMessageArgumentStream& args, uint32_t count) {
        if (count == 1) {   // One of the servo commands
            const char* cmd;
            args >> cmd >> osc::EndMessage;
            if (m_armServoCallback) m_armServoCallback(cmd);
        } else if (count == 2) {   // Midi format
            osc::int32 note, velocity;
            args >> note >> velocity >> osc::EndMessage;
            if (m_armMidiCallback) m_armMidiCallback((int)note, (int)velocity);
        } else {    // Raw format
            osc::int32 armId, position, v_max, hit_vel, time;
            float acc;
            if (count == 4) {
                args >> armId >> position >> acc >> v_max >> osc::EndMessage;
            } else if (count == 6) {    // Backward compatibility
                args >> armId >> position >> acc >> v_max >> hit_vel >> time >> osc::EndMessage;
            } else {
                LOG_WARN("Unknown command format");
                return;
            }
            if (m_armCallback) m_armCallback((int)armId-1, (int)position, acc, (float)v_max);
        }
    }

    void processStrikerMessage(osc::ReceivedMessageArgumentStream& args, uint32_t count) {
        osc::int32 strikerIds;
        char type;
        if (count == 3) {   // Midi format
            osc::int32 midiVelocity;
            args >> type >> strikerIds >> midiVelocity >> osc::EndMessage;
            if (strikerIds < 0 || strikerIds > 0xFF) return;
            if (m_strikerMidiCallback) m_strikerMidiCallback(type, strikerIds, midiVelocity);
        } else if (count == 5) {   // Raw format
            osc::int32 dummy, position, acceleration;
            args >> type >> strikerIds >> dummy >> position >> acceleration >> osc::EndMessage;
            if (strikerIds < 0 || strikerIds > 0xFF) return;
            if (m_strikerCallback) m_strikerCallback(type, (int)strikerIds, (int)dummy, (int)position, (int)acceleration);
        }
    }

    void processHeadMessage(osc::ReceivedMessageArgumentStream& args, uint32_t count, const char* typeTags) {
        const char* cmd;
        if (strcmp(typeTags, "sii") == 0) {
            osc::int32 val1, val2;
            args >> cmd >> val1 >> val2 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, (float)val1, (float)val2);
        } else if (strcmp(typeTags, "sif") == 0) {
            osc::int32 val1;
            float val2;
            args >> cmd >> val1 >> val2 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, (float)val1, val2);
        } else if (strcmp(typeTags, "sfi") == 0) {
            osc::int32 val2;
            float val1;
            args >> cmd >> val1 >> val2 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, val1, (float)val2);
        } else if (strcmp(typeTags, "sff") == 0) {
            float val1, val2;
            args >> cmd >> val1 >> val2 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, val1, val2);
        } else if (strcmp(typeTags, "sf") == 0) {
            float val1;
            args >> cmd >> val1 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, val1, std::optional<float>());
        } else if (strcmp(typeTags, "si") == 0) {
            osc::int32 val1;
            args >> cmd >> val1 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, (float)val1, std::optional<float>());
        } else if (strcmp(typeTags, "s") == 0) {
            args >> cmd >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, std::optional<float>(), std::optional<float>());
        } else {
            LOG_WARN("unknown typeTag: {}", typeTags);
        }
    }

    void processSysMessage(osc::ReceivedMessageArgumentStream& args, uint32_t count, const char* typeTags) {
        const char* cmd;
        if (strcmp(typeTags, "s") == 0) {
            args >> cmd >> osc::EndMessage;
            if (m_sysMsgCallback) m_sysMsgCallback(cmd);
        }
    }
};


#endif //SHIMONCONTROLLER_OSCLISTENER_H
