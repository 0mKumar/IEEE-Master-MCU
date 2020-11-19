//
// Created by om on 18/11/20.
//

#ifndef AWESOME_HC12_H
#define AWESOME_HC12_H

#include "Arduino.h"
#include "SoftwareSerial.h"

class AwesomeHC12 {
private:
    // address of this node
    uint8_t address;
    const uint8_t setPin;
    unsigned long baudRate;
    uint8_t channel;
    SoftwareSerial HC12;
    bool isStarted{false};
    uint16_t microBaudWaitPerByte;
    uint16_t maxPacketSize;
public:
    AwesomeHC12(const uint8_t txPin,
                const uint8_t rxPin,
                uint8_t setPin,
                const uint8_t address,
                const unsigned int baudRate = 9600,
                const uint8_t channel = 1,
                const uint16_t maxPacketSize = 32) :
            address{address},
            setPin{setPin},
            baudRate{baudRate},
            channel{channel},
            HC12(txPin, rxPin),
            maxPacketSize{maxPacketSize} {
        pinMode(setPin, OUTPUT);
        microBaudWaitPerByte = 2 * (8 * 2 + 1) * (float) 1000000 / (float) baudRate;
        readBuffer = new uint8_t[maxPacketSize];
        writeBuffer = new uint8_t[maxPacketSize];
        cmdResponse = new char[20];
    }

    ~AwesomeHC12() {
        if (isStarted) HC12.end();
        isStarted = false;
    };

    void init();

    void send(uint8_t *payload, size_t size);

    void send(uint8_t *payload, size_t size, uint8_t to);

    void send(const char *str);

    void read(void (*receivePacket)(uint8_t *buf, size_t size, uint8_t from));

    bool read(uint8_t *readBuffer, size_t &len, uint8_t &from);

    bool available();

    bool sendATCommand(const char *cmd);

    void clearReadBuffer();

    char *cmdResponse;
    uint8_t *readBuffer;
    uint8_t *writeBuffer;
};

#endif // AWESOME_HC12_H
