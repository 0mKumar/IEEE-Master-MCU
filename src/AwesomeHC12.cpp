//
// Created by om on 18/11/20.
//

#include "AwesomeHC12.h"

void AwesomeHC12::read(uint8_t *buffer, size_t &len, uint8_t &from) {
    int readIndex = 0;
    bool overflow = false;
    while (HC12.available()) {
        while (HC12.available()) {
            if (readIndex < maxPacketSize) {
                readBuffer[readIndex++] = HC12.read();
            } else {
                HC12.read();
                overflow = true;
            }
        }
        delayMicroseconds(microBaudWaitPerByte);
    }
    if (overflow) {
        Serial.println("Read buffer overflow!");
    }
    len = overflow ? maxPacketSize : readIndex;
    if (len > 0) len -= 1;
    from = readBuffer[0];
    memcpy(buffer, readBuffer + 1, len);
}

void AwesomeHC12::read(void (*receivePacket)(uint8_t *buf, size_t size, uint8_t from)) {
    size_t len;
    uint8_t from;
    read(readBuffer, len, from);
    if (len > 0) {
        receivePacket(readBuffer, len, from);
        Serial.println(microBaudWaitPerByte);
    }
}

void AwesomeHC12::init() {
    Serial.print("Setting baud rate = ");
    Serial.print((long) baudRate);
    HC12.begin(9600);
    char charBuffer[11];

    sprintf(charBuffer, "AT+B%lu", baudRate);
    digitalWrite(setPin, LOW);
    delay(50);
    HC12.write(charBuffer);
    delay(100);

    HC12.end();
    delay(100);
    HC12.begin((long) baudRate);

    while (HC12.available()) {
        Serial.write((char) HC12.read());
    }
    Serial.println();
    sprintf(charBuffer, "AT+C%03d", channel);
    HC12.write(charBuffer);
    delay(100);
    while (HC12.available()) {
        Serial.write((char) HC12.read());
    }
    Serial.println();
    digitalWrite(setPin, HIGH);
    delay(100);
    isStarted = true;
}

void AwesomeHC12::send(uint8_t *payload, size_t size) {
    HC12.write(payload, size);
}

void AwesomeHC12::send(const char *str) {
    send((uint8_t *) str, strlen(str));
}

void AwesomeHC12::send(uint8_t *payload, size_t size, uint8_t to) {
    writeBuffer[0] = to;
    memcpy(writeBuffer + 1, payload, size);
    send(writeBuffer, size + 1);
}

bool AwesomeHC12::available() {
    return HC12.available();
}

void AwesomeHC12::clearReadBuffer() {
    while (HC12.available()) {
        HC12.read();
    }
}

bool AwesomeHC12::sendATCommand(const char *cmd) {
    digitalWrite(setPin, LOW);
    delay(50);
    HC12.write(cmd);
    delay(100);
    bool overflow = true;
    if (HC12.available() <= 20) {
        HC12.readBytes(cmdResponse, HC12.available());
        overflow = false;
    }
    clearReadBuffer();
    digitalWrite(setPin, HIGH);
    delay(100);
    return overflow;
}
