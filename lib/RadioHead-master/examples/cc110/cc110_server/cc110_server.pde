// cc110_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RH_CC110 class. RH_CC110 class does not provide for addressing or
// reliability, so you should only use RH_CC110  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example cc110_client
// Tested with Teensy 3.1 and Anaren 430BOOST-CC110L


#include <SPI.h>
#include <RH_CC110.h>

// Singleton instance of the radio driver
RH_CC110 cc110;

void setup() {
    Serial.begin(9600);
    while (!Serial); // wait for serial port to connect. Needed for native USB

    // CC110L may be equipped with either 26 or 27MHz crystals. You MUST
    // tell the driver if a 27MHz crystal is installed for the correct configuration to
    // occur. Failure to correctly set this flag will cause incorrect frequency and modulation
    // characteristics to be used. You can call this function, or pass it to the constructor
    cc110.setIs27MHz(true); // Anaren 430BOOST-CC110L Air BoosterPack test boards have 27MHz
    if (!cc110.init())
        Serial.println("init failed");
    // After init(), the following default values apply:
    // TxPower: TransmitPower5dBm
    // Frequency: 915.0
    // Modulation: GFSK_Rb1_2Fd5_2 (GFSK, Data Rate: 1.2kBaud, Dev: 5.2kHz, RX BW 58kHz, optimised for sensitivity)
    // Sync Words: 0xd3, 0x91
    // But you can change them:
//  cc110.setTxPower(RH_CC110::TransmitPowerM30dBm);
//  cc110.setModemConfig(RH_CC110::GFSK_Rb250Fd127);
//cc110.setFrequency(928.0);
}

void loop() {
    if (cc110.available()) {
        // Should be a message for us now
        uint8_t buf[RH_CC110_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (cc110.recv(buf, &len)) {
//      RH_CC110::printBuffer("request: ", buf, len);
            Serial.print("got request: ");
            Serial.println((char *) buf);
//      Serial.print("RSSI: ");
//      Serial.println(cc110.lastRssi(), DEC);

            // Send a reply
            uint8_t data[] = "And hello back to you";
            cc110.send(data, sizeof(data));
            cc110.waitPacketSent();
            Serial.println("Sent a reply");
        } else {
            Serial.println("recv failed");
        }
    }
}


