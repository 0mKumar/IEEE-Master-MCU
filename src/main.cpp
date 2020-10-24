#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <RH_NRF24.h>

//const char* ssid = "Om Kumar";
//const char* password = "ppppuuurrrry";

unsigned long startTime, elapsedTime;
int moistureValue = -1;
RH_NRF24 nrf24(2, 4);

#define PACKET_TYPE_SENSOR_DATA 1
#define PACKET_TYPE_MESSAGE 2
#define PACKET_TYPE_RESPONSE 3
#define PACKET_TYPE_INSTRUCTION 4

#define INSTRUCTION_SEND_SENSOR_DATA 1
#define INSTRUCTION_OPEN_VALVE 2
#define INSTRUCTION_CLOSE_VALVE 3

#define RESPONSE_OPENED_VALVE 5
#define RESPONSE_CLOSED_VALVE 6

unsigned int counter = 0;

void reInitialiseNRF() {
    if (!nrf24.init())
        Serial.println("initialization failed");
    if (!nrf24.setChannel(5))
        Serial.println("Channel set failed");
    if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPowerm18dBm))
        Serial.println("RF set failed");
    nrf24.setModeRx();
}

union DataPacket {
    struct {
        int packet_type;
        unsigned int id;
        union {
            struct {
                int moisture_percent;
                float atmospheric_temperature;
                float humidity;
                float soil_temperature;

                void print() const {
                    Serial.print(F("Sensor Data | "));
                    Serial.print(F("moisture : "));
                    Serial.print(moisture_percent);
                    Serial.print(F(" %\t atm_temp : "));
                    Serial.print(atmospheric_temperature);
                    Serial.print(F(" C\t humidity : "));
                    Serial.print(humidity);
                    Serial.print(F(" \t soil_temp : "));
                    Serial.print(soil_temperature);
                    Serial.println(F(" C"));
                }
            } sensor;

            struct {
                int code;
                char text[RH_NRF24_MAX_MESSAGE_LEN - 12];

                void print() {
                    Serial.print(F("Response "));
                    Serial.print(code);
                    Serial.print(F(" | "));
                    Serial.println(text);
                }
            } response;

            struct {
                int code;
                char text[RH_NRF24_MAX_MESSAGE_LEN - 12];

                void print() {
                    Serial.print(F("Instruction "));
                    Serial.print(code);
                    Serial.print(F(" | "));
                    Serial.println(text);
                }
            } instruction;

            char message[RH_NRF24_MAX_MESSAGE_LEN - 8];
        } data;

        void print() {
            Serial.print("#");
            Serial.print(id);
            Serial.print(": ");
            switch (packet_type) {
                case PACKET_TYPE_SENSOR_DATA:
                    data.sensor.print();
                    break;
                case PACKET_TYPE_MESSAGE:
                    Serial.print(F("Slave >> "));
                    Serial.println(data.message);
                    break;
                case PACKET_TYPE_RESPONSE:
                    data.response.print();
                    break;
                case PACKET_TYPE_INSTRUCTION:
                    data.instruction.print();
                    break;
            }
        }
    } packet;

    byte bytes[RH_NRF24_MAX_MESSAGE_LEN];
};

void sendData(uint8_t *bytes, byte length) {
    Serial.println("Sending data...");
    for (byte i = 0; i < length; i++) {
        Serial.print(bytes[i]);
        Serial.print(" ");
    }
    Serial.println();
    nrf24.setModeTx();
    nrf24.send(bytes, length);
    if (!nrf24.waitPacketSent()) {
        Serial.println(F("Transmission Failed!!"));
        reInitialiseNRF();
    }
    nrf24.setModeRx();
}

DataPacket createDataPacket(int type) {
    DataPacket packet{{type}};
    packet.packet.id = counter++;
    return packet;
}

void sendMessage(String text) {
    Serial.println(text);
    DataPacket packet = createDataPacket(PACKET_TYPE_MESSAGE);
    memcpy(packet.packet.data.message, text.begin(), sizeof(text));

    Serial.println("Sending message");
    packet.packet.print();

    sendData(packet.bytes, sizeof(packet.bytes));
}

void sendResponse(int responseCode, String text) {
    Serial.println(text);
    DataPacket packet = createDataPacket(PACKET_TYPE_RESPONSE);
    packet.packet.data.response.code = responseCode;

    memcpy(packet.packet.data.response.text, text.begin(), sizeof(text));

    Serial.println("Sending response");
    packet.packet.print();

    sendData(packet.bytes, sizeof(packet.bytes));
}

void sendInstruction(int instructionCode, String text) {
    Serial.println(text);
    DataPacket packet = createDataPacket(PACKET_TYPE_INSTRUCTION);
    packet.packet.data.instruction.code = instructionCode;
    memcpy(packet.packet.data.response.text, text.begin(), sizeof(text));

    Serial.println("Sending Instruction");
    packet.packet.print();

    sendData(packet.bytes, sizeof(packet.bytes));
}

void readAndConsumeDataPacket() {
    DataPacket dataPacket{};
    uint8_t len = sizeof(dataPacket.bytes);
    if (nrf24.recv(dataPacket.bytes, &len)) {
        dataPacket.packet.print();
        for (int i = 0; i < len; i++) {
            Serial.print(dataPacket.bytes[i]);
            Serial.print(" ");
        }
        Serial.println();
        switch (dataPacket.packet.packet_type) {
            case PACKET_TYPE_INSTRUCTION:
                break;
            case PACKET_TYPE_RESPONSE: {
                switch (dataPacket.packet.data.response.code) {
                    case RESPONSE_CLOSED_VALVE:
                        moistureValue = -1;
                        break;
                    case RESPONSE_OPENED_VALVE:
                        moistureValue = -1;
                        break;
                    default:
                        Serial.println("Unknown response received");
                }
                break;
            }
            case PACKET_TYPE_SENSOR_DATA: {
//                dataPacket.packet.data.sensor.print();
                moistureValue = dataPacket.packet.data.sensor.moisture_percent;
                if (moistureValue < 80) {
                    sendInstruction(INSTRUCTION_OPEN_VALVE, "Open the valve");
                } else {
                    sendInstruction(INSTRUCTION_CLOSE_VALVE, "Close the valve");
                }
            }
        }
    } else {
        Serial.println(F("Receive failed"));
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    reInitialiseNRF();

    Serial.setDebugOutput(true);

    WiFi.mode(WIFI_OFF);
//
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//  Serial.println("\nConnecting to WiFi");
//  while (WiFi.status() != WL_CONNECTED) {
//    Serial.print(".");
//    delay(1000);
//  }
}

void loop() {
    while (nrf24.available()) {
        readAndConsumeDataPacket();
    }

    elapsedTime = millis() - startTime;
    if (elapsedTime > 15000) {
        sendInstruction(INSTRUCTION_SEND_SENSOR_DATA, "Send sensor data");
        startTime = millis();
    }
}
