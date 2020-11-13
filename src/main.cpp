#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>

#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ArduinoJson.h"

#define THIS_ADDRESS 128

const char *ssid = "ESP8266-Access-Point";
const char *password = "123456789";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

int moistureValue = -1;
RH_NRF24 driver(2, 4);

RHReliableDatagram manager(driver, THIS_ADDRESS);

#define PACKET_TYPE_SENSOR_DATA 1
#define PACKET_TYPE_MESSAGE 2
#define PACKET_TYPE_RESPONSE 3
#define PACKET_TYPE_INSTRUCTION 4
#define PACKET_TYPE_RECEIVE_SUCCESS 5

#define INSTRUCTION_SEND_SENSOR_DATA 1
#define INSTRUCTION_OPEN_VALVE 2
#define INSTRUCTION_CLOSE_VALVE 3
#define INSTRUCTION_PREPARE_SENSOR_DATA 4

#define RESPONSE_OPENED_VALVE 5
#define RESPONSE_CLOSED_VALVE 6

#define NUMBER_OF_SLAVES 3
#define ADDRESS_SLAVE_1 1

unsigned int counter = 0;

void reInitialiseNRF() {
    if (!manager.init())
        Serial.println("init failed");
    driver.setChannel(2);
    manager.setRetries(10);
    manager.setTimeout(300);
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
                case PACKET_TYPE_RECEIVE_SUCCESS:
                    Serial.println("Receive success ack");
                    break;
            }
        }
    } packet;

    byte bytes[RH_NRF24_MAX_MESSAGE_LEN];
};

#define VALVE_STATE_UNKNOWN 0
#define VALVE_STATE_OPEN 2
#define VALVE_STATE_CLOSE 3

struct Slave {
    uint8_t address{};
    unsigned long lastRespondedAt = 0;
    unsigned long lastCommunicationTryAt = 0;
    DataPacket sensor{};
    uint8_t valveState = VALVE_STATE_UNKNOWN;

    bool isConnected() const {
        return lastRespondedAt > lastCommunicationTryAt;
    }
};

Slave slaves[NUMBER_OF_SLAVES];

void sendInstruction(int instructionCode, String text, uint8_t to);

void readAndConsumeDataPacket() {
    DataPacket dataPacket{};
    uint8_t len = sizeof(dataPacket.bytes);
    uint8_t from;
    if (manager.recvfromAckTimeout(dataPacket.bytes, &len, 4000, &from)) {
        Slave &slave = slaves[from - 1];
        slave.lastRespondedAt = millis();
        dataPacket.packet.print();
        for (int i = 0; i < len; i++) {
            Serial.print(dataPacket.bytes[i], HEX);
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
                memcpy(slave.sensor.bytes, dataPacket.bytes, sizeof(dataPacket.bytes));
                slave.address = from;
//                dataPacket.packet.data.sensor.print();
                moistureValue = dataPacket.packet.data.sensor.moisture_percent;
                if (moistureValue < 80) {
                    sendInstruction(INSTRUCTION_OPEN_VALVE, "Open the valve", from);
                } else {
                    sendInstruction(INSTRUCTION_CLOSE_VALVE, "Close the valve", from);
                }
            }
        }
    } else {
        Serial.println(F("No reply from slave"));
    }
}

void sendData(uint8_t *bytes, byte length, uint8_t to) {
    Serial.print("Sending data to ");
    Serial.println(to, HEX);
    for (byte i = 0; i < length; i++) {
        Serial.print(bytes[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    slaves[to - 1].lastCommunicationTryAt = millis();
    if (manager.sendtoWait(bytes, length, to)) {
        readAndConsumeDataPacket();
    } else {
        Serial.println(F("sendtoWait Failed!!"));
    }
}

DataPacket createDataPacket(int type) {
    DataPacket packet{{type}};
    packet.packet.id = counter++;
    return packet;
}

void sendMessage(String text, uint8_t to) {
    Serial.println(text);
    DataPacket packet = createDataPacket(PACKET_TYPE_MESSAGE);
    memcpy(packet.packet.data.message, text.begin(), sizeof(text));

    Serial.println("Sending message");
    packet.packet.print();

    sendData(packet.bytes, sizeof(packet.bytes), to);
}

void sendResponse(int responseCode, String text, uint8_t to) {
    Serial.println(text);
    DataPacket packet = createDataPacket(PACKET_TYPE_RESPONSE);
    packet.packet.data.response.code = responseCode;

    memcpy(packet.packet.data.response.text, text.begin(), sizeof(text));

    Serial.println("Sending response");
    packet.packet.print();

    sendData(packet.bytes, sizeof(packet.bytes), to);
}

void sendInstruction(int instructionCode, String text, uint8_t to) {
    Serial.println(text);
    DataPacket packet = createDataPacket(PACKET_TYPE_INSTRUCTION);
    packet.packet.data.instruction.code = instructionCode;
    memcpy(packet.packet.data.response.text, text.begin(), sizeof(text));

    Serial.println("Sending Instruction");
    packet.packet.print();

    sendData(packet.bytes, sizeof(packet.bytes), to);
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    reInitialiseNRF();

    Serial.setDebugOutput(true);

    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Print ESP8266 Local IP Address
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", "Hello from master!");
    });

    server.on("/slave_sensor", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasArg("slave_id")) {
            String slaveId = request->arg("slave_id");
            byte id = (byte) slaveId.toInt();
            if (id >= ADDRESS_SLAVE_1 && id < ADDRESS_SLAVE_1 + NUMBER_OF_SLAVES) {
                const size_t capacity = JSON_OBJECT_SIZE(5);
                DynamicJsonDocument doc(capacity);

                Slave &slave = slaves[id - 1];
                doc["sid"] = id;
                doc["at"] = slave.sensor.packet.data.sensor.atmospheric_temperature;
                doc["ah"] = slave.sensor.packet.data.sensor.humidity;
                doc["sm"] = slave.sensor.packet.data.sensor.moisture_percent;
                doc["st"] = slave.sensor.packet.data.sensor.soil_temperature;

                char buffer[256];
                serializeJson(doc, buffer);
                request->send_P(200, "application/json", buffer);
            } else {
                request->send_P(300, "application/json", "bad request: slave with given slave_id does not exist");
            }
        } else {
            request->send_P(300, "application/json", "bad request: slave id not provided");
        }
    });

    server.on("/valve", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasArg("slave_id") && request->hasArg("new_state")) {
            String slaveId = request->arg("slave_id");
            String newValveState = request->arg("new_state");
            byte id = (byte) slaveId.toInt();
            if (id >= ADDRESS_SLAVE_1 && id < ADDRESS_SLAVE_1 + NUMBER_OF_SLAVES) {

                if (newValveState == "open") {
                    sendInstruction(INSTRUCTION_OPEN_VALVE, "App request open", id);
                } else if (newValveState == "close") {
                    sendInstruction(INSTRUCTION_CLOSE_VALVE, "App request close", id);
                }

                const size_t capacity = JSON_OBJECT_SIZE(2);
                DynamicJsonDocument doc(capacity);

                doc["sid"] = id;
                doc["ac"] = "OK";

                char buffer[256];
                serializeJson(doc, buffer);
                request->send_P(200, "application/json", buffer);
            } else {
                request->send_P(300, "text/plain", "bad request: slave with given slave_id does not exist");
            }
        } else {
            request->send_P(300, "text/plain", "bad request: slave_id or new_state not provided");
        }
    });

    server.on("/motor", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasArg("power")) {
            String power = request->arg("power");

            if (power == "on") {

            } else if (power == "off") {

            }

            const size_t capacity = JSON_OBJECT_SIZE(1);
            DynamicJsonDocument doc(capacity);

            doc["ac"] = "OK";

            char buffer[256];
            serializeJson(doc, buffer);
            request->send_P(200, "application/json", buffer);

        } else {
            request->send_P(300, "text/plain", "bad request: slave_id or new_state not provided");
        }
    });

    // Start server
    server.begin();
}

unsigned long currentStateSince = millis();
unsigned long elapsedTime;

#define STATE_PREPARE_SENSOR_DATA 1
#define STATE_IDLE_FOR_SENSOR_DATA 2
#define STATE_SEND_SENSOR_DATA 3

int state = STATE_PREPARE_SENSOR_DATA;

void loop() {
    elapsedTime = millis() - currentStateSince;
    switch (state) {
        case STATE_PREPARE_SENSOR_DATA:
            for (int to = ADDRESS_SLAVE_1; to <= NUMBER_OF_SLAVES; to++) {
                sendInstruction(INSTRUCTION_PREPARE_SENSOR_DATA, "Prepare sensor data", to);
            }
            state = STATE_IDLE_FOR_SENSOR_DATA;
            currentStateSince = millis();
            break;
        case STATE_IDLE_FOR_SENSOR_DATA:
            if (elapsedTime > 5000) {
                state = STATE_SEND_SENSOR_DATA;
                currentStateSince = millis();
            }
            break;
        case STATE_SEND_SENSOR_DATA:
            for (int to = ADDRESS_SLAVE_1; to <= NUMBER_OF_SLAVES; to++) {
                sendInstruction(INSTRUCTION_SEND_SENSOR_DATA, "Send sensor data", to);
            }
            state = STATE_PREPARE_SENSOR_DATA;
            currentStateSince = millis();
            break;
    }
}
