#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <RH_NRF24.h>

//const char* ssid = "Om Kumar";
//const char* password = "ppppuuurrrry";

unsigned long StartTime, CurrentTime, ElapsedTime;
bool flag = false;
int MoistureValue = -1;
RH_NRF24 nrf24(2, 4);

void printSensorValue(uint8_t arr[]) {
    int moisture = arr[0];
    unsigned long dht_temp_b =
            ((arr[4] & 0xff) << 24) | ((arr[3] & 0xff) << 16) | ((arr[2] & 0xff) << 8) | (arr[1] & 0xff);
    unsigned long humidity_b =
            ((arr[8] & 0xff) << 24) | ((arr[7] & 0xff) << 16) | ((arr[6] & 0xff) << 8) | (arr[5] & 0xff);
    unsigned long soil_temp_b =
            ((arr[12] & 0xff) << 24) | ((arr[11] & 0xff) << 16) | ((arr[10] & 0xff) << 8) | (arr[9] & 0xff);
    float dht_temp = *(float *) &dht_temp_b;
    float humidity = *(float *) &humidity_b;
    float soil_temp = *(float *) &soil_temp_b;
    Serial.print(moisture);
    Serial.print(" ");
    Serial.print(dht_temp);
    Serial.print(" ");
    Serial.print(humidity);
    Serial.print(" ");
    Serial.println(soil_temp);
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Hello");

    if (!nrf24.init())
        Serial.println("initialization failed");
    if (!nrf24.setChannel(5))
        Serial.println("Channel set failed");
    if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
        Serial.println("RF set failed");

    Serial.setDebugOutput(true);
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
    if (flag == false) {
        String data = "";
        if (MoistureValue == -1)
            data = String("Attempt to connect ");
        else {
            if (MoistureValue < 80)
                data = String("Open Valve ");
            else
                data = String("Close Valve ");
        }
        Serial.println(data);
        uint8_t dataArray[data.length()];
        data.getBytes(dataArray, data.length());
        nrf24.send(dataArray, sizeof(dataArray));
        nrf24.waitPacketSent();
        flag = true;
        StartTime = millis();
    }
    //Serial.println(flag);
    if (nrf24.available()) {
        String str1 = "Valve Opened", str2 = "Valve Closed";
        bool openedvalve = true, closedvalve = true;
        uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (nrf24.recv(buf, &len)) {
            for (int i = 0; i < str1.length(); i++) {
                if (str1[i] != buf[i]) {
                    openedvalve = false;
                    break;
                }
            }

            for (int i = 0; i < str2.length(); i++) {
                if (str2[i] != buf[i]) {
                    closedvalve = false;
                    break;
                }
            }

            if (openedvalve == false && closedvalve == false) {
                Serial.print(
                        "Soil Mositure Percent, Atmospheric Temperature, Atmospheric Humidity, Soil Temperature : ");
                printSensorValue(buf);
                MoistureValue = (int) buf[0];
            } else {
                Serial.println((char *) buf);
                MoistureValue = -1;
            }
            flag = false;
        } else {
            Serial.println("recv failed");
        }
    }

    CurrentTime = millis();
    ElapsedTime = CurrentTime - StartTime;
    if (ElapsedTime > 10000)
        flag = false;
}
