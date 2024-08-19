#include <SPI.h>
#include <LoRa.h>
#include "DHT.h"

#define SS_PIN 22
#define RST_PIN 15
#define DHT_PIN 14
#define MQ_PIN 27
#define VOLTAGE_PIN 13
#define DIO0_PIN 21

#define DHTTYPE DHT11

/*
NodeData nodes[] = {
  {1, 21.37, 24.11, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {2, 7.62, 65.98, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {3, 74.63, 89.28, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {4, 70.73, 67.54, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {5, 53.65, 42.44, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {6, 95.90, 18.32, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {7, 94.48, 3.31, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {8, 28.01, 95.86, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {9, 51.58, 67.39, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {10, 2.20, 22.54, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {11, 26.23, 67.35, 100.00, false, false, 0.0, 0.0, 0, 0.0},
  {12, 99.24, 82.93, 100.00, false, false, 0.0, 0.0, 0, 0.0},
};
*/
const int nodeID = 6;  // Set the unique node ID here
const float posX = 95.90;
const float posY = 18.32;
const float initialEnergy = 4.20;
const int sinkID = 99;  // Use a specific ID for Sink

unsigned long previousMillis = 0;
const long interval = 5000;  // Interval for sensor data transmission (5 seconds)
bool dataAcknowledged = false;  // Flag to check if data is acknowledged
bool phase2DataAcknowledged = false;  // Flag to check if phase 2 data is acknowledged
bool waitingForNextRound = false; // Flag to wait for the next round
unsigned long syncTime = 0;  // To store synchronized time from sink

DHT dht(DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(1000);
  Serial.println("Starting Node Setup...");

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  dht.begin();

  Serial.println("LoRa Node Initialized");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incomingData = "";
    while (LoRa.available()) {
      incomingData += (char) LoRa.read();
    }

    // Check if the incoming data is from the Sink
    if (incomingData.startsWith(String(sinkID) + ";")) {
      Serial.print("Received data: ");
      Serial.println(incomingData);

      // Remove the sinkID prefix before processing
      incomingData = incomingData.substring(incomingData.indexOf(';') + 1);

      if (incomingData.startsWith("SYNC")) {
        String syncMessage = String(sinkID) + ";TIME;" + String(millis());
        LoRa.beginPacket();
        LoRa.print(syncMessage);
        LoRa.endPacket();
        Serial.println("Sent SYNC response: " + syncMessage);
      } else if (incomingData.startsWith("TIME")) {
        syncTime = incomingData.substring(5).toInt();
        Serial.print("Synchronized time with sink: ");
        Serial.println(syncTime);
        // Ready to send phase 2 data after synchronization
        phase2DataAcknowledged = false;
        dataAcknowledged = true;  // Reset phase 1 acknowledgment
        waitingForNextRound = false;
      } else if (incomingData.startsWith("REQ")) {
        sendInitialData();
      } else if (incomingData.startsWith("ACK")) {
        int ackId = incomingData.substring(4).toInt();
        if (ackId == nodeID) {
          Serial.println("ACK received for initial data.");
          dataAcknowledged = true;
        }
      } else if (incomingData.startsWith("REQ2")) {
        if (dataAcknowledged) {  // Only send phase 2 data if initial data has been acknowledged
          sendSensorData();
        }
      } else if (incomingData.startsWith("ACK2")) {
        int ackId = incomingData.substring(5).toInt();
        if (ackId == nodeID) {
          Serial.println("ACK received for sensor data.");
          phase2DataAcknowledged = true;
          waitingForNextRound = true;
        }
      } else if (incomingData.startsWith("NEXT")) {
        Serial.println("Received start of next round.");
        dataAcknowledged = false;
        phase2DataAcknowledged = false;
        waitingForNextRound = false;
        sendInitialData(); // Reset and send initial data for the new round
      } else {
        Serial.println("Received unknown command.");
      }
    } else {
      Serial.println("Ignored data not from Sink.");
    }
  }

  if (!waitingForNextRound) {
    if (dataAcknowledged && !phase2DataAcknowledged) {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        sendSensorData();
      }
    } else if (!dataAcknowledged) {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        sendInitialData();
      }
    }
  }
}

void sendInitialData() {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%d;INIT;%d;%.2f,%.2f;%.2f", sinkID, nodeID, posX, posY, initialEnergy);  // Menambahkan sinkID di awal pesan
  LoRa.beginPacket();
  LoRa.print(buffer);
  LoRa.endPacket();
  Serial.println("Mengirimkan Data Initial (ID, posisi, Energi Awal): " + String(buffer));
}

void sendSensorData() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int gas = analogRead(MQ_PIN);
  float voltage = analogRead(VOLTAGE_PIN) * (4.2 / 2048.0);  // Adjust the voltage calculation as needed

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%d;PHASE2;%d;%.2f;%.2f;%d;%.2f", sinkID, nodeID, temperature, humidity, gas, voltage);  // Menambahkan sinkID di awal pesan
  LoRa.beginPacket();
  LoRa.print(buffer);
  LoRa.endPacket();
  Serial.println("Sent sensor data (ID, Suhu, kelembaban, Gas, Tegangan): " + String(buffer));
}
