#include <SPI.h>
#include <LoRa.h>
#include <vector>
#include <algorithm>

#define SS_PIN 22
#define RST_PIN 15
#define DIO0_PIN 21

#define RSSI_THRESHOLD -90 // Nilai threshold untuk RSSI

const int sinkID = 99;  // Use a specific ID for Sink
unsigned long previousMillisSync = 0;
unsigned long previousMillisReq = 0;
const long syncInterval = 15000;  // Interval for synchronization requests (15 seconds)
const long reqInterval = 10000;  // Interval for requesting initial data (10 seconds)
const long roundInterval = 30000;  // Total interval for each round (30 seconds)
unsigned long startTime = 0;

struct NodeData {
  int id;
  float posX;
  float posY;
  float initialEnergy;
  bool dataAcknowledged;
  bool phase2DataAcknowledged;
  float temperature;
  float humidity;
  int gas;
  float voltage;
  unsigned long lastHeard; // Waktu terakhir data diterima
  bool isActive; // Status aktif node
  int receivedBytes; // Jumlah byte yang diterima dari data phase2
  bool isCH; // Apakah node adalah CH
  int cluster; // Nomor cluster
  int chID; // ID dari CH
  float chX; // Posisi X dari CH
  float chY; // Posisi Y dari CH
};

std::vector<NodeData> allData;  // Kontainer untuk menyimpan semua data yang diterima
std::vector<int> isolatedNodes; // Kontainer untuk menyimpan ID node yang terisolasi

NodeData nodes[] = {
  {4, 70.73, 67.54, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, false, 0, 0, 0.0, 0.0},
  {6, 95.90, 18.32, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, false, 0, 0, 0.0, 0.0},
  {8, 30.20, 64.00, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, false, 0, 0, 0.0, 0.0},
  //{10, 2.20, 22.54, 4.20, false, false, 0.0, 0.0, 0, 0.0, 0, false, false, 0, 0, 0.0, 0.0}
};

bool allNodesAcknowledgedFlag = false;
int currentRound = 1; // Variable to track the current round

void clearSerialMonitor() {
  for (int i = 0; i < 10; i++) {
    Serial.println();
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(1000);
  Serial.println("Starting Sink Setup...");

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Sink Initialized");
  Serial.println("Starting round " + String(currentRound)); // Display the initial round
  
  startTime = millis(); // Initialize start time
  sendSyncRequest(); // Send initial sync request at the start of each round
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - startTime;

  if (elapsedTime < syncInterval) {
    // Handle synchronization
    if (currentMillis - previousMillisSync >= syncInterval) {
      previousMillisSync = currentMillis;
      sendSyncRequest();
    }
  } else if (elapsedTime < syncInterval + reqInterval) {
    // Handle initial data request
    if (currentMillis - previousMillisReq >= reqInterval) {
      previousMillisReq = currentMillis;
      requestInitialData();
    }
  } else if (elapsedTime < roundInterval) {
    // Handle phase 2 data request if all initial data is acknowledged
    if (allNodesAcknowledged()) {
      if (!allNodesAcknowledgedFlag) {
        Serial.println("All nodes acknowledged. Transitioning to phase 2.");
        allNodesAcknowledgedFlag = true;
      }
      requestPhase2Data();
    }
  } else {
    // Handle end of round
    Serial.println("Round time exceeded. Reporting and moving to next round.");
    // Perform CH Selection before printing summary
    CHSelection();
    assignNodesToClusters(); // Assign nodes to clusters
    printSummary();
    identifyIsolatedNodes(); // Identify isolated nodes
    reClusterIsolatedNodes(); // Re-cluster isolated nodes
    clearSerialMonitor();
    currentRound++;
    Serial.println("Starting round " + String(currentRound));
    resetForNextRound();
    sendNextRoundSignal(); // Send signal to start the next round
    startTime = millis(); // Reset start time for the new round
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incomingData = "";
    while (LoRa.available()) {
      incomingData += (char) LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    Serial.print("Received: ");
    Serial.println(incomingData);
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" SNR: ");
    Serial.println(snr);

    processIncomingData(incomingData, rssi);
  }
}

void sendSyncRequest() {
  LoRa.beginPacket();
  LoRa.print(String(sinkID) + ";SYNC");
  LoRa.endPacket();
  Serial.println("Mengirimkan permintaan SYNC pada semua Node.");
}

void sendNextRoundSignal() {
  LoRa.beginPacket();
  LoRa.print(String(sinkID) + ";NEXT");
  LoRa.endPacket();
  Serial.println("Mengirimkan sinyal untuk memulai ronde berikutnya pada semua Node.");
}

//Steady-State
void requestInitialData() {
  bool anyUnacknowledged = false;
  for (NodeData &node : nodes) {
    if (!node.dataAcknowledged) {
      anyUnacknowledged = true;
      LoRa.beginPacket();
      LoRa.print(String(sinkID) + ";REQ;" + String(node.id));
      LoRa.endPacket();
      Serial.println("Mengirimkan permintaan data inisialisasi ke Node ID: " + String(node.id));
    }
  }
  if (!anyUnacknowledged) {
    allNodesAcknowledgedFlag = true;
  }
}

void requestPhase2Data() {
  for (NodeData &node : nodes) {
    if (node.dataAcknowledged && !node.phase2DataAcknowledged) {
      LoRa.beginPacket();
      LoRa.print(String(sinkID) + ";REQ2;" + String(node.id));
      LoRa.endPacket();
      Serial.println("Mengirimkan permintaan data fase 2 ke Node ID: " + String(node.id));
    }
  }
}

void processIncomingData(String data, int rssi) {
  Serial.print("Processing data: ");
  Serial.println(data);

  if (data.startsWith(String(sinkID) + ";")) {
    data = data.substring(data.indexOf(';') + 1);

    Serial.print("Data after removing sinkID: ");
    Serial.println(data);

    if (data.startsWith("INIT")) {
      Serial.println("Received INIT data");
      int nodeId = data.substring(5, data.indexOf(';', 5)).toInt();
      
      // Validasi ID node
      if (nodeId == 0) {
        Serial.println("Invalid Node ID: 0, ignoring data.");
        return;
      }
      
      String posData = data.substring(data.indexOf(';', 5) + 1);
      float posX = posData.substring(0, posData.indexOf(',')).toFloat();
      float posY = posData.substring(posData.indexOf(',') + 1, posData.indexOf(';')).toFloat();
      float initialEnergy = posData.substring(posData.indexOf(';') + 1).toFloat();

      Serial.print("Parsed INIT data - nodeId: ");
      Serial.print(nodeId);
      Serial.print(", posX: ");
      Serial.print(posX);
      Serial.print(", posY: ");
      Serial.print(posY);
      Serial.print(", initialEnergy: ");
      Serial.println(initialEnergy);

      storeInitialData(nodeId, posX, posY, initialEnergy);

      // Kirim ACK ke node
      LoRa.beginPacket();
      LoRa.print(String(sinkID) + ";ACK;" + String(nodeId));
      LoRa.endPacket();
      Serial.println("Pengiriman ACK ke Node ID: " + String(nodeId));

      // Update waktu terakhir data diterima
      updateLastHeard(nodeId);
      // Tandai node sebagai aktif
      setActiveStatus(nodeId, true);
    } else if (data.startsWith("PHASE2")) {
      Serial.println("Received PHASE2 data");
      int nodeId = data.substring(7, data.indexOf(';', 7)).toInt();
      
      // Validasi ID node
      if (nodeId == 0) {
        Serial.println("Invalid Node ID: 0, ignoring data.");
        return;
      }
      
      String sensorData = data.substring(data.indexOf(';', 7) + 1);
      float temperature = sensorData.substring(0, sensorData.indexOf(';')).toFloat();
      float humidity = sensorData.substring(sensorData.indexOf(';') + 1, sensorData.indexOf(';', sensorData.indexOf(';') + 1)).toFloat();
      int gas = sensorData.substring(sensorData.indexOf(';', sensorData.indexOf(';') + 1) + 1, sensorData.indexOf(';', sensorData.indexOf(';', sensorData.indexOf(';') + 1) + 1)).toInt();
      float voltage = sensorData.substring(sensorData.lastIndexOf(';') + 1).toFloat();

      Serial.print("Parsed PHASE2 data - nodeId: ");
      Serial.print(nodeId);
      Serial.print(", temperature: ");
      Serial.print(temperature);
      Serial.print(", humidity: ");
      Serial.print(humidity);
      Serial.print(", gas: ");
      Serial.print(gas);
      Serial.print(", voltage: ");
      Serial.println(voltage);

      storePhase2Data(nodeId, temperature, humidity, gas, voltage);

      LoRa.beginPacket();
      LoRa.print(String(sinkID) + ";ACK2;" + String(nodeId));
      LoRa.endPacket();
      Serial.println("Pengiriman ACK2 ke Node ID: " + String(nodeId));

      // Update waktu terakhir data diterima
      updateLastHeard(nodeId);
      // Tandai node sebagai aktif
      setActiveStatus(nodeId, true);
    } else {
      Serial.println("Received unknown command.");
    }

    // Check if the node is isolated based on RSSI value
    if (rssi < RSSI_THRESHOLD) {
      Serial.println("Node is considered isolated due to low RSSI.");
      handleIsolatedNode(data); // Panggil fungsi untuk menangani node terisolasi
    }
  } else {
    Serial.println("Ignored data not from Sink.");
  }
}

void setActiveStatus(int nodeId, bool status) {
  for (NodeData &node : nodes) {
    if (node.id == nodeId) {
      node.isActive = status;
      break;
    }
  }
}

void updateLastHeard(int nodeId) {
  for (NodeData &node : nodes) {
    if (node.id == nodeId) {
      node.lastHeard = millis();
      break;
    }
  }
}

void storeInitialData(int id, float posX, float posY, float initialEnergy) {
  Serial.println("Storing initial data");
  for (NodeData &node : nodes) {
    if (node.id == id) {
      node.posX = posX;
      node.posY = posY;
      node.initialEnergy = initialEnergy;
      node.dataAcknowledged = true;
      Serial.println("Stored initial data for Node ID: " + String(id));
      Serial.println("Position: (" + String(posX) + ", " + String(posY) + ") Initial Energy: " + String(initialEnergy));
      return;
    }
  }
  Serial.println("Node ID not found for storing initial data.");
}

void storePhase2Data(int id, float temperature, float humidity, int gas, float voltage) {
  Serial.println("Storing phase 2 data");
  for (NodeData &node : nodes) {
    if (node.id == id) {
      if (!node.dataAcknowledged) {
        Serial.println("Mengabaikan data fase kedua dari node yang belum mengirimkan data fase pertama.");
        return;
      }
      node.temperature = temperature;
      node.humidity = humidity;
      node.gas = gas;
      node.voltage = voltage;
      node.phase2DataAcknowledged = true;
      Serial.println("Stored phase 2 data for Node ID: " + String(id));
      Serial.println("Temperature: " + String(temperature) + " Humidity: " + String(humidity) + " Gas: " + String(gas) + " Voltage: " + String(voltage));

      // Simpan data ke kontainer
      bool nodeExists = false;
      for (auto &existingNode : allData) {
        if (existingNode.id == node.id) {
          existingNode = node;
          nodeExists = true;
          break;
        }
      }
      if (!nodeExists) {
        allData.push_back(node);
      }
      return;
    }
  }
  Serial.println("Node ID not found for storing phase 2 data.");
}

bool allNodesAcknowledged() {
  for (NodeData &node : nodes) {
    if (!node.dataAcknowledged) {
      return false;
    }
  }
  return true;
}

bool allNodesPhase2Acknowledged() {
  for (NodeData &node : nodes) {
    if (!node.phase2DataAcknowledged) {
      return false;
    }
  }
  return true;
}

void resetForNextRound() {
  unsigned long currentMillis = millis();
  for (NodeData &node : nodes) {
    if (currentMillis - node.lastHeard > roundInterval) {
      node.dataAcknowledged = false;
      node.phase2DataAcknowledged = false;
      node.isCH = false; // Reset CH status
      node.cluster = 0; // Reset cluster number
      node.chID = 0; // Reset CH ID
    }
  }
  allNodesAcknowledgedFlag = false;
  isolatedNodes.clear(); // Clear isolated nodes for the next round
  Serial.println("Reset all nodes for the next round.");
}

void printSummary() {
  Serial.println("Rekap Data:");
  Serial.println("Ronde, ID, PosisiX, PosisiY, Energi Awal, Suhu, Kelembaban, Gas, Tegangan, Cluster, CH, CH ID");
  for (const NodeData &data : allData) {
    Serial.print(currentRound);
    Serial.print(", ");
    Serial.print(data.id);
    Serial.print(", ");
    Serial.print(data.posX);
    Serial.print(", ");
    Serial.print(data.posY);
    Serial.print(", ");
    Serial.print(data.initialEnergy);
    Serial.print(", ");
    Serial.print(data.temperature);
    Serial.print(", ");
    Serial.print(data.humidity);
    Serial.print(", ");
    Serial.print(data.gas);
    Serial.print(", ");
    Serial.print(data.voltage);
    Serial.print(", ");
    Serial.print(data.cluster);
    Serial.print(", ");
    Serial.print(data.isCH ? "Yes" : "No");
    Serial.print(", ");
    Serial.println(data.chID);
  }
  Serial.println("Isolated Nodes: ");
  for (const int &id : isolatedNodes) {
    Serial.print("Node ID ");
    Serial.print(id);
    Serial.print(", ");
  }
  Serial.println();
}

//Fase Set-Up
void CHSelection() {
    std::vector<NodeData> uniqueData;
    for (const NodeData &node : allData) {
        auto it = std::find_if(uniqueData.begin(), uniqueData.end(), [&node](const NodeData &n) {
            return n.id == node.id;
        });
        if (it == uniqueData.end()) {
            uniqueData.push_back(node);
        } else {
            *it = node; // Update dengan data terbaru
        }
    }

    int n = uniqueData.size();
    int clusterCount = 0;

    // Seleksi CH maksimal 3 cluster
    for (int i = 0; i < n; i++) {
        if (uniqueData[i].id != sinkID && uniqueData[i].initialEnergy > 2.8) {
            uniqueData[i].isCH = true;
            uniqueData[i].cluster = ++clusterCount;
            uniqueData[i].chID = uniqueData[i].id; // Set CH ID ke ID node itu sendiri
            uniqueData[i].chX = uniqueData[i].posX;
            uniqueData[i].chY = uniqueData[i].posY;
            Serial.println("Node ID " + String(uniqueData[i].id) + " dipilih sebagai CH untuk cluster " + String(uniqueData[i].cluster));
            if (clusterCount >= 3) {
                break;
            }
        }
    }

    // Update allData with the new cluster and CH assignments
    for (const NodeData &uniqueNode : uniqueData) {
        for (NodeData &node : allData) {
            if (node.id == uniqueNode.id) {
                node.isCH = uniqueNode.isCH;
                node.cluster = uniqueNode.cluster;
                node.chID = uniqueNode.chID;
                node.chX = uniqueNode.chX;
                node.chY = uniqueNode.chY;
            }
        }
    }
}

void assignNodesToClusters() {
  std::vector<NodeData> uniqueData;
  for (const NodeData &node : allData) {
      auto it = std::find_if(uniqueData.begin(), uniqueData.end(), [&node](const NodeData &n) {
          return n.id == node.id;
      });
      if (it == uniqueData.end()) {
          uniqueData.push_back(node);
      } else {
          *it = node; // Update dengan data terbaru
      }
  }

  int n = uniqueData.size();
  
  // Penugasan node ke cluster terdekat
  for (int i = 0; i < n; i++) {
    if (!uniqueData[i].isCH && uniqueData[i].initialEnergy > 2.2) {
      float minDist = 90.0;
      int minIndex = -1;
      for (int j = 0; j < n; j++) {
        if (uniqueData[j].isCH) {
          float dist = sqrt(pow(uniqueData[i].posX - uniqueData[j].posX, 2) + pow(uniqueData[i].posY - uniqueData[j].posY, 2));
          if (dist < minDist) {
            minDist = dist;
            minIndex = j;
          }
        }
      }
      if (minIndex != -1) {
        uniqueData[i].cluster = uniqueData[minIndex].cluster;
        uniqueData[i].chID = uniqueData[minIndex].id;
        uniqueData[i].chX = uniqueData[minIndex].posX;
        uniqueData[i].chY = uniqueData[minIndex].posY;
        Serial.println("Node ID " + String(uniqueData[i].id) + " ditugaskan ke cluster " + String(uniqueData[i].cluster) + " dengan CH ID " + String(uniqueData[i].chID));
      }
    }
  }

  // Update allData with the new cluster and CH assignments
  for (const NodeData &uniqueNode : uniqueData) {
      for (NodeData &node : allData) {
          if (node.id == uniqueNode.id) {
              node.isCH = uniqueNode.isCH;
              node.cluster = uniqueNode.cluster;
              node.chID = uniqueNode.chID;
              node.chX = uniqueNode.chX;
              node.chY = uniqueNode.chY;
          }
      }
  }
}

void identifyIsolatedNodes() {
  isolatedNodes.clear();
  for (const NodeData &node : nodes) {
    if (!node.dataAcknowledged && !node.phase2DataAcknowledged) {
      isolatedNodes.push_back(node.id); // Menambahkan node.id yang sebenarnya ke dalam isolatedNodes
    }
  }
}

void reClusterIsolatedNodes() {
  for (const int &id : isolatedNodes) {
    float minDist = 90.0; // Atur jarak minimal ke nilai yang besar
    int minIndex = -1;
    for (NodeData &node : nodes) {
      if (node.id == id && node.initialEnergy > 2.2) {
        for (NodeData &potentialCH : nodes) {
          if (potentialCH.id != id && !potentialCH.isCH && potentialCH.initialEnergy > 2.2) {
            float dist = sqrt(pow(node.posX - potentialCH.posX, 2) + pow(node.posY - potentialCH.posY, 2));
            if (dist < minDist) {
              minDist = dist;
              minIndex = potentialCH.id;
            }
          }
        }
        if (minIndex != -1) {
          // Assign the isolated node to a new cluster with the closest non-CH node
          for (NodeData &potentialCH : nodes) {
            if (potentialCH.id == minIndex) {
              node.cluster = potentialCH.cluster;
              node.chID = potentialCH.id;
              node.chX = potentialCH.posX;
              node.chY = potentialCH.posY;
              potentialCH.isCH = true; // Jadikan node ini sebagai CH untuk cluster baru
              potentialCH.cluster = node.cluster;
              potentialCH.chID = potentialCH.id;
              Serial.println("Node ID " + String(node.id) + " direcluster ke cluster baru dengan CH ID " + String(potentialCH.id));
            }
          }
        }
      }
    }
  }
}

void handleIsolatedNode(String data) {
  int nodeId = data.substring(0, data.indexOf(';')).toInt();
  
  // Validasi ID node
  if (nodeId == 0) {
    Serial.println("Invalid Node ID: 0, ignoring isolated node.");
    return;
  }
  
  isolatedNodes.push_back(nodeId);
  Serial.println("Node ID " + String(nodeId) + " added to isolated nodes list.");
}
