//Deploy on TTGO LoRa32-OLED to be used for "MobileNode"

//Connection via LoRa
#include <SPI.h>
#include <LoRa.h>
//Connection via BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//use OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//LoRa variables
#define LORA_SCK 5         // GPIO5 - SX1276 SCK
#define LORA_MISO 19       // GPIO19 - SX1276 MISO
#define LORA_MOSI 27       // GPIO27 - SX1276 MOSI
#define LORA_CS 18         // GPIO18 - SX1276 CS
#define LORA_RST 14        // GPIO14 - SX1276 RST
#define LORA_IRQ 26        // GPIO26 - SX1276 IRQ (interrupt request)
#define RANGE 868E6        // Frequency
String outgoing;           // outgoing message
byte msgCount = 0;         // count of outgoing messages
byte localAddress = 0xAA;  // address of this device
byte destination = 0xBB;   // destination to send to // only to base
long lastSendTime = 0;     // last send time
int interval = 2000;       // interval between sends

volatile bool newMessageReceived = false;
String receivedMessage = "";
int sender, recipient, incomingMsgId, incomingLength;
int rssi;
float snr;

//BLE variables
String deviceName = "001_NODE";
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic_1 = NULL;  // LoRa-Message
int LoRaCharaNum = 1;
BLECharacteristic* pCharacteristic_2 = NULL;  // GeoLoc
int GeoCharaNum = 2;
bool pCharacteristic_2Changed = false;
BLEDescriptor* pDescr_1;  // LoRa-Message
BLEDescriptor* pDescr_2;  // GeoLoc
BLE2902* pBLE2902;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;  // wozu??
//?int LED = 25;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define IN_GAME_NACHRICHT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  //In Game Nachricht
#define GPS_CHARACTERISTIC_UUID "3d453f10-c88d-4ab5-8133-e7020f311a1e"                //GPS Nachricht

//OLED variables
#define I2C_SDA 21
#define I2C_SCL 22
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Game variablen
String beispielLocation = "47.353450,7.903995";  //TEKO 47.353450, 7.903995

// classes
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};
class Characteristic2Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic_2) override {
    outgoing = pCharacteristic_2->getValue().c_str();
    pCharacteristic_2Changed = true;
  }
};


void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // warten auf Serial monitor
  Serial.println("Starting Setup with LED's...");

  setupOLED();
  Serial.println("OLED started");

  setupBLE();
  Serial.println("BLE connected");

  Serial.println("Starting Radio...");
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);  // set CS, reset, IRQ pin
  if (!LoRa.begin(RANGE)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  // Interrupts and receives
  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("LoRa init succeeded.");

  // Initiate all connevtions:
  String message = String(deviceName) + " started: " + String(millis());
  sendLoRaMessage(message);
  processBLEmessage(message);
  pCharacteristic_2->setValue(beispielLocation);
  Serial.println("Sending: " + message);

  Serial.println("Setup done!");
}

void loop() {
  //health check Debugging
  // if (millis() - lastSendTime > interval) {
  //   lastSendTime = millis();                            // timestamp the message
  //   String message = "alive: " + String(lastSendTime);  // send a message
  //   sendLoRaMessage(message);
  //   processBLEmessage(LoRaCharaNum, message);
  //   processBLEmessage(GeoCharaNum, String(beispielLocation));
  //   Serial.println("Sending " + message);
  //   interval = random(20000);  // 20 seconds
  // }

  if (pCharacteristic_2Changed) {
    sendLoRaMessage(outgoing);  // Send the changed value over LoRa
    Serial.println("Characteristic 2 changed, message sent over LoRa: " + outgoing);
    pCharacteristic_2Changed = false;
  }

  // Process the received message
  if (newMessageReceived) {
    processReceivedLoRaMessage();
    newMessageReceived = false;
  }
  LoRa.receive();
  delay(2000);
}

void sendLoRaMessage(String outgoing) {
  LoRa.beginPacket();             // start packet
  LoRa.write(destination);        // add destination address
  LoRa.write(localAddress);       // add sender address
  LoRa.write(msgCount);           // add message ID
  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);           // add payload
  LoRa.endPacket();               // finish packet and send it
  msgCount++;

  LoRa.receive();  // Ensure LoRa is set back to receive mode after sending
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;  // if there's no packet, return

  // read packet header bytes:
  recipient = LoRa.read();       // recipient address
  sender = LoRa.read();          // sender address
  incomingMsgId = LoRa.read();   // incoming msg ID
  incomingLength = LoRa.read();  // incoming msg length

  String incoming = "";  // payload of packet

  while (LoRa.available()) {        // can't use readString() in callback, so
    incoming += (char)LoRa.read();  // add bytes one by one
  }

  if (incomingLength != incoming.length()) {  // check length for error
    Serial.println("error: message length does not match length");
    return;  // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;  // skip rest of function
  }

  // Store the received message and set the flag
  receivedMessage = incoming;
  rssi = LoRa.packetRssi();
  snr = LoRa.packetSnr();
  newMessageReceived = true;

  // Ensure LoRa is set back to receive mode after processing the message
  LoRa.receive();
}

void processReceivedLoRaMessage() {
  char recMessage[200];
  Serial.println("Processing received message:");
  Serial.println(receivedMessage);
  snprintf(recMessage, sizeof(recMessage),
           "0x%X,0x%X,%d,%d,%s,%d,%.2f",
           sender, recipient, incomingMsgId, incomingLength, receivedMessage.c_str(), rssi, snr);
  Serial.print("Got Message: ");
  Serial.println(recMessage);

  // Send the received message to the phone via Bluetooth
  processBLEmessage(String(receivedMessage.c_str()));

  drawScrolltext(receivedMessage.c_str());
}

void processBLEmessage(String bleMessage) {

  if (deviceConnected) {
    //drawScreen("Connected");
    pCharacteristic_1->setValue(bleMessage);
    pCharacteristic_1->notify();
    value++;
    //delay(2000);
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    //drawScreen("Disconnected");
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.print("start advertising...");
    String time = String(millis());
    Serial.println(time);
    oldDeviceConnected = deviceConnected;
    //drawScreen("Reconnected");
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    sendLoRaMessage("Device (re)connected: " + deviceName);
    oldDeviceConnected = deviceConnected;
  }
}

void characteristic_2Changed() {
  String message = pCharacteristic_2->getValue();
  sendLoRaMessage(message);
}

void setupBLE() {
  // Create the BLE Device
  BLEDevice::init(deviceName);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create the BLE Characteristics for incoming LoRa messages outgoing via BLE to smartphone
  pCharacteristic_1 = pService->createCharacteristic(
    IN_GAME_NACHRICHT_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);
  // Create the BLE Characteristics for incomming GPS data from Smartphone outgoing via LoRa to Base
  pCharacteristic_2 = pService->createCharacteristic(
    GPS_CHARACTERISTIC_UUID,
    (BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE));  // Not necessary I think?

  // callback for changed values (send LoRa)
  pCharacteristic_2->setCallbacks(new Characteristic2Callback());

  // Create a BLE Descriptor for LoRa Messages
  pDescr_1 = new BLEDescriptor((uint16_t)0x2901);
  pDescr_1->setValue("LoRa-Message");
  pCharacteristic_1
    ->addDescriptor(pDescr_1);
  // Create a BLE Descriptor for Geo Messages
  pDescr_2 = new BLEDescriptor((uint16_t)0x2901);
  pDescr_2->setValue("GEO-Message");
  pCharacteristic_2
    ->addDescriptor(pDescr_2);

  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic_1
    ->addDescriptor(pBLE2902);
  // Start the service
  pService->start();
  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  drawMain();
  Serial.println("Waiting a client connection to notify...");
  //infinite loop until device is connected
  while (!deviceConnected) {
    lastSendTime = millis();  // timestamp the message
    String message = "alive: " + String(lastSendTime);
    processBLEmessage("BLE starting");
    delay(200);
  }
  drawScreen("Connected");
}

void setupOLED() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  drawScrolltext("OGP!");
  delay(1000);
}

void drawMain() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, 0);
  display.println(F("Connect to:"));
  display.setTextSize(2);
  display.setCursor(2, 12);
  display.println(String(deviceName));
  display.display();
}

void drawScrolltext(String message) {
  display.clearDisplay();
  display.setTextSize(2);  // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 5);
  display.println(message);
  display.display();
  delay(100);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
}

void drawScreen(String message) {
  display.clearDisplay();
  display.setTextSize(2);  // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 5);
  display.println(message);
  display.display();
  delay(100);
}