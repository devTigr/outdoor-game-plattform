//Deploy on TTGO LoRa32-OLED to be used for "BaseNode"

//LoRa
#include <SPI.h>
#include <LoRa.h>
//OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//WLAN
#include <WiFi.h>
#include "secrets.h"
//MQTT
#include <PubSubClient.h>

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
byte localAddress = 0xBB;  // address of this device
byte destination = 0xFF;   // destination to send to
String deviceName = "Base";
long lastSendTime = 0;  // last send time
int interval = 2000;    // interval between sends
volatile bool newMessageReceived = false;
String receivedMessage = "";
int sender, recipient, incomingMsgId, incomingLength;
int rssi;
float snr;

//OLED variables
#define I2C_SDA 21
#define I2C_SCL 22
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//WiFi variables
WiFiClient espClient;
PubSubClient client(espClient);
long currentTime, lastTime;
int MQTTcount = 0;
char messages[200];

//MQTT variables
const char* DeviceName = "Base01";
const char* brokerUser = "";
const char* brokerPW = "";
const char* broker = "OGP.local";
const int port = 1883;
const char* outTopic = "gadgetMessage";
const char* inTopic = "inGameMessage";
const char* healthCheck = "healthCheck";

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // warten au Serial monitor
  Serial.println("Starting Setup...");

  // WLAN setup
  setupWiFi();
  client.setServer(broker, port);
  client.setCallback(callback);

  // LORA setup
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

  // OLED setup
  setupOLED();
  Serial.println("OLED started");

  Serial.println("Setup done!");
}

void loop() {
  //Reconect to MQTT if not allready
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //health check
  // if (millis() - lastSendTime > interval) {
  //   String time = String(millis()/1000) ;
  //   String message = "Base alive: " ;
  //   sendMQTTmessage(message);
  //   sendLoRaMessages(message + time);
  //   lastSendTime = millis();      // timestamp the message
  //   interval = random(20000);     // about 5 Min
  // }

  //Process received messages
  if (newMessageReceived) {
    processReceivedMessage();
    newMessageReceived = false;
  }
  LoRa.receive();
  delay(1000);
}

void sendLoRaMessages(String outgoing) {
  LoRa.beginPacket();             // start packet
  LoRa.write(destination);        // add destination address
  LoRa.write(localAddress);       // add sender address
  LoRa.write(msgCount);           // add message ID
  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);           // add payload
  LoRa.endPacket();               // finish packet and send it
  msgCount++;

  delay(200);

  LoRa.receive();  // Ensure LoRa is set back to receive mode after sending
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;  // if there's no packet, return
  
  //read packet header bytes:
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
  if (recipient != localAddress /*&& recipient != 0xFF*/) {
    Serial.print("This message is not for me.");
    Serial.println(" Was for: " + String(recipient, HEX));
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

void processReceivedMessage() {
  // Process the received message
  char recMessage[200];
  Serial.print("Processing received message: ");
  Serial.println(receivedMessage);
  snprintf(recMessage, sizeof(recMessage),
           "0x%X,0x%X,%d,%d,%s,%d,%.2f",
           sender, recipient, incomingMsgId, incomingLength, receivedMessage.c_str(), rssi, snr);
  Serial.println("MQTT send " + String(outTopic) + ": " + String(recMessage));
  client.publish(outTopic, recMessage);
  LoRa.receive();
}

void sendMQTTmessage(String message) {
  MQTTcount++;
  char mqttMessage[200];
  snprintf(mqttMessage, sizeof(mqttMessage),
           "0x%X,0x%X,%d,%d,%s,%d,%d",
           localAddress, recipient, localAddress, 0, message, 0, 0);
  Serial.print("Sending MQTT-Messages: ");
  Serial.println(mqttMessage);
  client.publish(healthCheck, mqttMessage);
  LoRa.receive();
}

void setupWiFi() {
  delay(100);
  Serial.print("SetupWiFI Connection to: ");
  Serial.println(ssid);
  WiFi.begin(ssid, WiFiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.print("\nConnected to: ");
  Serial.println(ssid);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("\nConnecting to ");
    Serial.println(broker);
    if (client.connect(DeviceName, brokerUser, brokerPW)) {
      Serial.print("\nConnected to: ");
      Serial.println(broker);
      client.subscribe(inTopic);
    } else {
      Serial.print("Please try connecting to MQTT again!");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received messages: ");
  Serial.println(topic);
  String msg = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg += (char)payload[i];
  }
  Serial.println();
  sendLoRaMessages(msg);
  Serial.println("Sending " + msg);
  LoRa.receive();
}

void setupOLED() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  drawMain();
}
void drawMain() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // display.setCursor(2, 0);
  // display.println(F("Connect to:"));
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
}