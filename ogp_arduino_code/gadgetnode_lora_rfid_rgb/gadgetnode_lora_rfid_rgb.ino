
//Deploy on TTGO LoRa32-OLED to be used for "GadgetNode"
#include <Arduino.h>
//LoRa
#include <SPI.h>
#include <LoRa.h>
//RFID-Reader
#include <MFRC522.h>
//OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//LED
#include <FastLED.h>

// SPI Setup (Da es zwei verschiedene SPI's braucht da nicht alle PINs vorhanden)
//clock miso mosi ss
#define LORA_MISO 19  // #define LORA_MISO 19       // GPIO19 - SX1276 MISO (SDO)
#define LORA_MOSI 27  // #define LORA_MOSI 27       // GPIO27 - SX1276 MOSI (SDI or DIN)
#define LORA_SCK 5    // #define LORA_SCK 5         // GPIO5 - SX1276 SCK
#define LORA_SS 18    // #define LORA_CS 18         // GPIO18 - SX1276 CS (SS)

#define RFID_MISO 12
#define RFID_MOSI 13
#define RFID_SCLK 14
#define RFID_SS 15
int currentSPI = -1;  // -1 = not started, 0 = RFID, 1 = LORA

//LoRa variables
#define LORA_RST 14        // GPIO14 - SX1276 RST
#define LORA_IRQ 26        // GPIO26 - SX1276 IRQ (interrupt request)
#define RANGE 868E6        // Frequency
String outgoing;           // outgoing message
byte msgCount = 0;         // count of outgoing messages
byte localAddress = 0xCC;  // address of this device
byte destination = 0xBB;   // destination to send to
String deviceName = "Gadget01";
long lastSendTime = 0;  // last send time
int interval = 2000;    // interval between sends
volatile bool newMessageReceived = false;
volatile bool loraInterruptTriggered = false;
String receivedMessage = "";
int sender, recipient, incomingMsgId, incomingLength;
int rssi;
float snr;
void loraReceiveInterrupt();  // Forward declaration for loraReceiveInterrupt

//RFID variables
#define RST_PIN 23
MFRC522 rfid(RFID_SS, RST_PIN);  // Create MFRC522 instance
// Init array that will store a new NUID
byte nuidPICC[4];
// All known ID's:
byte knownUIDred[4] = { 0xC3, 0xC2, 0x0A, 0x97 };
byte knownUIDyellow[4] = { 0x13, 0x69, 0xCC, 0x15 };
byte knownUIDgreen[4] = { 0x04, 0xFE, 0xEA, 0xEA };

//OLED variables
#define I2C_SDA 21
#define I2C_SCL 22
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//LED variables
#define LED_PIN 4
#define LED_BUILTIN 25
#define NUM_LEDS 3
CRGB g_LEDs[NUM_LEDS] = { 0 };  //Buffer for FastLED
int brightness = 250;           //max 255
int powerLimit = 900;           //max 900mW Powerdraw

// Game variables
String currentColor = "";
bool flagChange = false;


void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // warten auf Serial monitor
  Serial.println("Starting Setup with LED's...");

  // LED setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<SK6812, LED_PIN, GRB>(g_LEDs, NUM_LEDS);
  FastLED.setBrightness(brightness);
  setLEDColor("red");
  delay(500);
  setLEDColor("yellow");
  delay(500);
  setLEDColor("green");
  delay(500);
  setLEDColor("");
  set_max_power_indicator_LED(LED_BUILTIN);
  FastLED.setMaxPowerInMilliWatts(powerLimit);
  setLEDColor("");

  // OLED setup
  setupOLED();
  Serial.println("OLED started");

  // LORA setup
  Serial.print("Starting Radio...");
  spi_channel(1);
  if (!LoRa.begin(RANGE)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;  //Stall the Program
  }
  LoRa.receive();
  attachInterrupt(digitalPinToInterrupt(LORA_IRQ), loraReceiveInterrupt, FALLING);  // Attach interrupt to LORA_IRQ pin
  Serial.println("LoRa init succeeded.");

  // RFID setup
  Serial.print("Starting RFID...");
  spi_channel(0);
  ShowReaderDetails();
  Serial.println("RFID started successfully!");
  spi_channel(1);

  Serial.println("Setup done!");
}

void loop() {
  // LoRa health check and send message periodically
  // if (millis() - lastSendTime > interval) {
  //   String time = String(millis());
  //   String message = currentColor + time;
  //   sendMessage(message);
  //   Serial.println("Sending " + message);
  //   lastSendTime = millis();   // timestamp the message
  //   interval = random(20000);  // about 5 min
  // }
  LoRa.receive();
  delay(10000);

  // Check if the LoRa interrupt has triggered
  if (loraInterruptTriggered) {
    loraInterruptTriggered = false;  // Reset the interrupt flag
    spi_channel(1);                  // Ensure we're on the LoRa SPI channel
    onReceive(LoRa.parsePacket());   // Process the received packet
  }

  // Process any new received message
  if (newMessageReceived) {
    processReceivedMessage();
    newMessageReceived = false;
  }

  readRFIDCard();

  if (flagChange) {
    setLEDColor(currentColor);
    sendMessage(currentColor);
    currentColor = "";
    flagChange = false;
  }

  spi_channel(1);
  LoRa.receive();
}

// Interrupt handler for LoRa receive
void IRAM_ATTR loraReceiveInterrupt() {
  loraInterruptTriggered = true;  // Set flag to handle in main loop
}

void spi_channel(int channel) {
  if (channel == currentSPI) return;
  SPI.end();
  switch (channel) {
    case 0:
      SPI.begin(RFID_SCLK, RFID_MISO, RFID_MOSI);
      MFRC522::MIFARE_Key key;
      rfid.PCD_Init();
      Serial.println("Switched to SPI-Channel: 0");
      break;
    case 1:
      SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
      LoRa.setPins(LORA_SS, LORA_RST, LORA_IRQ);
      if (!LoRa.begin(RANGE)) {
        Serial.println("Starting LoRa failed!");
        while (1)
          ;
      }
      Serial.println("Switched to SPI-Channel: 1");
      LoRa.receive();
      break;
  }
  currentSPI = channel;
}

// RFID ID to Serial HEX
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}
// RFID ID to Serial DEC
void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(' ');
    Serial.print(buffer[i], DEC);
  }
}
// Infos about the Reader (Debugging only)
void ShowReaderDetails() {
  spi_channel(0);
  // Get the mrfc522 software version
  byte v = rfid.PCD_ReadRegister(rfid.VersionReg);
  Serial.print(F("mrfc522 Software Version: 0x"));
  Serial.print(v, HEX);
  if (v == 0x91)
    Serial.print(F(" = v1.0"));
  else if (v == 0x92)
    Serial.print(F(" = v2.0"));
  else
    Serial.print(F(" (unknown)"));
  Serial.println("");
  // When 0x00 or 0xFF is returned, communication probably failed
  if ((v == 0x00) || (v == 0xFF)) {
    Serial.println(F("WARNING: Communication failure, is the mrfc522 properly connected?"));
  }
  //spi_channel(1);
}
void readRFIDCard() {
  spi_channel(0);
  delay(200);
  if (!rfid.PICC_IsNewCardPresent())
    return;

  // Verify if the NUID has been readed
  if (!rfid.PICC_ReadCardSerial())
    return;

  // Serial.print(F("PICC type: "));
  // MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  // Serial.println(rfid.PICC_GetTypeName(piccType));

  if (rfid.uid.uidByte[0] != nuidPICC[0] || rfid.uid.uidByte[1] != nuidPICC[1] || rfid.uid.uidByte[2] != nuidPICC[2] || rfid.uid.uidByte[3] != nuidPICC[3]) {
    Serial.println(F("A new card has been detected."));

    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
      //Serial.println("nuidPICC: ");
      //Serial.println(nuidPICC[1]);
    }
    // check color by ID:
    if (nuidPICC[1] == knownUIDred[1]) {
      // Serial.println("red");
      // Serial.println(nuidPICC[1]);
      // Serial.println(knownUIDred[1]);
      currentColor = "red";
    } else if (nuidPICC[1] == knownUIDyellow[1]) {
      // Serial.println("yellow");
      // Serial.println(nuidPICC[1]);
      // Serial.println(knownUIDyellow[1]);
      currentColor = "yellow";
    } else if (nuidPICC[1] == knownUIDgreen[1]) {
      // Serial.println("green");
      // Serial.println(nuidPICC[1]);
      // Serial.println(knownUIDgreen[1]);
      currentColor = "green";
    }
    flagChange = true;
    // Serial.println(F("The NUID tag is:"));
    // Serial.print(F("In hex: "));
    // printHex(rfid.uid.uidByte, rfid.uid.size);
    // Serial.println();
    // Serial.print(F("In dec: "));
    // printDec(rfid.uid.uidByte, rfid.uid.size);
    // Serial.println();
  } else Serial.println(F("Card read previously."));

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
  spi_channel(1);
}

void sendMessage(String outgoing) {
  spi_channel(1);
  delay(200);
  for (int i = 0; i < 2; i++) {
    LoRa.beginPacket();             // start packet
    LoRa.write(destination);        // add destination address
    LoRa.write(localAddress);       // add sender address
    LoRa.write(msgCount);           // add message ID
    LoRa.write(outgoing.length());  // add payload length
    LoRa.print(outgoing);           // add payload
    LoRa.endPacket();
    int randTime = random(1000);
    delay(1000 + randTime);  // finish packet and send it
  }
  msgCount++;

  LoRa.receive();  // Ensure LoRa is set back to receive mode after sending
}
void onReceive(int packetSize) {
  spi_channel(1);
  delay(200);
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
void processReceivedMessage() {
  // Process the received message
  char recMessage[200];
  Serial.println("Processing received message:");
  Serial.println(receivedMessage);
  snprintf(recMessage, sizeof(recMessage),
           "0x%X,0x%X,%d,%d,%s,%d,%.2f",
           sender, recipient, incomingMsgId, incomingLength, receivedMessage.c_str(), rssi, snr);
  Serial.print("Got Message: ");
  Serial.println(recMessage);
  drawScrolltext(receivedMessage.c_str());
}

void setupOLED() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  drawMain();
  delay(1000);
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

void setLEDColor(String color) {
  CRGB colorName = CRGB::Black;
  if (color.equals("red")) {
    colorName = CRGB::Red;
  } else if (color.equals("yellow")) {
    colorName = CRGB::Gold;
  } else if (color.equals("green")) {
    colorName = CRGB::Green;
  }
  g_LEDs[0] = colorName;
  // for (int i = 1; i <= NUM_LEDS; i++) {
  //   g_LEDs[i] = colorName;
  // } // wenn genug power
  FastLED.show();
  //delay(2000);
}