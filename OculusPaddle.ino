/*
  Oculus Paddle Net Custom Controller
  
  Developed by Anthony Russell, 2025

*/

#include <Arduino.h>
#include "ESPTelnet.h"
#include <WiFiClient.h>
#include <WiFiUdp.h>

// WiFi credentials
#define WIFI_SSID     "Same AP that your Quest is Connected to"
#define WIFI_PASSWORD "Wifi password here"

// UDP and Telnet
WiFiUDP udp;
ESPTelnet telnet;
IPAddress ip;

// Pins
const int analogInPin     = A0;
const int redButtonPin    = D1;
const int greenButtonPin  = D2;
const int whiteButtonPin  = D7;

// Telnet settings
const uint16_t port = 23;
bool telnetConnected = false;
int wifiTimeout = 30;  // 15 seconds

// Sensor and button states
int sensorValue  = 0;
int outputValue  = 0;
int redButton    = 0;
int greenButton  = 0;
int whiteButton  = 0;

// IP Broadcast class
class IPBroadcast {
  private:
    char hostName[16];
    char ipAddress[16];
    char udpPacket[40];
    unsigned long previousMillis = 0;
    const unsigned long interval = 1000;

  public:
    void setup(const char* _hostName, const char* _ipAddress) {
      strncpy(hostName, _hostName, sizeof(hostName));
      strncpy(ipAddress, _ipAddress, sizeof(ipAddress));

      snprintf(udpPacket, sizeof(udpPacket), "ipB-|%s|%s", hostName, ipAddress);
    }

    void update() {
      if (millis() - previousMillis >= interval) {
        Serial.println(udpPacket);
        udp.beginPacket("255.255.255.255", 9999);
        udp.write(udpPacket);
        udp.endPacket();
        previousMillis = millis();
      }
    }
};

IPBroadcast ipBroadcast;

// --- Telnet Event Handlers ---
void onTelnetConnect(String clientIp) {
  Serial.printf("- Telnet: %s connected\n", clientIp.c_str());
  telnet.printf("\nWelcome to Oculus Paddle Net: %s\n", telnet.getIP().c_str());
  telnet.println("(Use ^] + q to disconnect.)");
  telnetConnected = true;
}

void onTelnetDisconnect(String clientIp) {
  Serial.printf("- Telnet: %s disconnected\n", clientIp.c_str());
  telnetConnected = false;
}

void onTelnetReconnect(String clientIp) {
  Serial.printf("- Telnet: %s reconnected\n", clientIp.c_str());
  telnetConnected = true;
}

void onTelnetConnectionAttempt(String clientIp) {
  Serial.printf("- Telnet: %s connection attempt\n", clientIp.c_str());
}

void onTelnetInput(String input) {
  if (input == "ping") {
    telnet.println("> pong");
  } else if (input == "bye") {
    telnet.println("> disconnecting you...");
    telnet.disconnectClient();
  } else {
    telnet.println(input);
  }
}

void setupTelnet() {
  telnet.onConnect(onTelnetConnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onInputReceived(onTelnetInput);

  if (!telnet.begin(port)) {
    Serial.println("Telnet failed to start. Restarting...");
    delay(2000);
    ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nWelcome to Oculus Paddle Net");

  pinMode(redButtonPin, INPUT_PULLUP);
  pinMode(greenButtonPin, INPUT_PULLUP);
  pinMode(whiteButtonPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // WiFi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED && wifiTimeout > 0) {
    Serial.print(".");
    delay(500);
    wifiTimeout--;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi!");
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.printf("\nWiFi Connected. IP: %s\n", WiFi.localIP().toString().c_str());

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  ip = WiFi.localIP();
  setupTelnet();

  // Start IP Broadcast
  Serial.println("Starting IP Broadcast.");
  ipBroadcast.setup("OculusPaddle", ip.toString().c_str());
}

void loop() {
  telnet.loop();

  sensorValue = analogRead(analogInPin);
  redButton   = digitalRead(redButtonPin);
  greenButton = digitalRead(greenButtonPin);
  whiteButton = digitalRead(whiteButtonPin);

  outputValue = map(sensorValue, 0, 1023, 0, 255);

  char telnetPacket[20];
  snprintf(telnetPacket, sizeof(telnetPacket), "{%d,%d,%d,%d}", outputValue, redButton, greenButton, whiteButton);

  telnet.println(telnetPacket);

  if (!telnetConnected) {
    ipBroadcast.update();
  }

  delay(20);  // 50Hz
}
