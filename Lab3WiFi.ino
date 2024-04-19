#include <WiFiEsp.h>
#include <WiFiEspUdp.h>

#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(19, 18); // RX, TX (USE RX1/TX1)
#endif

char ssid[] = "SSID";      // your network SSID (name)
char pass[] = "PASSWORD";  // your network password
int status = WL_IDLE_STATUS;    // the WiFi radio's status

const unsigned int localPort = 10002;   // local port to listen on
const size_t PACKET_BUFFER_SIZE = 256;  // define the packet buffer size

char packetBuffer[PACKET_BUFFER_SIZE];   // buffer to hold incoming packet
char ReplyBuffer[] = "ACK";             // a string to send back your reply

WiFiEspUDP Udp;

void setup() {
  Serial.begin(115200);  // initialize serial for debugging
  Serial1.begin(115200); // initialize serial for ESP module
  WiFi.init(&Serial1);   // initialize ESP module

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // Retry connecting to WiFi shield or take alternative action
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000); // Wait 10 seconds before retrying
  }

  Serial.println("Connected to WiFi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);

  Serial.print("Listening on port ");
  Serial.println(localPort);
}

void loop() {
  //Send a Message
  Udp.beginPacket("192.168.1.255", localPort);
  Udp.write(ReplyBuffer);
  Udp.endPacket();

  delay(1000); // Introduce a small delay for stability

  //Receive a message
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    if (packetSize <= PACKET_BUFFER_SIZE - 1) {
      IPAddress remoteIp = Udp.remoteIP();
      int remotePort = Udp.remotePort();

      int len = Udp.read(packetBuffer, packetSize);
      packetBuffer[len] = 0; // Null-terminate the received data
      Serial.print("Received packet from ");
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(remotePort);
      Serial.println("Contents:");
      Serial.println(packetBuffer);
    } else {
      // Packet too big for buffer
      Serial.println("Packet exceeds buffer size, message truncated");
      while (Udp.available()) Udp.read(); // Clear the buffer
    }
  }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("Signal Strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
}
