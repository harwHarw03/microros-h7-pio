
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>

#define W5500_CS_PIN PB12
#define ETHERNET_STATIC

#ifdef ETHERNET_STATIC
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,1,120);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
#else
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
#endif

SPIClass SPI_2(PB15,PB14,PB13);

EthernetServer server(80);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI_2.begin();

  Ethernet.init(W5500_CS_PIN);
#ifdef ETHERNET_STATIC
  Ethernet.begin(mac, ip, gateway, gateway, subnet);
#else
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    for (;;);
  }
#endif

  delay(1000);

  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());

  server.begin();
}

void loop() {
  EthernetClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    client.println("Hello from STM32H743 DevBox W5500!");
    delay(1);
    client.stop();
    Serial.println("Client disconnected");
  }
}