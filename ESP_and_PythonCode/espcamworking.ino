#include <WiFi.h>
#include <WebServer.h>

#define ENA 23
#define IN1 22
#define IN2 21
#define ENB 5
#define IN3 19
#define IN4 18

const char* ssid = "NIKHIL";
const char* password = "12345678";

// Static IP (change to your network range)
IPAddress local_IP(192, 168, 137, 79);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8,8,8,8);
IPAddress secondaryDNS(8,8,4,4);

WebServer server(80);

int currentSpeed = 0;

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Default direction: Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Setup WiFi with static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("⚠️ Failed to set static IP");
  }
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ WiFi Connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Web route for /set_speed
  server.on("/set_speed", HTTP_GET, []() {
    if (server.hasArg("spd")) {
      int speed = server.arg("spd").toInt();
      currentSpeed = constrain(speed, 0, 120);

      int pwmValue = map(currentSpeed, 0, 120, 0, 255);
      analogWrite(ENA, pwmValue);
      analogWrite(ENB, pwmValue);

      Serial.print("🚗 Speed set to: ");
      Serial.print(currentSpeed);
      Serial.print(" km/h → PWM: ");
      Serial.println(pwmValue);
      
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Missing spd parameter");
    }
  });

  server.begin();
  Serial.println("🌐 Server started: use /set_speed?spd=60");
}

void loop() {
  server.handleClient();
}
