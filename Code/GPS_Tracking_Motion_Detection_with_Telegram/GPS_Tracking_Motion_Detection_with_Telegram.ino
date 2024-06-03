/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/telegram-esp8266-nodemcu-motion-detection-arduino/
  
  Project created using Brian Lough's Universal Telegram Bot Library: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
*/

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TinyMPU6050.h>

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(12, 14);

String lat;
String lng;
bool warningSent = false;
bool distancesent = false;

MPU6050 mpu(Wire);

// Replace with your network credentials
const char* ssid = "xxxxx";
const char* password = "xxxxx";

// Initialize Telegram BOT
#define BOTtoken "xxxxxxxx"  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
#define CHAT_ID "7046851423"

X509List cert(TELEGRAM_CERTIFICATE_ROOT);
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);


const int buzzerPin = 2;  // Pin buzzer
int const trigPin = 15;
int const echoPin = 13;
const unsigned long BOT_MTBS = 1000;  // mean time between scan messages

unsigned long bot_lasttime;  // last time messages' scan has been done

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 10000;

void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i = 0; i < numNewMessages; i++) {
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;

    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "Use the following commands to control your outputs.\n\n";
      welcome += "/get_gps acquire Google Maps link for current Location \n";
      welcome += "/angle acquire Angle of the Vehicle \n";
      bot.sendMessage(chat_id, welcome, "");
    }

    if (text == "/get_gps") {
      String url = "Your vehicle location is www.google.com/maps/place/" + String(lat) + "," + String(lng);
      bot.sendMessage(chat_id, url, "");
    }

    if (text == "/angle") {
      String angle = "Angle X :";
      angle += mpu.GetAngX();
      angle += "|| Y : ";
      angle += mpu.GetAngY();
      angle += "|| Z : ";
      angle += mpu.GetAngZ();
      bot.sendMessage(chat_id, angle, "");
    }
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  mpu.Initialize();
  // Start the software serial port at the GPS's default baud
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");
  Serial.println("Offsets:");
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());

  configTime(0, 0, "pool.ntp.org");  // get UTC time via NTP
  client.setTrustAnchors(&cert);     // Add root certificate for api.telegram.org

  pinMode(buzzerPin, OUTPUT);  // Set the buzzer pin as an output
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attempt to connect to Wifi network:
  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  bot.sendMessage(CHAT_ID, "Bot started up", "");
}

void loop() {
  int duration, distance;
  digitalWrite(trigPin, HIGH);
  delay(1);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

  currentMillis = millis();
  if (millis() - bot_lasttime > BOT_MTBS) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    bot_lasttime = millis();
  }
  mpu.Execute();
  Serial.print("AngX = ");
  Serial.print(mpu.GetAngX());
  Serial.print("  /  AngY = ");
  Serial.print(mpu.GetAngY());
  Serial.print("  /  AngZ = ");
  Serial.print(mpu.GetAngZ());
  Serial.print("  /  Dis= ");
  Serial.println(distance);

  if (distance < 45 && distance > 2) {
    if (!distancesent) {
      String text = "SOMEONE IN YOUR VEHICLE!!!";
      bot.sendMessage(CHAT_ID, text, "");
      Serial.println("Near Person Detected");
    }
    distancesent = true;
  } else {
    distancesent = false;
  }
  if (mpu.GetAngY() > 10 || mpu.GetAngY() < -10) {
    digitalWrite(buzzerPin, HIGH);
    if (!warningSent) {
      String warning = "MOVEMENT ON YOUR VEHICLE!!!";
      bot.sendMessage(CHAT_ID, warning, "");
      Serial.println("WARNING SENT");
    }
    warningSent = true;
  } else {
    warningSent = false;
    digitalWrite(buzzerPin, LOW);
  }
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        lat = String(gps.location.lat(), 6);
        lng = String(gps.location.lng(), 6);
      } else {
        Serial.println("Location: Not Available");
      }
    }
  }
}
