#include <Wire.h>
#include <RTClib.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

RTC_DS1307 rtc;

const char *ssid = "Lucas";
const char *password = "159753lucas";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "a.st1.ntp.br", -3600*3, 60000);  // Use the NTP server provided by RNP

void setup() {
  Serial.begin(115200);

  connectToWiFi();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (!rtc.isrunning()) {
    Serial.println("RTC is not running! Calibrating with NTP.");
  } else {
    Serial.println("RTC is already running. Calibrating with NTP.");
    calibrateRTC();
  }
}

void loop() {
  // Your main loop code, if needed
  DateTime now = rtc.now();

  // Create file name with the format WS1_ddmmyyyy.csv
  char fileName[20];
  snprintf(fileName, sizeof(fileName), "WS1_%02d%02d%04d.csv", now.day(), now.month(), now.year());

  // Get time in the format hh:mm:ss
  char timeBuffer[20];
  snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  Serial.println(fileName);
  Serial.println(timeBuffer);
  delay(5000);
}

void calibrateRTC() {
  Serial.println("Calibrating RTC with NTP...");

  timeClient.begin();

  while (!timeClient.update()) {
    timeClient.forceUpdate();
    delay(100);
  }

  DateTime now = timeClient.getEpochTime();

  rtc.adjust(now);

  Serial.println("RTC calibrated with NTP.");
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());
}
