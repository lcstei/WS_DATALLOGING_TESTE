// Station config
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

#include "esp_sleep.h"

// RTC - Real Time Clock Config
#include <RTClib.h>
RTC_DS1307 rtc;

// SD Card config
#include <SPI.h>
#include <SdFat.h>
SdFat SD;
FsFile myFile;
#define FILE_WRITE (O_RDWR | O_CREAT | O_AT_END)
#define FILE_APPEND FAPPEND
#define SD_CS_PIN 5
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(15))

// BME280 CONFIG
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C
float bme_temperature, bme_altitude, bme_pressure, bme_humidity;

// AM2320 CONFIG
#include <Adafruit_AM2320.h>
Adafruit_AM2320 am2320 = Adafruit_AM2320();
float am_temperature, am_humidity;

// BAT WATCH
float VBAT;
#define VBAT_PIN 33

// ULP SLEEP TIMER
uint64_t UpdateInterval =
    1 * 30 * 1000000;  // e.g. 0.33 * 60 * 1000000; // Sleep time
RTC_DATA_ATTR int bootCount = 0;

// DECLARATIONS
void WriteToCSV(DateTime, float, float, float, float, float, float);
void INIT_SD();
void INIT_RTC();
void INIT_BME();
void Deep_Sleep_Now();
void readdata();
void printdata();
void led_indicator();

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
        ;  // wait for serial port to connect. Needed for native USB port only
    }

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(VBAT_PIN, INPUT);
    ++bootCount;

    // INITIALIZES
    INIT_SD();
    delay(500);
    INIT_RTC();
    delay(500);
    INIT_BME();
    delay(500);
}

void loop() {
    Wire.begin();

    // LOGIC
    delay(2000);
    readdata();
    printdata();

    VBAT = map(analogRead(VBAT_PIN), 0, 4094, 0, 4.2);
    Serial.println();
    Serial.println(VBAT);

    // LOGGING
    DateTime now = rtc.now();

    WriteToCSV(now, bme_temperature, bme_humidity, bme_pressure, bme_pressure,
               am_temperature, am_humidity);

    // Find out how long since the ESP rebooted
    float wake_time = (float)millis() / float(1000);

    // ULP - SLEEP
    Serial.print("Wake Time = ");
    Serial.print(wake_time);
    Serial.println(" seconds");
    delay(100);
    Deep_Sleep_Now();
}

void INIT_SD() {
    led_indicator();
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CONFIG)) {
        led_indicator();
        led_indicator();
        led_indicator();
        SD.initErrorPrint(&Serial);
        Deep_Sleep_Now();
    }
    Serial.println("initialization done.");
    led_indicator();
}

void INIT_RTC() {
    led_indicator();
    if (!rtc.begin()) {
        led_indicator();
        led_indicator();
        led_indicator();
        Serial.println("Couldn't find RTC");
        Deep_Sleep_Now();
    }

    if (!rtc.isrunning()) {
        led_indicator();
        led_indicator();
        led_indicator();
        Serial.println("RTC is not running! Setting the time.");
        // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    led_indicator();
}
void INIT_BME() {
    led_indicator();
    bool status;
    status = bme.begin(0x76);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    led_indicator();
}

void WriteToCSV(DateTime timestamp, float bme_t, float bme_h, float bme_alt,
                float bme_pres, float AM_t, float AM_hu) {
    led_indicator();
    char fileName[25];
    sprintf(fileName, "WS1_%02d%02d%04d.csv", timestamp.day(),
            timestamp.month(), timestamp.year());
    Serial.println(fileName);
    char timeBuffer[20];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", timestamp.hour(),
             timestamp.minute(), timestamp.second());

    // Check if the file exists or create it
    if (!SD.exists(fileName)) {
        led_indicator();
        led_indicator();
        led_indicator();
        // If the file doesn't exist, create it
        myFile = SD.open(fileName, FILE_WRITE);

        if (myFile) {
            // Write header to the file (assuming CSV header format)
            myFile.println(
                "Timestamp;bme_temperature;bme_humidity;bme_pressure;bme_"
                "altitude;am2320_temperature;am2320_humidity");
            myFile.close();
            Serial.println("Header written to file: " + String(fileName));
        } else {
            Serial.println("Error creating file: " + String(fileName));
            return;
        }
    }

    // Open the file for appending data
    led_indicator();
    myFile = SD.open(fileName, FILE_WRITE);

    if (myFile) {
        led_indicator();
        String data = String(timeBuffer) + ";" + String(bme_t) + ";" +
                      String(bme_h) + ";" + String(bme_pres) + ";" +
                      String(bme_alt) + ";" + String(AM_t) + ";" +
                      String(AM_hu);
        data.replace(".", ",");
        // Write sensor data to the file
        myFile.println(data);
        myFile.close();
        Serial.println("Data written to file: " + String(fileName));
        Serial.println(data);
        led_indicator();
    } else {
        led_indicator();
        led_indicator();
        led_indicator();
        Serial.println("Error opening file: " + String(fileName));
    }
}

void Deep_Sleep_Now() {
    led_indicator();
    led_indicator();
    esp_sleep_enable_timer_wakeup(UpdateInterval);
    Serial.println("ESP is tired, going to sleep.");
    Serial.flush();
    esp_deep_sleep_start();

    delay(2000);
}

void printdata() {
    // BME280 READOUT
    led_indicator();
    Serial.print("Air temperature [°C]: ");
    Serial.println(bme_temperature);
    Serial.print("Altitude [m]: ");
    Serial.println(int(bme_altitude));
    Serial.print("Barometric pressure [hPa]: ");
    Serial.println(bme_pressure);
    Serial.print("BME Humidity [%]: ");
    Serial.println(bme_humidity);
    led_indicator();
    // AM2320 READOUT
    led_indicator();
    Serial.print("AM2320 temperature [C]: ");
    Serial.println(am_temperature);
    Serial.print("AM2320 Humidity [%]: ");
    Serial.println(am_humidity);
    led_indicator();
}

void readdata() {
    // Reading BME280 sensor
    led_indicator();
    bme_temperature = bme.readTemperature();
    bme_pressure = bme.readPressure() / 100.0F;
    bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    bme_humidity = bme.readHumidity();
    led_indicator();

    // Reading AM2320 Sensor
    led_indicator();
    am_temperature = am2320.readTemperature();
    am_humidity = am2320.readHumidity();
    led_indicator();
}

void led_indicator() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}