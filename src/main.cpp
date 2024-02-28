#include <Arduino.h>

// RTC - Real Time Clock Config
#include <RTClib.h>
#include <Wire.h>
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

// Station config
#include <Adafruit_AM2320.h>
#include <BME280.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
#include <Wire.h>

#include "esp_sleep.h"

Adafruit_AM2320 am2320 = Adafruit_AM2320();
BME280I2C bme;
// Variables used in reading temp,pressure and humidity (BME280)
float temperature, humidity, pressure;
// variaveis do AM2320
float temp_2320, humidity_2320;

uint64_t UpdateInterval =
    5 * 60 * 1000000;  // e.g. 0.33 * 60 * 1000000; // Sleep time
RTC_DATA_ATTR int bootCount = 0;

// DECLARATIONS
void WriteToCSV(DateTime, float, float, float, float, float);
void INIT_SD();
void INIT_RTC();
void Deep_Sleep_Now();
void readdata();
void printdata();

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
        ;  // wait for serial port to connect. Needed for native USB port only
    }

    ++bootCount;

    // INITIALIZES
    INIT_SD();
    INIT_RTC();
}

void loop() {
    // LOGIC
    delay(2000);
    readdata();
    printdata();

    DateTime now = rtc.now();

    WriteToCSV(now, temperature, humidity, pressure / 100, temp_2320,
               humidity_2320);

    float wake_time = (float)millis() /
                      float(1000);  // Find out how long since the ESP rebooted

    Serial.print("Wake Time = ");
    Serial.print(wake_time);
    Serial.println(" seconds");

    delay(100);

    Deep_Sleep_Now();
}

void INIT_SD() {
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CONFIG)) {
        SD.initErrorPrint(&Serial);
        Deep_Sleep_Now();
    }
    Serial.println("initialization done.");
}

void INIT_RTC() {
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        Deep_Sleep_Now();
        while (1) {
            ;
        }
    }

    if (!rtc.isrunning()) {
        Serial.println("RTC is not running! Setting the time.");
        // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void WriteToCSV(DateTime timestamp, float temp, float humi, float press,
                float tmp_2320, float humi_2320) {
    char fileName[25];
    sprintf(fileName, "WS1_%02d%02d%04d.csv", timestamp.day(),
            timestamp.month(), timestamp.year());
    Serial.println(fileName);
    char timeBuffer[20];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", timestamp.hour(),
             timestamp.minute(), timestamp.second());

    // Check if the file exists or create it
    if (!SD.exists(fileName)) {
        // If the file doesn't exist, create it
        myFile = SD.open(fileName, FILE_WRITE);

        if (myFile) {
            // Write header to the file (assuming CSV header format)
            myFile.println(
                "Timestamp;temperature;humidity;pressure;temp_2320;humidity_"
                "2320");
            myFile.close();
            Serial.println("Header written to file: " + String(fileName));
        } else {
            Serial.println("Error creating file: " + String(fileName));
            return;
        }
    }

    // Open the file for appending data
    myFile = SD.open(fileName, FILE_WRITE);

    if (myFile) {
        String data = String(timeBuffer) + ";" + String(temp) + ";" +
                      String(humi) + ";" + String(press) + ";" +
                      String(tmp_2320) + ";" + String(humi_2320);
        data.replace(".", ",");
        // Write sensor data to the file
        myFile.println(data);
        myFile.close();
        Serial.println("Data written to file: " + String(fileName));
        Serial.println(data);
    } else {
        Serial.println("Error opening file: " + String(fileName));
    }
}

void Deep_Sleep_Now() {
    esp_sleep_enable_timer_wakeup(UpdateInterval);
    Serial.println("ESP is tired, going to sleep.");
    Serial.flush();
    esp_deep_sleep_start();

    delay(2000);
}

void printdata() {
    // printa os dados do BME280
    Serial.print("Air temperature [°C]: ");
    Serial.println(temperature);
    Serial.print("Humidity [%]: ");
    Serial.println(int(humidity));
    Serial.print("Barometric pressure [hPa]: ");
    Serial.println(pressure / 100);
    delay(1000);
    Serial.print("AM2320 temperature [C]: ");
    Serial.println(temp_2320);
    Serial.print("AM2320 Humidity [%]: ");
    Serial.println(humidity_2320);
}

void readdata() {
    // Reading BME280 sensor
    bme.read(pressure, temperature, humidity, BME280::TempUnit_Celsius,
             BME280::PresUnit_Pa);

    // Reading AM2320 Sensor

    temp_2320 = am2320.readTemperature();
    humidity_2320 = am2320.readHumidity();
}