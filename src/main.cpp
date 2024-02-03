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

// DECLARATIONS
void WriteToCSV(DateTime, float, float);
void INIT_SD();
void INIT_RTC();
float readSensor1();
float readSensor2();

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
        ;  // wait for serial port to connect. Needed for native USB port only
    }

    // INITIALIZES
    INIT_SD();
    INIT_RTC();

    // LOGIC
    float SENS1 = readSensor1();
    float SENS2 = readSensor2();
    DateTime now = rtc.now();
    WriteToCSV(now, SENS1, SENS2);

    delay(5000);
    ESP.restart();
}

void loop() {}

void INIT_SD() {
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CONFIG)) {
        SD.initErrorPrint(&Serial);
        ESP.restart();
    }
    Serial.println("initialization done.");
}

void INIT_RTC() {
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1)
            ;
    }

    if (!rtc.isrunning()) {
        Serial.println("RTC is not running! Setting the time.");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void WriteToCSV(DateTime timestamp, float Value1, float Value2) {
    char fileName[15];
    snprintf(fileName, sizeof(fileName), "WS1_%02d%02d%04d.csv",
             timestamp.day(), timestamp.month(), timestamp.year());

    char timeBuffer[9];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", timestamp.hour(),
             timestamp.minute(), timestamp.second());

    // Check if the file exists or create it
    if (!SD.exists(fileName)) {
        // If the file doesn't exist, create it
        myFile = SD.open(fileName, FILE_WRITE);

        if (myFile) {
            // Write header to the file (assuming CSV header format)
            myFile.println("Timestamp,SensorValue1,SensorValue2");
            myFile.close();
            Serial.println("Header written to file: " + String(fileName));
        } else {
            Serial.println("Error creating file: " + String(fileName));
            return;
        }
    }

    // Open the file for appending data
    myFile = SD.open(fileName, FILE_APPEND);

    if (myFile) {
        // Write sensor data to the file
        myFile.print(timeBuffer);
        myFile.print(",");
        myFile.print(Value1);
        myFile.print(",");
        myFile.println(Value2);

        myFile.close();

        Serial.println("Data written to file: " + String(fileName));
    } else {
        Serial.println("Error opening file: " + String(fileName));
    }
}

float readSensor1() {
    // Replace this with the code to read sensor 1
    return random(0, 100);
}

float readSensor2() {
    // Replace this with the code to read sensor 2
    return random(50, 150);
}