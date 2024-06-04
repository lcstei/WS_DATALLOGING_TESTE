//=====================Cabeçalhos===========================
#include <Adafruit_AM2320.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BME280.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <EnvironmentCalculations.h>
#include "esp_sleep.h" 
#include "PubSubClient.h"

//=============== Objetos====================================
#define led 2
#define VOLT_PIN     33
#define mqtt_server "192.168.15.74" 
Adafruit_AM2320 am2320 = Adafruit_AM2320();

WiFiClient espClient;
PubSubClient client(espClient);
BME280I2C bme;

// Variables used in reading temp,pressure and humidity (BME280)
  float temperature, humidity, pressure;
// variaveis do AM2320    
  float temp_2320, humidity_2320;


// Variables used in calculating the battery voltage 
    float batteryVolt;   
    float Vout = 0.00;
    float Vin = 0.00;
    float R1 = 27000.00; // resistance of R1 (27K) // You can also use 33K 
    float R2 = 100000.00; // resistance of R2 (100K) 
    int val = 0; 



//=========================Deep Sleep Time ================================================ 

 uint64_t UpdateInterval = 1 * 60 * 1000000;  // e.g. 0.33 * 60 * 1000000; // Sleep time  
 //const int UpdateInterval = 15 * 60 * 1000000;  // e.g. 15 * 60 * 1000000; // // Example for a 15-Min update interval 15-mins x 60-secs * 10000
  //#define TIME_TO_SLEEP 7200      /* Time ESP32 will go to sleep (in seconds) */
//float sleep_time_minutes =     1;   // New variable - how long (in minutes) we sleep for. As this is a float variable then it can be 0.5 for a 30 second test

 //========================= Variables for wifi server setup =============================
     
  // Your WiFi credentials.
  // Set password to "" for open networks.
  char ssid[] = "Lucas"; // WiFi Router ssid
  char pass[] = "159753lucas"; // WiFi Router password

  int wifi_connect_count = 0;          // New variable to keep track of how manty times we've tried to connect to the Wi-Fi
  int wifi_connect_max_retries = 50;   // New variable to specify how many attempts we will have at connecting to the Wi-Fi
  
 

  RTC_DATA_ATTR int bootCount = 0;
  
void setup() {
  Serial.begin(115200);
  delay(25);

  Serial.println("\nWeather station powered on.\n");
  Wire.begin();  
  WiFi_Connect();

 if (WiFi.status() != WL_CONNECTED){
    Serial.println ("Wi-Fi connection failed - going to sleep");
    //sleep_time_minutes = sleep_time_minutes * 2; // If you enable this line of code the it will make the device go to sleep for twice as long before trying again. Changing to 0.5 would make it try again sooner than normal
    Deep_Sleep_Now();
  }
  
  ++bootCount;


  Serial.println("wifi connected");
  Serial.println("Boot number: " + String(bootCount));
      
  delay(2000);     
  bme.begin(); // 0x76 is the address of the BME280 module
  am2320.begin();
      
      
}  // End of Setup function

void loop() {
  
  delay(2000);
  readdata();
  getBattery();
  printdata(); 
  delay(2000);       
  Send_Data();  
 
}

void printdata(){
  // printa os dados do BME280
   Serial.print("Air temperature [°C]: "); Serial.println(temperature);
   Serial.print("Humidity [%]: "); Serial.println(int(humidity));
   Serial.print("Barometric pressure [hPa]: "); Serial.println(pressure / 100);  
   delay(1000);
   Serial.print("AM2320 temperature [C]: "); Serial.println(temp_2320);
   Serial.print("AM2320 Humidity [%]: "); Serial.println(humidity_2320);
    
  }

void readdata(){
  
  // Reading BME280 sensor
    bme.read(pressure, temperature, humidity, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);
    
  // Reading AM2320 Sensor
  
  temp_2320 = am2320.readTemperature();     
  humidity_2320 = am2320.readHumidity();     

}

void Send_Data()
{
  client.setServer(mqtt_server, 1883);
  if (!client.connected()) {
    while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic","hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
    }
  }
  
  // Publish temperature to MQTT topic
  client.publish("WS03/BME280/temperature", String(temperature).c_str());
  client.publish("WS03/BME280/humidity", String(humidity).c_str());
  client.publish("WS03/BME280/pressure", String(pressure / 100).c_str());
  client.publish("WS03/AM2320/temperature", String(temp_2320).c_str());
  client.publish("WS03/AM2320/humidity", String(humidity_2320).c_str());

  // // Publish humidity to MQTT topic
  // client.publish("WS03/bme280/humidity", String(humidity), 24, false);

  // // Publish pressure to MQTT topic
  // client.publish("WS03/bme280/pressure", String(pressure), 24, false);

  


  float wake_time = (float)millis()/float(1000); // Find out how long since the ESP rebooted

  Serial.print("Wake Time = ");
  Serial.print(wake_time);
  Serial.println(" seconds");


  delay(100); 

  Deep_Sleep_Now();   
   
}

void WiFi_Connect() // New functon to handle the connection to the Wi-Fi network
{
  Serial.println(F("Connecting to Wi-Fi"));
  //WiFi.config(device_ip, dns, gateway, subnet); // Not needed if you just want to have a DHCP assigned IP address. If you diont use this then delete the device_ip, dns, gateway & subnet declarations
    
  if (WiFi.status() != WL_CONNECTED)
  {
      WiFi.begin(ssid, pass); // connect to the network
  }
  while (WiFi.status() != WL_CONNECTED  && wifi_connect_count < wifi_connect_max_retries) // Loop until we've connected, or reached the maximum number of attemps allowed
  {
    delay(500);
    wifi_connect_count++;   
    Serial.print(F("Wi-Fi connection - attempt number "));
    Serial.println(wifi_connect_count);
  }
  
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFi.mode(WIFI_STA);
    Serial.println(F("Wi-Fi CONNECTED"));
    Serial.println();
  }
} // End of void WiFi_Connect


void getBattery()
{
  //Reading Battery Level in %
  val = analogRead(VOLT_PIN);//reads the analog input
  Vout = (val * 3.3 ) / 4095.0; // formula for calculating voltage out 
  batteryVolt = Vout *( R2+R1)/ R2; // formula for calculating voltage in
}
   

void Deep_Sleep_Now() // New function - moded code out of void loop so that the sleep function can be called if we fail to connect to Wi-Fi
{
  esp_sleep_enable_timer_wakeup(UpdateInterval);
  Serial.println("ESP is tired, going to sleep.");
  Serial.flush(); 
  esp_deep_sleep_start();
  
  delay(2000);
}
  
