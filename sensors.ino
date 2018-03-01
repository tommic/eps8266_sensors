#include <Wire.h>
#include <ESP8266WiFi.h>
#include "DHT.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       "AP"
#define WLAN_PASS       "xxx"

#define HOST        "192.168.0.xxx"
#define PORT        1883
#define USERNAME    "............."
#define PASSWORD    "............."
#define DHTPIN 2 


DHT dht(DHTPIN, DHT22);
const int AnalogIn  = A0;
#define co2Zero 158;

const int intervall = 30000;


WiFiClient client;
// Adafruit_MQTT_Client mqtt(&client, HOST, PORT, USERNAME, PASSWORD);
Adafruit_MQTT_Client mqtt(&client, HOST, PORT);
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, "humiditySensor/temperature");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, "humiditySensor/humidity");
Adafruit_MQTT_Publish voltage = Adafruit_MQTT_Publish(&mqtt, "humiditySensor/voltage");
Adafruit_MQTT_Publish co2 = Adafruit_MQTT_Publish(&mqtt, "co2/value");
void MQTT_connect();


void setup() {
  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  dht.begin();

  MQTT_connect();
  temperature.publish(dht.readTemperature());
  Serial.println(dht.readTemperature());
  humidity.publish(dht.readHumidity());
  Serial.println(dht.readHumidity());


 int co2now[10];                               //int array for co2 readings
  int co2raw = 0;                               //int for raw value of co2
  int co2comp = 0;                              //int for compensated co2 
  int co2ppm = 0;                               //int for calculated ppm
  int zzz = 0;                                  //int for averaging

  for (int x = 0;x<10;x++){                   //samplpe co2 10x over 2 seconds
    co2now[x]=analogRead(AnalogIn);
    delay(200);
  }

  for (int x = 0;x<10;x++){                     //add samples together
    zzz=zzz + co2now[x];
  }
  co2raw = zzz/10;                            //divide samples by 10
  co2comp = co2raw - co2Zero;                 //get compensated value
  co2ppm = map(co2comp,0,1023,400,5000);      //map value for atmospheric levels


  co2.publish(co2ppm);
  Serial.println(co2ppm);
  delay(500);
  Serial.println("deep sleep");
  ESP.deepSleep(intervall * 1000/*, WAKE_RF_DISABLED*/);

  
}

void loop() 
{
}



void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
