/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/Brosana/Documents/IoT/Plant_Watering_System/Plant_Watering_System/src/Plant_Watering_System.ino"
/*
 * Project Plant_Watering_System
 * Description:
 * Author:
 * Date:
 */


#include "Adafruit_SSD1306.h"
#include "I2CSoilMoistureSensor.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "JsonParserGeneratorRK.h"
#include "credentials.h"
#include "Adafruit_BME280.h"


/************ Global State (you don't need to change this!) ***   ***************/ 
void setup();
void loop();
void testdrawstyles(void);
void MQTT_connect();
void checkBME (void);
void createEventPayLoad (void);
#line 20 "/Users/Brosana/Documents/IoT/Plant_Watering_System/Plant_Watering_System/src/Plant_Watering_System.ino"
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

//****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish mqttmoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture"); 
Adafruit_MQTT_Publish mqtttemperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish mqtthumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Subscribe mqttplantWatering = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/plant_Monitor");
/************Declare Variables*************/
unsigned long last, lastTime;
SYSTEM_MODE(SEMI_AUTOMATIC);
byte count, i; //8-bit integer that goes from 0 to 255
int moisturePin = A5;
int soilMoisture; 
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET = D4;
const int SCREEN_ADDRESS = 0x3C;
String DateTime , TimeOnly;
//float diceRoll1, diceRoll2, ledOn;
int LEDPIN = D7;
int ledValue; 
bool status; 
float tempC, pressPA, humidRH, tempF, inHG, currentTempF, lastTemp, lastHG;

//void printMoistureRead(moisture);
//void createEventPayLoad();


Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;

void setup()
{
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  delay(1000);

  WiFi.connect();
    while (WiFi.connecting()){
      Serial.printf(".");
    }
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  Time.zone(-6);
  Particle.syncTime();
  delay(1000);
  pinMode(moisturePin, INPUT);
  pinMode(A4, OUTPUT);
  status = bme.begin(0x76);

  Wire.setSpeed(400000);
    if (!Wire.isEnabled())
    {
      Wire.begin();
    }

    if (status == false) {
      Serial.printf("BME280 at address 0x%02X failed to start \n", 0x76);
    }
  //Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&mqttplantWatering);
  //mqtt.subscribe(&mqttbutton1);
}

void loop(){
  MQTT_connect();
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11, 19);
  soilMoisture = analogRead(moisturePin); 
  checkBME();
  Serial.printf("Date and time is %s\n", DateTime.c_str());
  Serial.printf("Time is %s\n", TimeOnly.c_str());
  testdrawstyles();
  delay(10000); // only loop every 10 seconds

  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
   }
   if ((millis()-last)>60000){
    soilMoisture = analogRead(moisturePin); 
    if (soilMoisture > 2600){
    digitalWrite(A4, HIGH);
    delay (500);
    digitalWrite(A4, LOW);
    }

    if ((millis() - last) >60000){
      if(mqtt.Update()) { //if mqtt object (Adafruit.io) is available to receive data
      Serial.printf("Publishing %i to Adafruit.io feed moisture \n",soilMoisture);
      mqttmoisture.publish(soilMoisture); }
      }

      if ((millis() - last) >60000){
        if(mqtt.Update()) { //if mqtt object (Adafruit.io) is available to receive data
        Serial.printf("Publishing %f to Adafruit.io feed moisture \n",tempF);
        mqtttemperature.publish(tempF); 
        }
      }
      if ((millis() - last) >60000){
        if(mqtt.Update()) { //if mqtt object (Adafruit.io) is available to receive data
        Serial.printf("Publishing %f to Adafruit.io feed moisture \n",humidRH);
        mqtthumidity.publish(humidRH); 
        }
      }
    }


   //this is our 'wait for incoming subscription packets' busy subloop
  //  Adafruit_MQTT_Subscribe *subscription;
  //  while ((subscription = mqtt.readSubscription(1000))) {
  //     if (subscription == &mqttplantWatering) {
  //       mqttmoisture = atof((char *)mqttplantWatering.lastread);
  //           Serial.printf("Received %0.2f from Adafruit.io feed FeedNameB \n", mqttmoisture);
  //     }
  //  }
}

void testdrawstyles(void){
  display.clearDisplay();
  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);  
  display.setCursor(10, 0);
  display.printf("Time is %s", TimeOnly.c_str());
  display.display();
  delay(1000);

  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);  
  display.setCursor(10, 10);
  display.printf("Moisture is %i", soilMoisture);
  display.display();
  delay(1000);

  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 20);
  //display.setRotation(2);
  display.printf("Temp is %f", tempF);
  display.display();
  delay(1000);

  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 30);
  //display.setRotation(2);
  display.printf("Humidity is %f", humidRH);
  display.display();
  delay(1000);

}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}

void checkBME (void) {
  tempC = bme.readTemperature();
  pressPA = bme.readPressure();
  humidRH = bme.readHumidity();
  tempF = tempC*9/5+32;
  inHG = pressPA/3386; 
  Serial.printf("%f, %f, %f \n", tempF, inHG, humidRH);
}

 

void createEventPayLoad (void) {
  JsonWriterStatic <256> jw;
  {
    JsonWriterAutoObject obj (&jw);

    jw.insertKeyValue("moisture", soilMoisture);
    //jw.insertKeyValue("lon", coord.lon);

  }
     if(mqtt.Update()) {
      mqttmoisture.publish(jw.getBuffer()); 
      Particle.publish("moisture", jw.getBuffer(), PRIVATE);

    }
}

  
