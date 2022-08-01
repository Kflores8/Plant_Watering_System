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
#include "Grove_Air_quality_Sensor.h"
#include "math.h"


/************ Global State (you don't need to change this!) ***   ***************/ 
void setup();
void loop();
void readingUpdate(void);
void readingUpdate1(void);
void MQTT_connect();
void checkBME (void);
void createEventPayLoad (void);
void getDust();
void airQuality();
#line 22 "/Users/Brosana/Documents/IoT/Plant_Watering_System/Plant_Watering_System/src/Plant_Watering_System.ino"
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

//****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish mqttmoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture"); 
Adafruit_MQTT_Publish mqtttemperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish mqtthumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish mqttgetDust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/concentration");
Adafruit_MQTT_Publish mqttairQuality = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airValue");
Adafruit_MQTT_Subscribe mqttbutton1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ButtonFeed");
/************Declare Variables*************/
unsigned long last, last2, last3, lastTime;
SYSTEM_MODE(SEMI_AUTOMATIC);
byte count, i; //8-bit integer that goes from 0 to 255
int moisturePin = A5;
int waterPump = A4; 
int soilMoisture; 
int LEDPIN = D7;
int ledValue; 
int current_quality =-1;
int dustPin = D8;
int qualityValue;
int airValue; 

const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET = D4;
const int SCREEN_ADDRESS = 0x3C;

String DateTime , TimeOnly;

unsigned long duration;
unsigned long starttime; 
unsigned long timems = 60000;
unsigned long lowpulseoccupancy = 0;

float tempC, pressPA, humidRH, tempF, inHG, currentTempF, lastTemp, lastHG;
float ratio = 0;
float concentration = 0;

bool status, buttonOnOff; 

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;
AirQualitySensor sensor(A0);

void setup() {
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
  sensor.init();
  Particle.syncTime();
  delay(1000);
  pinMode(dustPin, INPUT);
  starttime = millis();
  pinMode(moisturePin, INPUT);
  pinMode(waterPump, OUTPUT);

  Serial.printf("Waiting sensor to init...");
  delay(20000);
  
    if (sensor.init()) {
      Serial.printf("Sensor ready.");
    }
    else {
      Serial.printf("Sensor ERROR!");
    }

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
  mqtt.subscribe(&mqttbutton1);
  //mqtt.subscribe(&mqttbutton1);
}

void loop(){
  MQTT_connect();

  Adafruit_MQTT_Subscribe *subscription;
  if (subscription == &mqttbutton1) {
  buttonOnOff = atoi((char *)mqttbutton1.lastread);
      if (buttonOnOff = 1) { 
      digitalWrite(waterPump, HIGH); 
      delay (500);
      digitalWrite(waterPump, LOW); 
      }
      Serial.printf("Recieved %i from Adafruit.io feed ButtonFeed \n", buttonOnOff);
  } 
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11, 19);
  checkBME();
  getDust();
  airQuality();
  Serial.printf("Date and time is %s\n", DateTime.c_str());
  Serial.printf("Time is %s\n", TimeOnly.c_str());
  
  readingUpdate();
  display.clearDisplay();
  readingUpdate1();
  display.clearDisplay();



  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
   }
   if ((millis()-last2)>1800000){
    soilMoisture = analogRead(moisturePin); 
    if (soilMoisture > 2600){
    digitalWrite(waterPump, HIGH);
    delay (500);
    digitalWrite(waterPump, LOW);
    last2 = millis();
    }
   }

    if ((millis() - last3) > 100000){
      if(mqtt.Update()) { //if mqtt object (Adafruit.io) is available to receive data
      Serial.printf("Publishing %i to Adafruit.io feed moisture \n",soilMoisture);
      Serial.printf("Publishing %f to Adafruit.io feed temperature \n",tempF);
      Serial.printf("Publishing %f to Adafruit.io feed humidity \n",humidRH);
      Serial.printf("Publishing %f to Adafruit.io feed particulants in cf \n",concentration);
      Serial.printf("Sensor value: %i, \n Quality value: %i to Adafruit.io feed Air Quality\n", airValue);
      mqttmoisture.publish(soilMoisture);
      mqtttemperature.publish(tempF);
      mqtthumidity.publish(humidRH);
      mqttgetDust.publish(concentration);
      mqttairQuality.publish(airValue);
      last3 = millis();
      
    }
  }
}


void readingUpdate(void){
  display.clearDisplay();
  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);  
  display.setCursor(10, 0);
  display.printf("Time is %s", TimeOnly.c_str());
  display.display();


  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);  
  display.setCursor(10, 10);
  display.printf("Moisture is %i", soilMoisture);
  display.display();


  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 20);
  //display.setRotation(2);
  display.printf("Temp is %f", tempF);
  display.display();
  delay(5000);

  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 30);
  //display.setRotation(2);
  display.printf("Humidity is %f", humidRH);
  display.display();


}

void readingUpdate1(void){

  display.clearDisplay();
  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);  
  display.setCursor(10, 0);
  display.printf("Time is %s", TimeOnly.c_str());
  display.display();


  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 20);
  //display.setRotation(2);
  display.printf("Particulants is %f", concentration);
  display.display();


  display.setTextSize(1);  //Draw 5x-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 40);
  //display.setRotation(2);
  display.printf("Air Quality %i", airValue);
  display.display();
  delay(5000);

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

void getDust() {
duration = pulseIn(dustPin, LOW);
lowpulseoccupancy = lowpulseoccupancy+duration; 

  if ((millis()-starttime) > timems) {
    ratio = lowpulseoccupancy/(timems*10.00);
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;
    Serial.printf("lowpulseoccupancy, ratio, and concentration is %f \n", concentration);
    Serial.printf("ratio is %f \n", ratio);
    Serial.printf("lowpulseoccupancy is %i \n", lowpulseoccupancy);
    lowpulseoccupancy = 0;
    starttime = millis();

  }
}

void airQuality() {
  
  qualityValue = sensor.slope();
  airValue = sensor.getValue(); 
  Serial.printf("Sensor value: %i, \n Quality value: %i \n", airValue, qualityValue);
  
  if (qualityValue == AirQualitySensor::FORCE_SIGNAL) {
    Serial.printf("High pollution! Force signal active.");
  }
  else if (qualityValue == AirQualitySensor::HIGH_POLLUTION) {
    Serial.printf("High pollution!");
  }
  else if (qualityValue == AirQualitySensor::LOW_POLLUTION) {
    Serial.printf("Low pollution!");
  }
  else if (qualityValue == AirQualitySensor::FRESH_AIR) {
    Serial.printf("Fresh air.");
  }
}

