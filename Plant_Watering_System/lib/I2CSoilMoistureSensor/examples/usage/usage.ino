// Example usage for I2CSoilMoistureSensor library by Mike Morales.

#include "I2CSoilMoistureSensor.h"

// Initialize objects from the lib for default address of 0x20
I2CSoilMoistureSensor i2CSoilMoistureSensorDefault;
// Use this two lines instead for non-standard address
#define SOIL_SENSOR_ADDRESS 0x30
I2CSoilMoistureSensor i2CSoilMoistureSensor(SOIL_SENSOR_ADDRESS);

void setup() {
    //Init Wire for I2C Communication
    Wire.setSpeed(400000);
    if (!Wire.isEnabled()) {
        Wire.begin();
    }
    // Call functions on initialized library objects that require hardware
    i2CSoilMoistureSensorDefault.begin();
    delay(3000);
}

void loop() {
    // Read moisture value
  while (i2CSoilMoistureSensorDefault.isBusy()) delay(50);
    int soilMoisture = i2CSoilMoistureSensorDefault.getCapacitance();
    delay(1000);
  while (i2CSoilMoistureSensorDefault.isBusy()) delay(50);
    int temperature = i2CSoilMoistureSensorDefault.getTemperature();
    delay(1000);
  while (i2CSoilMoistureSensorDefault.isBusy()) delay(50);
    int light = i2CSoilMoistureSensorDefault.getLight(true);//true initiates startMeasureLightand adds a 3 second pausebefore reading, false reads immediately
    delay(1000);
    //divide raw temperature reading by 10 to get celsius
    float tempInC=(temperature/(float)10);
    float tempInF=(tempInC*9/5)+32;


  Particle.publish("TestSensor", "Soil Moisture Reading: "+ String(soilMoisture));
  Particle.publish("TestSensor", "Soil Temperature (c) Reading: "+ String(tempInC));
  Particle.publish("TestSensor", "Soil Temperature (f) Reading: "+ String(tempInF));
  Particle.publish("TestSensor", "Light Reading: "+ String(light));
  delay(10000);
}
