//reference:  http://www.robot-electronics.co.uk/htm/srf08tech.html

#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>

//#define MAIN_08_ADDRESS (0x79 >> 1)
#define SRF_0_ADDRESS 0xF2
#define SRF_1_ADDRESS (0xE0 >> 1)

#define COMMAND_REGISTER  0x00
#define MAX_GAIN_REGISTER 0x01
#define RANGE_REGISTER    0x02

#define GAIN_REGISTER 0x09
#define LOCATION_REGISTER 0x8C

bool selSRF = 0;
char unit = 'c'; // 'i' for inches, 'c' for centimeters, 'm' for micro-seconds
int loopiter = 0;
unsigned int timer;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(1000);
  Wire.begin();
  Serial.println("derp");
  delay(100);
  timer = millis();
}

void loop()
{
  if (millis()- timer > 1000)
  {
    Serial.print("Sonar reading at: ");
    Serial.print(loopiter);
    Serial.println("Hz");
    timer = millis();
    loopiter = 0;
  }

  //start request for data
  requestRange(SRF_0_ADDRESS);
  requestRange(SRF_1_ADDRESS);
  delay(70);
  //read incoming data
  int res0 = readData(SRF_0_ADDRESS);
  int res1 = readData(SRF_1_ADDRESS);

  distance(res0, res1);
 
  loopiter++;
}

void requestRange(uint8_t address)
{
  Wire.beginTransmission(address); // Start I2C transmission
    Wire.write((uint8_t)(COMMAND_REGISTER)); // Send command
    Wire.write((uint8_t)(0x51)); // range in cm command
  Wire.endTransmission();
}

int readData(uint8_t address)
{
  Wire.beginTransmission(address); // Start I2C transmission
    Wire.write(RANGE_REGISTER); // Send command
  Wire.endTransmission();

  uint8_t l = 2;
  Wire.requestFrom(address, l); // Request length bytes
  while (Wire.available() < l); // Wait for result while bytes available
  int res = 0; // Read the bytes, and combine them into one int
  for (; l > 0; l--)
  { 
    res += Wire.read() << (8 * (l - 1));
  }
  return res;
}

// Print out distance
void distance(int sensorReading0, int sensorReading1)
{
  Serial.print("Sensor reading: \t");
  Serial.print(sensorReading0);
  Serial.print("\t");
  Serial.println(sensorReading1);
}

void toggleLights()
{
  if (selSRF)
  {
    RXLED0;
    TXLED1;
    selSRF = false;
  }
  else
  {
    RXLED1;
    TXLED0;
    selSRF = true;
  }
}

void writeAddressed( uint8_t newAddress, uint8_t oldAddress)
{
  Wire.beginTransmission(oldAddress);
    Wire.write(COMMAND_REGISTER);
    Wire.write(0xA0);
  Wire.endTransmission();
  delay(60);
    
  Wire.beginTransmission(oldAddress);
    Wire.write(COMMAND_REGISTER);
    Wire.write(0xAA);
  Wire.endTransmission();
  delay(60);
    
  Wire.beginTransmission(oldAddress);
    Wire.write(COMMAND_REGISTER);
    Wire.write(0xA5);
  Wire.endTransmission();
  delay(60);
    
  Wire.beginTransmission(oldAddress);
    Wire.write(COMMAND_REGISTER);
    Wire.write((uint8_t)(newAddress<< 1));
  Wire.endTransmission();
}

