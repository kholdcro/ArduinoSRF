//reference:  http://www.robot-electronics.co.uk/htm/srf08tech.html

#define USE_USBCON
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

//#define MAIN_08_ADDRESS (0x79 >> 1)
#define SRF_0_ADDRESS 0xF2
#define SRF_1_ADDRESS (0xE0 >> 1)

#define COMMAND_REGISTER  0x00
#define MAX_GAIN_REGISTER 0x01
#define RANGE_REGISTER    0x02

#define GAIN_REGISTER 0x09
#define LOCATION_REGISTER 0x8C

bool selSRF = 0;
int16_t sonOut[1];

std_msgs::Int16MultiArray sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);
ros::NodeHandle nh;

void setup()
{
  nh.initNode();
  nh.advertise(pub_sonar);
  delay(500);
  Wire.begin();
  delay(100);
}

void loop()
{
  //start request for data
  requestRange(SRF_0_ADDRESS);
  requestRange(SRF_1_ADDRESS);
  delay(70);
  //read incoming data
  sonOut[0] = readData(SRF_0_ADDRESS);
  sonOut[1] = readData(SRF_1_ADDRESS);

//  distance(res0, res1);
  sonar_msg.data_length = 2;
  sonar_msg.data = &sonOut[0];
  pub_sonar.publish( &sonar_msg );
  nh.spinOnce();

  toggleLights();
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

