//this sketch should work for both the LSM303DLHC or the HMC5883L
#include <Wire.h>

#define MAG_ADDRESS 0x1E
#define LSM303_CRA_REG (uint8_t)0x00 
#define LSM303_CRB_REG 0x01
#define LSM303_MR_REG 0x02
#define LSM303_OUT_X_H 0x03

typedef union{
  struct{
    int16_t x;
    int16_t y;
    int16_t z;
  }
  v;
  uint8_t buffer[6];
}
Sensor_t;

Sensor_t mag;

float xMax,xMin,yMax,yMin,zMax,zMin;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  MagInit();
}


void loop(){
  GetMag();
  xMax = max(xMax, mag.v.x);
  yMax = max(yMax, mag.v.y);
  zMax = max(zMax, mag.v.z);

  xMin = min(xMin, mag.v.x);
  yMin = min(yMin, mag.v.y);
  zMin = min(zMin, mag.v.z);

  Serial.print("Min: ");
  Serial.print(" X: ");
  Serial.print((int)xMin);
  Serial.print(" Y: ");
  Serial.print((int)yMin);
  Serial.print(" Z: ");
  Serial.print((int)zMin);

  Serial.print(" Max: ");
  Serial.print(" X: ");
  Serial.print((int)xMax);
  Serial.print(" Y: ");
  Serial.print((int)yMax);
  Serial.print("Z: ");
  Serial.println((int)zMax);
  
  delay(20);



}

void MagInit(){
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_CRA_REG);
  Wire.write(0x1C);//220Hz update rate
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_CRB_REG);
  Wire.write(0x60);//+/- 2.5 gauss
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_MR_REG);
  Wire.write((uint8_t)0x00);//continuous conversion mode
  Wire.endTransmission();
}


void GetMag(){

  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_OUT_X_H);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDRESS, 6);
  //the arudino is a little endian system, but the compass is big endian
  mag.buffer[1] = Wire.read();//X
  mag.buffer[0] = Wire.read();
  mag.buffer[5] = Wire.read();//Z
  mag.buffer[4] = Wire.read();
  mag.buffer[3] = Wire.read();//Y
  mag.buffer[2] = Wire.read();


}

