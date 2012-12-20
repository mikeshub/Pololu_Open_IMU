/*
Based on the Madgwick algorithm found at:
 See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 
 This code inherits all relevant liscenses and may be freely modified and redistributed.
 The MinIMU v1 has a roughly +/- 10degree accuracy
 The MinIMU v2 has a roughly +/- 1 degree accuracy
 */
#include <LSM303.h>
#include <L3G.h>
#include <Wire.h>

#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
#define GYRO_SCALE 0.07f
#define betaDef		0.08f

//To find the calibration values us the sketch included with the LSM303 driver from pololu
/*Change line 11 from
 
 compass.enableDefault();
 
 to 
 
 compass.writeMagReg(LSM303_CRA_REG_M, 0x1C);
 compass.writeMagReg(LSM303_CRB_REG_M, 0x60);
 compass.writeMagReg(LSM303_MR_REG_M, 0x00);  
 
 Then put the calibration values below
 
 */

#define compassXMax 216.0f
#define compassXMin -345.0f
#define compassYMax 210.0f
#define compassYMin -347.0f
#define compassZMax 249.0f
#define compassZMin -305.0f
#define inverseXRange (float)(2.0 / (compassXMax - compassXMin))
#define inverseYRange (float)(2.0 / (compassYMax - compassYMin))
#define inverseZRange (float)(2.0 / (compassZMax - compassZMin))



L3G gyro;
LSM303 compass;

long timer, printTimer;
float G_Dt;
int loopCount;

float q0;
float q1;
float q2;
float q3;
float beta;

float magnitude;

float pitch,roll,yaw;

float gyroSumX,gyroSumY,gyroSumZ;
float offSetX,offSetY,offSetZ;

float floatMagX,floatMagY,floatMagZ;
float smoothAccX,smoothAccY,smoothAccZ;
float accToFilterX,accToFilterY,accToFilterZ;

int i;

void setup(){
  Serial.begin(115200);
  Serial.println("Keeping the device still and level during startup will yeild the best results");
  Wire.begin();
  TWBR = ((F_CPU / 400000) - 16) / 2;//set the I2C speed to 400KHz
  IMUinit();
  printTimer = millis();
  timer = micros();
}



void loop(){

  if (micros() - timer >= 5000){
    //this runs in 4ms on the MEGA 2560
    G_Dt = (micros() - timer)/1000000.0;
    timer=micros();
    compass.read();
    floatMagX = ((float)compass.m.x - compassXMin) * inverseXRange - 1.0;
    floatMagY = ((float)compass.m.y - compassYMin) * inverseYRange - 1.0;
    floatMagZ = ((float)compass.m.z - compassZMin) * inverseZRange - 1.0;
    Smoothing(&compass.a.x,&smoothAccX);
    Smoothing(&compass.a.y,&smoothAccY);
    Smoothing(&compass.a.z,&smoothAccZ);
    accToFilterX = smoothAccX;
    accToFilterY = smoothAccY;
    accToFilterZ = smoothAccZ;
    gyro.read();
    AHRSupdate(&G_Dt);
  }

  if (millis() - printTimer > 50){
    printTimer = millis();
    GetEuler();
    Serial.print(printTimer);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.println(yaw);
  }

}

void IMUinit(){

  //init devices
  compass.init();
  gyro.init();

  gyro.writeReg(L3G_CTRL_REG1, 0xCF);
  gyro.writeReg(L3G_CTRL_REG2, 0x00);
  gyro.writeReg(L3G_CTRL_REG3, 0x00);
  gyro.writeReg(L3G_CTRL_REG4, 0x20); //
  gyro.writeReg(L3G_CTRL_REG5, 0x02);

  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x77);//400hz all enabled
  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x20);//+/-8g 4mg/LSB

  compass.writeMagReg(LSM303_CRA_REG_M, 0x1C);
  compass.writeMagReg(LSM303_CRB_REG_M, 0x60);
  compass.writeMagReg(LSM303_MR_REG_M, 0x00);  

  beta = betaDef;
  //calculate initial quaternion
  //take an average of the gyro readings to remove the bias 

  for (i = 0; i < 500;i++){
    gyro.read();
    compass.read();
    Smoothing(&compass.a.x,&smoothAccX);
    Smoothing(&compass.a.y,&smoothAccY);
    Smoothing(&compass.a.z,&smoothAccZ);
    delay(3);
  }
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (i = 0; i < 500;i++){
    gyro.read();
    compass.read();
    Smoothing(&compass.a.x,&smoothAccX);
    Smoothing(&compass.a.y,&smoothAccY);
    Smoothing(&compass.a.z,&smoothAccZ);
    gyroSumX += (gyro.g.x);
    gyroSumY += (gyro.g.y);
    gyroSumZ += (gyro.g.z);
    delay(3);
  }
  offSetX = gyroSumX / 500.0;
  offSetY = gyroSumY / 500.0;
  offSetZ = gyroSumZ / 500.0;
  compass.read();

  //calculate the initial quaternion 
  //these are rough values. This calibration works a lot better if the device is kept as flat as possible
  //find the initial pitch and roll
  pitch = ToDeg(fastAtan2(compass.a.x,sqrt(compass.a.y * compass.a.y + compass.a.z * compass.a.z)));
  roll = ToDeg(fastAtan2(-1*compass.a.y,sqrt(compass.a.x * compass.a.x + compass.a.z * compass.a.z)));


  if (compass.a.z > 0){
    if (compass.a.x > 0){
      pitch = 180.0 - pitch;
    }
    else{
      pitch = -180.0 - pitch;
    }
    if (compass.a.y > 0){
      roll = -180.0 - roll;
    }
    else{
      roll = 180.0 - roll;
    }
  }  

  floatMagX = (compass.m.x - compassXMin) * inverseXRange - 1.0;
  floatMagY = (compass.m.y - compassYMin) * inverseYRange - 1.0;
  floatMagZ = (compass.m.z - compassZMin) * inverseZRange - 1.0;
  //tilt compensate the compass
  float xMag = (floatMagX * cos(ToRad(pitch))) + (floatMagZ * sin(ToRad(pitch)));
  float yMag = -1 * ((floatMagX * sin(ToRad(roll))  * sin(ToRad(pitch))) + (floatMagY * cos(ToRad(roll))) - (floatMagZ * sin(ToRad(roll)) * cos(ToRad(pitch))));

  yaw = ToDeg(fastAtan2(yMag,xMag));

  if (yaw < 0){
    yaw += 360;
  }  
  Serial.println(pitch);
  Serial.println(roll);
  Serial.println(yaw);
  //calculate the rotation matrix
  float cosPitch = cos(ToRad(pitch));
  float sinPitch = sin(ToRad(pitch));

  float cosRoll = cos(ToRad(roll));
  float sinRoll = sin(ToRad(roll));

  float cosYaw = cos(ToRad(yaw));
  float sinYaw = sin(ToRad(yaw));

  //need the transpose of the rotation matrix
  float r11 = cosPitch * cosYaw;
  float r21 = cosPitch * sinYaw;
  float r31 = -1.0 * sinPitch;

  float r12 = -1.0 * (cosRoll * sinYaw) + (sinRoll * sinPitch * cosYaw);
  float r22 = (cosRoll * cosYaw) + (sinRoll * sinPitch * sinYaw);
  float r32 = sinRoll * cosPitch;

  float r13 = (sinRoll * sinYaw) + (cosRoll * sinPitch * cosYaw);
  float r23 = -1.0 * (sinRoll * cosYaw) + (cosRoll * sinPitch * sinYaw);
  float r33 = cosRoll * cosPitch;



  //convert to quaternion
  q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
  q1 = (r32 - r23)/(4 * q0);
  q2 = (r13 - r31)/(4 * q0);
  q3 = (r21 - r12)/(4 * q0);


}

void IMUupdate(float *dt) {
  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;

  static float recipNorm;
  static float s0, s1, s2, s3;
  static float qDot1, qDot2, qDot3, qDot4;
  static float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  gx = ToRad((gyro.g.x - offSetX) * GYRO_SCALE);
  gy = ToRad((gyro.g.y - offSetY) * GYRO_SCALE);
  gz = ToRad((gyro.g.z - offSetZ) * GYRO_SCALE);

  ax = -1.0 * compass.a.x;
  ay = -1.0 * compass.a.y;
  az = -1.0 * compass.a.z;
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(ax * ax + ay * ay + az * az);
  if ((magnitude > 384) || (magnitude < 128)){
    ax = 0;
    ay = 0;
    az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRSupdate(float *dt) {
  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;
  static float mx;
  static float my;
  static float mz;


  static float recipNorm;
  static float s0, s1, s2, s3;
  static float qDot1, qDot2, qDot3, qDot4;
  static float hx, hy;
  static float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  gx = ToRad((gyro.g.x - offSetX) * GYRO_SCALE);
  gy = ToRad((gyro.g.y - offSetY) * GYRO_SCALE);
  gz = ToRad((gyro.g.z - offSetZ) * GYRO_SCALE);

  ax = -1.0 * compass.a.x;
  ay = -1.0 * compass.a.y;
  az = -1.0 * compass.a.z;

  mx = floatMagX;
  my = floatMagY;
  mz = floatMagZ;
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(ax * ax + ay * ay + az * az);

  if ((magnitude > 384) || (magnitude < 128)){
    ax = 0;
    ay = 0;
    az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {


    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;



    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void GetEuler(void){
  roll = ToDeg(fastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2)));
  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1)));
  yaw = ToDeg(fastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));
  if (yaw < 0){
    yaw +=360;
  }

}
float fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}



void Smoothing(float *raw, float *smooth){
  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
}











