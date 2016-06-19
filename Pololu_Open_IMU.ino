#include <LSM303.h>
#include <L3G.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Wire.h>
#include <MahonyAHRS.h>
#include <MadgwickAHRS.h>

L3G L3GOBJ;
LSM303 LSM303OBJ;
LSM6 LSM6OBJ;
LIS3MDL LIS3MDLOBJ;


bool version5 = false;

//#define OUTPUT_MAG_FOR_CAL
//#define OUTPUT_ACC_FOR_CAL

#define GYRO_SCALE_FACTOR 0.0175
#define ACC_SCALE_FACTOR_Dxx 0.019140625
#define ACC_SCALE_FACTOR_D_6 0.0011962890625

//http://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html
//https://sites.google.com/site/sailboatinstruments1/home
//download and use magneto12.zip
//user defines
#define W_INV_00 1.0
#define W_INV_01 0.0
#define W_INV_02 0.0

#define W_INV_10 0.0
#define W_INV_11 1.0
#define W_INV_12 0.0

#define W_INV_20 0.0
#define W_INV_21 0.0
#define W_INV_22 1.0

#define MAG_OFF_X 0
#define MAG_OFF_Y 0
#define MAG_OFF_Z 0

#define ACC_OFF_X 0
#define ACC_OFF_Y 0
#define ACC_OFF_Z 0

//user acc scale factors
#define ACC_SCALE_X ACC_SCALE_FACTOR_Dxx
#define ACC_SCALE_Y ACC_SCALE_FACTOR_Dxx
#define ACC_SCALE_Z ACC_SCALE_FACTOR_Dxx

//#define USE_USER_CAL
//end user defines

#define X_ 0
#define Y_ 1
#define Z_ 2

float groScaled[3];
float accScaled[3];
float magScaled[3];

int16_t gyroOffSets[3] = {0,0,0};
float gyroScaleFactor[3] = {GYRO_SCALE_FACTOR,GYRO_SCALE_FACTOR,GYRO_SCALE_FACTOR};

int16_t accOffSets[3] = {ACC_OFF_X,ACC_OFF_Y,ACC_OFF_Z};
float accScaleFactor[3] = {ACC_SCALE_X,ACC_SCALE_Y,ACC_SCALE_Z};

int16_t magOffSets[3] = {MAG_OFF_X,MAG_OFF_Y,MAG_OFF_Z};
//ellipsoid fit matrix
float magScaleMatrix[3][3] = {{W_INV_00 , W_INV_01 , W_INV_02},
							  {W_INV_10 , W_INV_11 , W_INV_12},
							  {W_INV_20 , W_INV_21 , W_INV_22}};


uint32_t tau;

void setup(){
  Serial.begin(115200);
  Serial.println("Keeping the device still and level during startup will yield the best results");
  Wire.begin();
  Wire.setClock(400000);
  //todo include gyro offset cal at startup
  InitSensors();

}



void PollSensors(){
	int16_t shiftedMag[3];
	int16_t groRead[3],accRead[3],magRead[3];
	if (version5 == false){
		L3GOBJ.read();
		LSM303OBJ.readMag();
		LSM303OBJ.readAcc();
		groRead[X_] =  L3GOBJ.g.x;
		groRead[Y_] =  L3GOBJ.g.y;
		groRead[Z_] =  L3GOBJ.g.z;

		accRead[X_] =  LSM303OBJ.a.x;
		accRead[Y_] =  LSM303OBJ.a.y;
		accRead[Z_] =  LSM303OBJ.a.z;

		magRead[X_] =  LSM303OBJ.m.x;
		magRead[Y_] =  LSM303OBJ.m.y;
		magRead[Z_] =  LSM303OBJ.m.z;

		if (LSM303OBJ.getDeviceType() <  LSM303OBJ.device_DLHC){
			accRead[X_] = accRead[X_] >> 4;
			accRead[Y_] = accRead[Y_] >> 4;
			accRead[Z_] = accRead[Z_] >> 4;
		}


		/*Serial.print("Gyro: ");
		Serial.print(L3GOBJ.g.x);
		Serial.print(",");
		Serial.print(L3GOBJ.g.y);
		Serial.print(",");
		Serial.print(L3GOBJ.g.z);
		Serial.print("\r\n");*/
		/*Serial.print("Acc: ");
		Serial.print(LSM303OBJ.a.x>>4);
		Serial.print(",");
		Serial.print(LSM303OBJ.a.y>>4);
		Serial.print(",");
		Serial.print(LSM303OBJ.a.z>>4);
		Serial.print("\r\n");*/
		Serial.print("Mag: ");
		Serial.print(LSM303OBJ.m.x);
		Serial.print(",");
		Serial.print(LSM303OBJ.m.y);
		Serial.print(",");
		Serial.print(LSM303OBJ.m.z);
		Serial.print("\r\n");


	}else{
		LSM6OBJ.readAcc();
		LSM6OBJ.readGyro();
		LIS3MDLOBJ.read();
		groRead[X_] =  LSM6OBJ.g.x;
		groRead[Y_] =  LSM6OBJ.g.y;
		groRead[Z_] =  LSM6OBJ.g.z;

		accRead[X_] =  LSM303OBJ.a.x;
		accRead[Y_] =  LSM303OBJ.a.y;
		accRead[Z_] =  LSM303OBJ.a.z;

		magRead[X_] =  LIS3MDLOBJ.m.x;
		magRead[Y_] =  LIS3MDLOBJ.m.y;
		magRead[Z_] =  LIS3MDLOBJ.m.z;


	}
	groScaled[X_] = gyroScaleFactor[X_] * (groRead[X_] - gyroOffSets[X_]);
	groScaled[Y_] = gyroScaleFactor[Y_] * (groRead[Y_] - gyroOffSets[Y_]);
	groScaled[Z_] = gyroScaleFactor[Z_] * (groRead[Z_] - gyroOffSets[Z_]);

	accScaled[X_] = accScaleFactor[X_] * (accRead[X_] - accOffSets[X_]);
	accScaled[Y_] = accScaleFactor[Y_] * (accRead[Y_] - accOffSets[Y_]);
	accScaled[Z_] = accScaleFactor[Z_] * (accRead[Z_] - accOffSets[Z_]);

	shiftedMag[X_] = magRead[X_] - magOffSets[X_];
	shiftedMag[Y_] = magRead[Y_] - magOffSets[Y_];
	shiftedMag[Z_] = magRead[Z_] - magOffSets[Z_];
	magScaled[X_] = magScaleMatrix[X_][X_] * shiftedMag[X_] +  magScaleMatrix[X_][Y_] * shiftedMag[Y_] + magScaleMatrix[X_][Z_] * shiftedMag[Z_];
	magScaled[Y_] = magScaleMatrix[Y_][X_] * shiftedMag[X_] +  magScaleMatrix[Y_][Y_] * shiftedMag[Y_] + magScaleMatrix[Y_][Z_] * shiftedMag[Z_];
	magScaled[Z_] = magScaleMatrix[Z_][X_] * shiftedMag[X_] +  magScaleMatrix[Z_][Y_] * shiftedMag[Y_] + magScaleMatrix[Z_][Z_] * shiftedMag[Z_];
	/*Serial.print("Gyro: ");
	Serial.print(groScaled[X_]);
	Serial.print(",");
	Serial.print(groScaled[Y_]);
	Serial.print(",");
	Serial.print(groScaled[Z_]);
	Serial.print("\r\n");
	Serial.print("Acc: ");
	Serial.print(accScaled[X_]);
	Serial.print(",");
	Serial.print(accScaled[Y_]);
	Serial.print(",");
	Serial.print(accScaled[Z_]);
	Serial.print("\r\n");*/
	Serial.print("Mag: ");
	Serial.print(magScaled[X_]);
	Serial.print(",");
	Serial.print(magScaled[Y_]);
	Serial.print(",");
	Serial.print(magScaled[Z_]);
	Serial.print("\r\n");
}

void loop(){
	PollSensors();
	delay(500);


}

void InitSensors(){
	/*
	 IMU9
	v1 L3G4200D LSM303DLM
	v2 L3GD20 LSM303DLHC
	v3 L3GD20H LSM303D
	v4 L3GD20H LSM303D - alt
	v5 LSM6DS33 LIS3MDL - alt
	 */
	//check which IMU is connected
	if (L3GOBJ.init() == false) {
		if (LSM6OBJ.init() == false) {
			while (1) {
				Serial.println("init failure");
				delay(500);
			}

		}else{
			if (LIS3MDLOBJ.init() == false){
				while (1) {
					Serial.println("init failure");
					delay(500);
				}
			}
			version5 = true;
		}
	}else{
		if(LSM303OBJ.init() == false){
			while (1) {
				Serial.println("init failure");
				delay(500);
			}

		}
	}
	Serial.println("sensors detected");
	//setup device registers for ~100Hz operation
	if (version5 == true){
		Serial.print("Device types: ");
		switch ((int)LSM6OBJ.getDeviceType()){
		case LSM6OBJ.device_DS33:
			Serial.print("device_DS33");
			break;
		default:
			Serial.print("invalid LSM6 type");
			break;

		}
		Serial.print(" , ");
		switch ((int)LIS3MDLOBJ.getDeviceType()){
		case LIS3MDLOBJ.device_LIS3MDL:
			Serial.print("device_LIS3MDL");
			break;
		default:
			Serial.print("invalid LIS3MDL type");
			break;

		}
		Serial.print("\r\n");
		delay(1500);
		//ver5 reg setup and DT settings etc
		LSM6OBJ.writeReg(LSM6OBJ.CTRL1_XL,0x48);//104Hz +/- 4g default AA filter
		LSM6OBJ.writeReg(LSM6OBJ.CTRL2_G,0x44);//104Hz 500dps
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG1,0xE2);//155Hz UHP fast odr XY
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG2,0x00);//+/- 4 gauss
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG4,0x0C);//Z axis UHP
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG3,0x00);//continuous conversion mode
		accScaleFactor[X_] = ACC_SCALE_FACTOR_D_6;
		accScaleFactor[Y_] = ACC_SCALE_FACTOR_D_6;
		accScaleFactor[Z_] = ACC_SCALE_FACTOR_D_6;
		tau = 10000;
		return;
	}

	Serial.print("Device types: ");
	switch ((int)L3GOBJ.getDeviceType()){
	case L3GOBJ.device_4200D:
		Serial.print("device_4200D");
		break;
	case L3GOBJ.device_D20:
		Serial.print("device_D20");
			break;
	case L3GOBJ.device_D20H:
		Serial.print("device_D20H");
			break;
	default:
		Serial.print("invalid L3G type");
		break;
	}
	Serial.print(" , ");
	switch ((int)LSM303OBJ.getDeviceType()){
	case LSM303OBJ.device_DLH:
		Serial.print("device_DLH");
		break;
	case LSM303OBJ.device_DLM:
		Serial.print("device_DLM");
		break;
	case LSM303OBJ.device_DLHC:
		Serial.print("device_DLHC");
		break;
	case LSM303OBJ.device_D:
		Serial.println("device_D");
		break;
	default:
		Serial.print("invalid LSM303 type");
		break;
	}
	Serial.print("\r\n");
	delay(1500);

	L3GOBJ.writeReg(L3GOBJ.CTRL_REG1,0x0F);//100hz
	L3GOBJ.writeReg(L3GOBJ.CTRL_REG4,0x10);//500dps
	switch ((int)LSM303OBJ.getDeviceType()){
		case LSM303OBJ.device_DLH:
		case LSM303OBJ.device_DLM:
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG4_A,0x10);//Continuous update little endian +/- 4g
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG1_A,0x2F);//normal 100Hz all axes enabled
			LSM303OBJ.writeMagReg(LSM303OBJ.CRA_REG_M,0x18);//75Hz normal mode
			LSM303OBJ.writeMagReg(LSM303OBJ.CRB_REG_M,0xA0);
			LSM303OBJ.writeMagReg(LSM303OBJ.MR_REG_M,0x00);
			accScaleFactor[X_] = ACC_SCALE_FACTOR_Dxx;
			accScaleFactor[Y_] = ACC_SCALE_FACTOR_Dxx;
			accScaleFactor[Z_] = ACC_SCALE_FACTOR_Dxx;
			tau = 13333;
			break;
		case LSM303OBJ.device_DLHC:
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG4_A,0x10);//Continuous update little endian +/- 4g
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG1_A,0x57);//normal 100Hz all axes enabled
			LSM303OBJ.writeMagReg(LSM303OBJ.CRA_REG_M,0x98);//75Hz temp compensation enabled
			LSM303OBJ.writeMagReg(LSM303OBJ.CRB_REG_M,0xA0);
			LSM303OBJ.writeMagReg(LSM303OBJ.MR_REG_M,0x00);
			accScaleFactor[X_] = ACC_SCALE_FACTOR_Dxx;
			accScaleFactor[Y_] = ACC_SCALE_FACTOR_Dxx;
			accScaleFactor[Z_] = ACC_SCALE_FACTOR_Dxx;
			tau = 13333;
			break;
		case LSM303OBJ.device_D:
			//mag sensitivity is the same for XYZ
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL2,0x08);//default AA filter +/- 4g 4 wire SPI
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL1,0x67);//100Hz Continuous update all axes enabled
			LSM303OBJ.writeMagReg(LSM303OBJ.CTRL5,0xF4);//100Hz high res
			LSM303OBJ.writeMagReg(LSM303OBJ.CTRL6,0x20);// +/- 4 gauss
			LSM303OBJ.writeMagReg(LSM303OBJ.CTRL7,0x00);
			accScaleFactor[X_] = ACC_SCALE_FACTOR_D_6;
			accScaleFactor[Y_] = ACC_SCALE_FACTOR_D_6;
			accScaleFactor[Z_] = ACC_SCALE_FACTOR_D_6;
			tau = 10000;
			break;
	}

	Serial.println("config complete");
#ifdef USE_USER_CAL
	accScaleFactor[X_] = ACC_SCALE_X;
	accScaleFactor[Y_] = ACC_SCALE_Y;
	accScaleFactor[Z_] = ACC_SCALE_Z;
#endif

}




