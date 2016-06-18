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

template <typename T> struct vector
		{
		  T x, y, z;
		};
/*void (*GetGro)(void);
void (*GetAcc)(void);
void (*GetMag)(void);*/
vector<int16_t> *groSensor;
vector<int16_t> *accSensor;
vector<int16_t> *magSensor;
vector<float> groScaled;
vector<float> accScaled;
vector<float> magScaled;

void setup(){
  Serial.begin(115200);
  Serial.println("Keeping the device still and level during startup will yield the best results");
  Wire.begin();
  Wire.setClock(400000);
  InitSensors();

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
	bool version5 = false;
	if (L3GOBJ.init() == false) {
		if (LSM6OBJ.init() == false) {
			while (1) {
				Serial.println("init failure");
				delay(500);
			}

		}else{
			if(LSM303OBJ.init() == false){
				while (1) {
					Serial.println("init failure");
					delay(500);
				}

			}
			bool version5 = true;
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
	if (version5 == true){
		//ver5 reg setup and DT settings etc
		return;
	}
	//setup regs according to devic
	//todo scale factors
	L3GOBJ.writeReg(L3GOBJ.CTRL_REG1,0x0F);//100hz
	L3GOBJ.writeReg(L3GOBJ.CTRL_REG4,0x10);//500dps
	switch (LSM303OBJ._device){
		case LSM303OBJ.device_DLH:
		case LSM303OBJ.device_DLM:
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG4_A,0x10);
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG1_A,0x2F);
			LSM303OBJ.writeMagReg(LSM303OBJ.CRA_REG_M,0x18);
			LSM303OBJ.writeMagReg(LSM303OBJ.CRB_REG_M,0xA0);
			LSM303OBJ.writeMagReg(LSM303OBJ.MR_REG_M,0x00);
			break;
		case LSM303OBJ.device_DLHC:
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG4_A,0x10);
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG1_A,0x57);
			LSM303OBJ.writeMagReg(LSM303OBJ.CRA_REG_M,0x98);
			LSM303OBJ.writeMagReg(LSM303OBJ.CRB_REG_M,0xA0);
			LSM303OBJ.writeMagReg(LSM303OBJ.MR_REG_M,0x00);
			break;
		case LSM303OBJ.device_D:
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL2,0x08);
			LSM303OBJ.writeAccReg(LSM303OBJ.CTRL1,0x67);
			LSM303OBJ.writeMagReg(LSM303OBJ.CTRL5,0xF4);//100Hz high res
			LSM303OBJ.writeMagReg(LSM303OBJ.CTRL6,0x40);// +/- 8 gauss
			LSM303OBJ.writeMagReg(LSM303OBJ.CTRL7,0x00);
			break;
	}

}


void loop(){



}






