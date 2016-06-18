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

template <typename T> struct vector
		{
		  T x, y, z;
		};

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
	if (version5 == true){
		//ver5 reg setup and DT settings etc
		LSM6OBJ.writeReg(LSM6OBJ.CTRL1_XL,0x48);//104Hz +/- 4g
		LSM6OBJ.writeReg(LSM6OBJ.CTRL2_G,0x44);//104Hz 500dps
		LSM6OBJ.writeReg(LSM6OBJ.CTRL2_G,0x04);
		LSM6OBJ.writeReg(LSM6OBJ.CTRL9_XL,0x38);
		LSM6OBJ.writeReg(LSM6OBJ.CTRL10_C,0x38);
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG1,0xC2);//155Hz
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG2,0x20);//+/- 8 gauss
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG3,0x00);//+/- 8 gauss
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG4,0x0C);
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG5,0x00);
		return;
	}
	//setup regs according to devic
	//todo scale factors xy and z mag need to be the same scale factors
	L3GOBJ.writeReg(L3GOBJ.CTRL_REG1,0x0F);//100hz
	L3GOBJ.writeReg(L3GOBJ.CTRL_REG4,0x10);//500dps
	switch ((int)LSM303OBJ.getDeviceType()){
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

	Serial.println("config complete");
}


void loop(){



}






