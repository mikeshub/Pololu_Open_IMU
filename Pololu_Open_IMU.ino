#include <LSM303.h>
#include <L3G.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Wire.h>


/*todo define / detection for 32 bit
* switch to Madgwick filter and run 6dof at 400Hz
* also test online version of Madgwick with integral feedback
* add declination rotation?
* Make into library
* Test with other IMU versions. Tested on 0J3865 and 0J3337
* Output for python visualization
*/
L3G L3GOBJ;
LSM303 LSM303OBJ;
LSM6 LSM6OBJ;
LIS3MDL LIS3MDLOBJ;

//function prototypes
bool StationaryGyro();
void SetGyroAccMag();
void GetOffsetsAndInitialQuat();
void SetVariables();
void GenerateRotationMatrix();
void GetPitch();
void GetRoll();
void GetYaw();
void GetEuler();
void AHRS();
void InitSensors();
void PollSensors();
void OutputForCalibration();
//------
bool version5 = false;


//#define OUTPUT_FOR_CAL
//#define OUTPUT_MAG_FOR_CAL
#define OUTPUT_ACC_FOR_CAL
#ifdef OUTPUT_MAG_FOR_CAL
#ifdef OUTPUT_ACC_FOR_CAL
#undef OUTPUT_ACC_FOR_CAL
#endif
#endif

#define GYRO_SCALE_FACTOR 0.0175
#define ACC_SCALE_FACTOR_Dxx 0.019140625
#define ACC_SCALE_FACTOR_D_6 0.0011962890625

//http://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html
//https://sites.google.com/site/sailboatinstruments1/home
//download and use magneto12.zip
//user defines
#define W_INV_00 1.010225
#define W_INV_01 0.003858
#define W_INV_02 -0.019080

#define W_INV_10 0.003858
#define W_INV_11 1.061248
#define W_INV_12 -0.004000

#define W_INV_20 -0.019080
#define W_INV_21 -0.004000
#define W_INV_22 0.885518

#define MAG_OFF_X -0.176768
#define MAG_OFF_Y -1.051859
#define MAG_OFF_Z -6.395059

#define ACC_OFF_X -150
#define ACC_OFF_Y -50
#define ACC_OFF_Z -550

//user acc scale factors
#define ACC_SCALE_X 0.00120245398773006134969325153374
#define ACC_SCALE_Y 0.00120245398773006134969325153374
#define ACC_SCALE_Z 0.00118787878787878787878787878788
/*#define W_INV_00 1.007206
#define W_INV_01 0.007963
#define W_INV_02 0.009474

#define W_INV_10 0.007963
#define W_INV_11 1.042308
#define W_INV_12 0.010689

#define W_INV_20 0.009474
#define W_INV_21 0.010689
#define W_INV_22 0.967861

#define MAG_OFF_X -54.130350
#define MAG_OFF_Y 11.981912
#define MAG_OFF_Z -41.650412

#define ACC_OFF_X 13
#define ACC_OFF_Y 0
#define ACC_OFF_Z -20

//user acc scale factors
#define ACC_SCALE_X 0.01895551257253384912959381044487
#define ACC_SCALE_Y 0.0196
#define ACC_SCALE_Z 0.01884615384615384615384615384615*/

#define USE_USER_CAL
//end user defines

#define X_ 0
#define Y_ 1
#define Z_ 2
#define NUMBER_SAMPLES_FOR_AVG 50

float groScaled[3];
float accScaled[3];
float magScaled[3];

int16_t groRead[3], accRead[3], magRead[3];

float gyroOffSets[3] = { 0, 0, 0 };
float gyroScaleFactor[3] = { GYRO_SCALE_FACTOR, GYRO_SCALE_FACTOR,
GYRO_SCALE_FACTOR };

float accOffSets[3] = { ACC_OFF_X, ACC_OFF_Y, ACC_OFF_Z };
float accScaleFactor[3] = { ACC_SCALE_X, ACC_SCALE_Y, ACC_SCALE_Z };

float magOffSets[3] = { MAG_OFF_X, MAG_OFF_Y, MAG_OFF_Z };
//ellipsoid fit matrix
float magScaleMatrix[3][3] = { { W_INV_00, W_INV_01, W_INV_02 },
							   { W_INV_10, W_INV_11, W_INV_12 },
							   { W_INV_20, W_INV_21, W_INV_22 } };

uint32_t tau;
uint32_t currentTime, previousTime;
float dt;

//AHRS vars
float q0q0, q1q1, q2q2, q3q3, q0q1, q0q2, q0q3, q1q2, q1q3, q2q3;
float acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gro_x, gro_y, gro_z;
float yawInDegrees, pitchInDegrees, rollInDegrees;
float yawInRadians, pitchInRadians, rollInRadians;
float R11, R12, R13, R21, R22, R23, R31, R32, R33;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float initialAccMagnitude;
float kpAcc = 0.1;
float kiAcc = 0.0;
float kpMag = 0.1;
float kiMag = 0.0;
float feedbackLimit = 0.25;

uint32_t displayTimer;

void setup() {
	Serial.begin(115200);
	Serial.println(
			"Keeping the device still and level during startup will yield the best results");
	Wire.begin();
	Wire.setClock(400000);
	//todo include gyro offset cal at startup
	InitSensors();

#ifdef OUTPUT_FOR_CAL
	OutputForCalibration();
#endif//OUTPUT_FOR_CAL

	GetOffsetsAndInitialQuat();

	previousTime = micros();
	displayTimer = millis();

}
void loop() {

	currentTime = micros();
//todo tweak timing
	if (currentTime - previousTime >= tau) {
		dt = (currentTime - previousTime) * 0.000001;
		previousTime = currentTime;
		PollSensors();
		AHRS();
	}
	if (millis() - displayTimer > 250) {
		displayTimer = millis();
		GetEuler();
		Serial.print(pitchInDegrees);
		Serial.print(" , ");
		Serial.print(rollInDegrees);
		Serial.print(" , ");
		Serial.print(yawInDegrees);
		Serial.print("\r\n ");
	}
	/*PollSensors();
	 Serial.print(millis());
	 Serial.print(",");
	 Serial.print(groScaled[X_]);
	 Serial.print(",");
	 Serial.print(groScaled[Z_]);
	 Serial.print(",");
	 Serial.print(groScaled[Y_]);
	 Serial.print(",");


	 Serial.print(accScaled[X_]);
	 Serial.print(",");
	 Serial.print(accScaled[Y_]);
	 Serial.print(",");
	 Serial.print(accScaled[Z_]);
	 Serial.print(",");;
	 Serial.print(magScaled[X_]);
	 Serial.print(",");
	 Serial.print(magScaled[Y_]);
	 Serial.print(",");
	 Serial.print(magScaled[Z_]);
	 Serial.print("\r\n");
	 delay(250);*/

}


void SetGyroAccMag() {
	int32_t gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
	int32_t accSumX = 0, accSumY = 0, accSumZ = 0;
	int32_t magSumX = 0, magSumY = 0, magSumZ = 0;
	float avgX, avgY, avgZ;
	PollSensors();
	StationaryGyro();
	delay(10);
	for (uint16_t i = 0; i < NUMBER_SAMPLES_FOR_AVG; i++) {
		PollSensors();
		gyroSumX += groRead[X_];
		gyroSumY += groRead[Y_];
		gyroSumZ += groRead[Z_];
		accSumX += accScaled[X_];
		accSumY += accScaled[Y_];
		accSumZ += accScaled[Z_];
		magSumX += magScaled[X_];
		magSumY += magScaled[Y_];
		magSumZ += magScaled[Z_];
		if (StationaryGyro() == false) {
			gyroSumX = groRead[X_];
			gyroSumY = groRead[Y_];
			gyroSumZ = groRead[Z_];
			accSumX += accScaled[X_];
			accSumY += accScaled[Y_];
			accSumZ += accScaled[Z_];
			magSumX += magScaled[X_];
			magSumY += magScaled[Y_];
			magSumZ += magScaled[Z_];;
			i = 1;
		}

		delay(3);
	}
	gyroOffSets[X_] = gyroSumX / NUMBER_SAMPLES_FOR_AVG;
	gyroOffSets[Y_] = gyroSumY / NUMBER_SAMPLES_FOR_AVG;
	gyroOffSets[Z_] = gyroSumZ / NUMBER_SAMPLES_FOR_AVG;
	avgX = accSumX / NUMBER_SAMPLES_FOR_AVG;
	avgY = accSumY / NUMBER_SAMPLES_FOR_AVG;
	avgZ = accSumZ / NUMBER_SAMPLES_FOR_AVG;
	accScaled[X_] = avgX;
	accScaled[Y_] = avgY;
	accScaled[Z_] = avgZ;
	initialAccMagnitude = sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
	avgX = magSumX / NUMBER_SAMPLES_FOR_AVG;
	avgY = magSumY / NUMBER_SAMPLES_FOR_AVG;
	avgZ = magSumZ / NUMBER_SAMPLES_FOR_AVG;
	magScaled[X_] = avgX;
	magScaled[Y_] = avgY;
	magScaled[Z_] = avgZ;
}

void GetOffsetsAndInitialQuat() {
	float magnitude;
	float bx, by;
	SetGyroAccMag();
	SetVariables();
	//calculate the ypr from sensors convert to quaternion and rotation matrix
	pitchInRadians = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
	rollInRadians = atan2(acc_y, acc_z);

	yawInRadians = 0;
	q0 = cos(rollInRadians / 2.0) * cos(pitchInRadians / 2.0)* cos(yawInRadians / 2.0)- sin(rollInRadians / 2.0) * sin(pitchInRadians / 2.0)* sin(yawInRadians / 2.0);
	q1 = sin(rollInRadians / 2.0) * cos(pitchInRadians / 2.0)* cos(yawInRadians / 2.0)+ cos(rollInRadians / 2.0) * sin(pitchInRadians / 2.0)* sin(yawInRadians / 2.0);
	q2 = cos(rollInRadians / 2.0) * sin(pitchInRadians / 2.0)* cos(yawInRadians / 2.0)- sin(rollInRadians / 2.0) * cos(pitchInRadians / 2.0)* sin(yawInRadians / 2.0);
	q3 = cos(rollInRadians / 2.0) * cos(pitchInRadians / 2.0)* sin(yawInRadians / 2.0)+ sin(rollInRadians / 2.0) * sin(pitchInRadians / 2.0)* cos(yawInRadians / 2.0);
	magnitude = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 / magnitude;
	q1 = q1 / magnitude;
	q2 = q2 / magnitude;
	q3 = q3 / magnitude;

	GenerateRotationMatrix();
	GetEuler();

	bx = mag_x * cos(pitchInRadians)+ mag_y * sin(pitchInRadians) * sin(rollInRadians)+ mag_z * sin(pitchInRadians) * cos(rollInRadians);
	by = mag_z * sin(rollInRadians) - mag_y * cos(rollInRadians);
	yawInRadians = atan2(by, bx);
	q0 = cos(rollInRadians / 2.0) * cos(pitchInRadians / 2.0)* cos(yawInRadians / 2.0)- sin(rollInRadians / 2.0) * sin(pitchInRadians / 2.0)* sin(yawInRadians / 2.0);
	q1 = sin(rollInRadians / 2.0) * cos(pitchInRadians / 2.0)* cos(yawInRadians / 2.0)+ cos(rollInRadians / 2.0) * sin(pitchInRadians / 2.0)* sin(yawInRadians / 2.0);
	q2 = cos(rollInRadians / 2.0) * sin(pitchInRadians / 2.0)* cos(yawInRadians / 2.0)- sin(rollInRadians / 2.0) * cos(pitchInRadians / 2.0)* sin(yawInRadians / 2.0);
	q3 = cos(rollInRadians / 2.0) * cos(pitchInRadians / 2.0)* sin(yawInRadians / 2.0)+ sin(rollInRadians / 2.0) * sin(pitchInRadians / 2.0)* cos(yawInRadians / 2.0);
	magnitude = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 / magnitude;
	q1 = q1 / magnitude;
	q2 = q2 / magnitude;
	q3 = q3 / magnitude;

	GenerateRotationMatrix();
	GetEuler();

}
bool StationaryGyro() {
	static int16_t gyroPrevX = 0, gyroPrevY = 0, gyroPrevZ = 0;
	boolean stationary;
	if ( abs(gyroPrevX - groRead[X_]) > 100 || abs(gyroPrevY - groRead[Y_]) > 100|| abs(gyroPrevZ - groRead[Z_]) > 100) {
		stationary = false;
	} else {
		stationary = true;
	}
	gyroPrevX = groRead[X_];
	gyroPrevY = groRead[Y_];
	gyroPrevZ = groRead[Z_];
	return stationary;
}
void SetVariables() {
	acc_x = -accScaled[X_];
	acc_y = -accScaled[X_];
	acc_z = -accScaled[X_];
	mag_x = magScaled[X_];
	mag_y = magScaled[Y_];
	mag_z = magScaled[Z_];
	gro_x = groScaled[X_];
	gro_y = groScaled[Y_];
	gro_z = groScaled[Z_];
}
void GenerateRotationMatrix() {
	q0q0 = q0 * q0;
	q1q1 = q1 * q1;
	q2q2 = q2 * q2;
	q3q3 = q3 * q3;

	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;

	q1q2 = q1 * q2;
	q1q3 = q1 * q3;

	q2q3 = q2 * q3;
	R11 = 2.0 * (q0q0 - 0.5 + q1q1);
	R12 = 2.0 * (q1q2 + q0q3);
	R13 = 2.0 * (q1q3 - q0q2);
	R21 = 2.0 * (q1q2 - q0q3);
	R22 = 2.0 * (q0q0 - 0.5 + q2q2);
	R23 = 2.0 * (q2q3 + q0q1);
	R31 = 2.0 * (q1q3 + q0q2);
	R32 = 2.0 * (q2q3 - q0q1);
	R33 = 2.0 * (q0q0 - 0.5 + q3q3);

}
void GetPitch() {
	  pitchInRadians = asin(2.0 * (q0 * q2 - q3 * q1));
	  pitchInDegrees =  pitchInRadians * RAD_TO_DEG;

}

void GetRoll() {
	rollInRadians = atan2(2 * (q0 * q1 + q2 * q3),1 - 2.0 * (q1 * q1 + q2 * q2));
	rollInDegrees = rollInRadians * RAD_TO_DEG;
}

void GetYaw() {
	yawInRadians = atan2(2.0 * (q0 * q3 + q1 * q2),1 - 2.0 * (q2 * q2 + q3 * q3));
	yawInDegrees = yawInRadians * RAD_TO_DEG;

	 if (yawInDegrees < 0){
	 yawInDegrees +=360;
	 }
	 if (yawInDegrees > 360){
	 yawInDegrees -=360;
	 }
}
void GetEuler() {
	GetPitch();
	GetRoll();
	GetYaw();
}
void AHRS() {
	//the Mahoney filter
	static float integralFBX = 0, integralFBY = 0, integralFBZ = 0;
	float magnitude, recipNorm;
	float qa, qb, qc;

	float kiDTAcc, kiDTMag, dtby2;
	float bx, bz, wx, wy, wz, vx, vy, vz;

	float hx, hy, hz, exm, eym, ezm, exa, eya, eza;

	float magnitudeDifference;

	SetVariables();
	//todo add check for stationary gyro
	magnitude = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
	magnitudeDifference = fabs(initialAccMagnitude - magnitude);
	if (magnitudeDifference < feedbackLimit) {

		recipNorm = 1.0 / magnitude;
		acc_x *= recipNorm;
		acc_y *= recipNorm;
		acc_z *= recipNorm;

		recipNorm = 1.0 / sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
		mag_x *= recipNorm;
		mag_y *= recipNorm;
		mag_z *= recipNorm;

		hx = R11 * mag_x + R21 * mag_y + R31 * mag_z;
		hy = R12 * mag_x + R22 * mag_y + R32 * mag_z;
		hz = R13 * mag_x + R23 * mag_y + R33 * mag_z;

		bx = sqrt(hx * hx + hy * hy);
		bz = hz;

		wx = R11 * bx + R13 * bz;
		wy = R21 * bx + R23 * bz;
		wz = R31 * bx + R33 * bz;

		exm = (mag_y * wz - mag_z * wy);
		eym = (mag_z * wx - mag_x * wz);
		ezm = (mag_x * wy - mag_y * wx);

		vx = R13;
		vy = R23;
		vz = R33;

		exa = (acc_y * vz - acc_z * vy);
		eya = (acc_z * vx - acc_x * vz);
		eza = (acc_x * vy - acc_y * vx);

		kiDTAcc = kiAcc * dt;
		kiDTMag = kiMag * dt;
		if (kiAcc > 0) {
			integralFBX += exa * kiDTAcc + exm * kiDTMag;
			integralFBY += eya * kiDTAcc + eym * kiDTMag;
			integralFBZ += eza * kiDTAcc + ezm * kiDTMag;
			gro_x = gro_x + integralFBX;
			gro_y = gro_y + integralFBY;
			gro_z = gro_z + integralFBZ;
		} else {
			integralFBX = 0;
			integralFBY = 0;
			integralFBZ = 0;
		}
		gro_x += exa * kpAcc + exm * kpMag;
		gro_y += eya * kpAcc + eym * kpMag;
		gro_z += eza * kpAcc + ezm * kpMag;
	}
	dtby2 = dt * 0.5;
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += -1.0 * dtby2 * (gro_x * qb + gro_y * qc + gro_z * q3);
	q1 += dtby2 * (gro_x * qa - gro_y * q3 + gro_z * qc);
	q2 += dtby2 * (gro_x * q3 + gro_y * qa - gro_z * qb);
	q3 += dtby2 * (gro_y * qb - gro_x * qc + gro_z * qa);

	//normalize the quaternion
	recipNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


void InitSensors() {
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

		} else {
			if (LIS3MDLOBJ.init() == false) {
				while (1) {
					Serial.println("init failure");
					delay(500);
				}
			}
			version5 = true;
		}
	} else {
		if (LSM303OBJ.init() == false) {
			while (1) {
				Serial.println("init failure");
				delay(500);
			}

		}
	}
	Serial.println("sensors detected");
//setup device registers for ~100Hz operation
	if (version5 == true) {
		Serial.print("Device types: ");
		switch ((int) LSM6OBJ.getDeviceType()) {
		case LSM6OBJ.device_DS33:
			Serial.print("device_DS33");
			break;
		default:
			Serial.print("invalid LSM6 type");
			break;

		}
		Serial.print(" , ");
		switch ((int) LIS3MDLOBJ.getDeviceType()) {
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
		LSM6OBJ.writeReg(LSM6OBJ.CTRL1_XL, 0x48); //104Hz +/- 4g default AA filter
		LSM6OBJ.writeReg(LSM6OBJ.CTRL2_G, 0x44); //104Hz 500dps
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG1, 0xE2); //155Hz UHP fast odr XY
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG2, 0x00); //+/- 4 gauss
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG4, 0x0C); //Z axis UHP
		LIS3MDLOBJ.writeReg(LIS3MDLOBJ.CTRL_REG3, 0x00); //continuous conversion mode
		accScaleFactor[X_] = ACC_SCALE_FACTOR_D_6;
		accScaleFactor[Y_] = ACC_SCALE_FACTOR_D_6;
		accScaleFactor[Z_] = ACC_SCALE_FACTOR_D_6;
		tau = 10000;
		return;
	}

	Serial.print("Device types: ");
	switch ((int) L3GOBJ.getDeviceType()) {
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
	switch ((int) LSM303OBJ.getDeviceType()) {
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

	L3GOBJ.writeReg(L3GOBJ.CTRL_REG1, 0x0F); //100hz
	L3GOBJ.writeReg(L3GOBJ.CTRL_REG4, 0x10); //500dps
	switch ((int) LSM303OBJ.getDeviceType()) {
	case LSM303OBJ.device_DLH:
	case LSM303OBJ.device_DLM:
		LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG4_A, 0x10); //Continuous update little endian +/- 4g
		LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG1_A, 0x2F); //normal 100Hz all axes enabled
		LSM303OBJ.writeMagReg(LSM303OBJ.CRA_REG_M, 0x18); //75Hz normal mode
		LSM303OBJ.writeMagReg(LSM303OBJ.CRB_REG_M, 0xA0);
		LSM303OBJ.writeMagReg(LSM303OBJ.MR_REG_M, 0x00);
		accScaleFactor[X_] = ACC_SCALE_FACTOR_Dxx;
		accScaleFactor[Y_] = ACC_SCALE_FACTOR_Dxx;
		accScaleFactor[Z_] = ACC_SCALE_FACTOR_Dxx;
		tau = 13333;
		break;
	case LSM303OBJ.device_DLHC:
		LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG4_A, 0x10); //Continuous update little endian +/- 4g
		LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG1_A, 0x57); //normal 100Hz all axes enabled
		LSM303OBJ.writeMagReg(LSM303OBJ.CRA_REG_M, 0x98); //75Hz temp compensation enabled
		LSM303OBJ.writeMagReg(LSM303OBJ.CRB_REG_M, 0xA0);
		LSM303OBJ.writeMagReg(LSM303OBJ.MR_REG_M, 0x00);
		accScaleFactor[X_] = ACC_SCALE_FACTOR_Dxx;
		accScaleFactor[Y_] = ACC_SCALE_FACTOR_Dxx;
		accScaleFactor[Z_] = ACC_SCALE_FACTOR_Dxx;
		tau = 13333;
		break;
	case LSM303OBJ.device_D:
		//mag sensitivity is the same for XYZ
		LSM303OBJ.writeAccReg(LSM303OBJ.CTRL2, 0x08); //default AA filter +/- 4g 4 wire SPI
		LSM303OBJ.writeAccReg(LSM303OBJ.CTRL1, 0x67); //100Hz Continuous update all axes enabled
		LSM303OBJ.writeMagReg(LSM303OBJ.CTRL5, 0xF4); //100Hz high res
		LSM303OBJ.writeMagReg(LSM303OBJ.CTRL6, 0x20); // +/- 4 gauss
		LSM303OBJ.writeMagReg(LSM303OBJ.CTRL7, 0x00);
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
void PollSensors() {
	int16_t shiftedMag[3];
	if (version5 == false) {
		L3GOBJ.read();
		LSM303OBJ.readMag();
		LSM303OBJ.readAcc();
		groRead[X_] = L3GOBJ.g.x;
		groRead[Y_] = L3GOBJ.g.y;
		groRead[Z_] = L3GOBJ.g.z;

		accRead[X_] = LSM303OBJ.a.x;
		accRead[Y_] = LSM303OBJ.a.y;
		accRead[Z_] = LSM303OBJ.a.z;

		magRead[X_] = LSM303OBJ.m.x;
		magRead[Y_] = LSM303OBJ.m.y;
		magRead[Z_] = LSM303OBJ.m.z;

		if (LSM303OBJ.getDeviceType() < LSM303OBJ.device_DLHC) {
			accRead[X_] = accRead[X_] >> 4;
			accRead[Y_] = accRead[Y_] >> 4;
			accRead[Z_] = accRead[Z_] >> 4;
		}

	} else {
		LSM6OBJ.readAcc();
		LSM6OBJ.readGyro();
		LIS3MDLOBJ.read();
		groRead[X_] = LSM6OBJ.g.x;
		groRead[Y_] = LSM6OBJ.g.y;
		groRead[Z_] = LSM6OBJ.g.z;

		accRead[X_] = LSM303OBJ.a.x;
		accRead[Y_] = LSM303OBJ.a.y;
		accRead[Z_] = LSM303OBJ.a.z;

		magRead[X_] = LIS3MDLOBJ.m.x;
		magRead[Y_] = LIS3MDLOBJ.m.y;
		magRead[Z_] = LIS3MDLOBJ.m.z;

	}

	groScaled[X_] = gyroScaleFactor[X_] * (groRead[X_] - gyroOffSets[X_])* DEG_TO_RAD;
	groScaled[Y_] = gyroScaleFactor[Y_] * (groRead[Y_] - gyroOffSets[Y_])* DEG_TO_RAD;
	groScaled[Z_] = gyroScaleFactor[Z_] * (groRead[Z_] - gyroOffSets[Z_])* DEG_TO_RAD;

	accScaled[X_] = accScaleFactor[X_] * (accRead[X_] - accOffSets[X_]);
	accScaled[Y_] = accScaleFactor[Y_] * (accRead[Y_] - accOffSets[Y_]);
	accScaled[Z_] = accScaleFactor[Z_] * (accRead[Z_] - accOffSets[Z_]);

	shiftedMag[X_] = magRead[X_] - magOffSets[X_];
	shiftedMag[Y_] = magRead[Y_] - magOffSets[Y_];
	shiftedMag[Z_] = magRead[Z_] - magOffSets[Z_];
	magScaled[X_] = magScaleMatrix[X_][X_] * shiftedMag[X_]+ magScaleMatrix[X_][Y_] * shiftedMag[Y_]+ magScaleMatrix[X_][Z_] * shiftedMag[Z_];
	magScaled[Y_] = magScaleMatrix[Y_][X_] * shiftedMag[X_]+ magScaleMatrix[Y_][Y_] * shiftedMag[Y_]+ magScaleMatrix[Y_][Z_] * shiftedMag[Z_];
	magScaled[Z_] = magScaleMatrix[Z_][X_] * shiftedMag[X_]+ magScaleMatrix[Z_][Y_] * shiftedMag[Y_]+ magScaleMatrix[Z_][Z_] * shiftedMag[Z_];

}

void OutputForCalibration() {
	while (1) {
		PollSensors();
#ifdef OUTPUT_MAG_FOR_CAL
		Serial.print(magRead[X_]);
		Serial.print(" ");
		Serial.print(magRead[Z_]);
		Serial.print(" ");
		Serial.print(magRead[Y_]);
		Serial.print("\r\n");
#endif//OUTPUT_MAG_FOR_CAL
#ifdef OUTPUT_ACC_FOR_CAL
		Serial.print(millis());
		Serial.print(",");
		Serial.print(accRead[X_]);
		Serial.print(",");
		Serial.print(accRead[Y_]);
		Serial.print(",");
		Serial.print(accRead[Z_]);
		Serial.print("\r\n");
#endif//OUTPUT_ACC_FOR_CAL
		delay(100);
	}
}

