#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>


#include "rawdata.h"


#define SERIAL_DEBUG
//#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG
extern "C" {

#include <serial.h>
#include <fprintf.h>
#include <suli.h>
extern tsFILE sDebugStream;
extern tsFILE sSerStream;
}

#endif

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ADDRESS_A, BNO055_ID);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  //Serial.begin(9600);
  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
	  vfPrintf(&sSerStream, "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.delay(100);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
//  Serial.print("Current Temperature: ");
//  Serial.print(temp);
//  Serial.println(" C");
//  Serial.println("");
  vfPrintf(&sSerStream, "\n\r setup : Temperature %0d %0x", temp, temp);

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}


void init(void)
{
if(bno.init()) vfPrintf(&sSerStream, "\n\rinit ... ok");
}


/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void action(uint8_t* s, uint8_t* g, uint8_t* a, uint8_t* m)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2




	//imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

	bno.getVector2(Adafruit_BNO055::VECTOR_EULER);
	//bno.getVector2(Adafruit_BNO055::VECTOR_GYROSCOPE);

	Adafruit_BNO055::adafruit_bno055_rev_info_t info;

	bno.getRevInfo(&info);



	vfPrintf(&sSerStream, "\n\rInfo %0x %0x %0x %0x %0x", info.accel_rev, info.mag_rev, info.gyro_rev, info.sw_rev, info.bl_rev);
	byte d = bno.getVector3();
	vfPrintf(&sSerStream, "\n\rVector3 %0x", d);

  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(euler.x());
//  Serial.print(" Y: ");
//  Serial.print(euler.y());
//  Serial.print(" Z: ");
//  Serial.print(euler.z());
//  Serial.print("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  vfPrintf(&sSerStream, "\n\raction %0x %0x %0x %0x", system, gyro, accel, mag);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  Serial.print(" Mag=");
//  Serial.println(mag, DEC);
  *s = system;
  *g = gyro;
  *a = accel;
  *m = mag;

  bno.delay(BNO055_SAMPLERATE_DELAY_MS);
}
