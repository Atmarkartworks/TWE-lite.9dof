/****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* Euler.ino
* Date: 2014/09/09
* Revision: 3.0 $
*
* Usage:        Example code to stream Euler data
*
****************************************************************************
/***************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*/

/*
 * should include WProgram.h : because bring up error at the serial.h
 */
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

//#include <math.h>

#include <limits.h>
#include <stdint.h>


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

#include "NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
//#include <Wire.h>
#include "Euler.h"

NineAxesMotion mySensor;         //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))

void setup() //This code is executed once
{
  //Peripheral Initialization
//  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
//  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.

  suli_i2c_init(NULL);


  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(BNO055_OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor



}

void init(void)
{
if(mySensor.init()) vfPrintf(&sSerStream, "\n\rinit ... ok");
}

void action() //This code is looped forever
{
//  if ((millis() - lastStreamTime) >= streamPeriod)
//  {
//    lastStreamTime = millis();
    mySensor.updateEuler2();        //Update the Euler data into the structure of the object
    mySensor.updateCalibStatus();  //Update the Calibration Status

//    Serial.print("Time: ");
//    Serial.print(lastStreamTime);
//    Serial.print("ms ");


//    u16 h = (u16)mySensor.readEulerHeading(); //Heading data
//    u16 r = (u16)mySensor.readEulerRoll(); //Roll data
//    u16 p = (u16)mySensor.readEulerPitch(); //Pitch data

    s16 h = mySensor.readEulerHeading2(); //Heading data
    s16 r = mySensor.readEulerRoll2(); //Roll data
    s16 p = mySensor.readEulerPitch2(); //Pitch data



    u16 ac = mySensor.readAccelCalibStatus();  //Accelerometer Calibration Status (0 - 3)
    u16 mc = mySensor.readMagCalibStatus();    //Magnetometer Calibration Status (0 - 3)
    u16 gc = mySensor.readGyroCalibStatus();   //Gyroscope Calibration Status (0 - 3)
    u16 sc = mySensor.readSystemCalibStatus();   //System Calibration Status (0 - 3)



    vfPrintf(&sSerStream, "\n\r H:%0x deg  R:%0x deg  P:%0x deg", h, r, p);
    vfPrintf(&sSerStream, "\n\r A:%0x M:%0x G:%0x S:%0x", ac, mc, gc, sc);
    _delay(300);
//  }
}
