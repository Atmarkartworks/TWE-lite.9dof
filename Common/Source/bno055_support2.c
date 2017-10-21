/*
****************************************************************************
* Copyright (C) 2014 Bosch Sensortec GmbH
*
* bno055_support.c
* Date: 2014/12/12
* Revision: 1.0.3 $
*
* Usage: Sensor Driver support file for BNO055 sensor
*
****************************************************************************
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
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

//#include "../../../Board.h"
//#include "i2c_helper.h"


#define BNO055_API
#include "bno055.h"
#include "bno055_support.h"

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

//#include <math.h>
#include <limits.h>
#define SERIAL_DEBUG
//#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG

#include <serial.h>
#include <fprintf.h>
#include <suli.h>
extern tsFILE sDebugStream;
extern tsFILE sSerStream;

#endif

#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)


/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *	BNO055_t having the following parameters
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Burst read function pointer: BNO055_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*---------------------------------------------------------------------------*/
struct bno055_t my_bno055;
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */

s32 imu_init() //function to be called before starting the imu task.
{
	//TODO: verify correct axis alignment!

	/* Variable used to return value of
	communication routine*/
	s32 comres = ERROR;
	/*---------------------------------------------------------------------------*
	 *********************** START INITIALIZATION ************************
	 *--------------------------------------------------------------------------*/
	/*	Based on the user need configure I2C interface.
	 *	It is example code to explain how to use the bno055 API*/
		BNO055_I2C_routine();
	/*--------------------------------------------------------------------------*
	 *  This function used to assign the value/reference of
	 *	the following parameters
	 *	I2C address
	 *	Bus Write
	 *	Bus read
	 *	Chip id
	 *	Page id
	 *	Accel revision id
	 *	Mag revision id
	 *	Gyro revision id
	 *	Boot loader revision id
	 *	Software revision id
	 *-------------------------------------------------------------------------*/
		comres = bno055_init(&my_bno055);

	/*	For initializing the BNO sensor it is required to the operation mode
		of the sensor as NORMAL
		Normal mode can set from the register
		Page - page0
		register - 0x3E
		bit positions - 0 and 1*/
		/* set the power mode as NORMAL*/
		comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	/*--------------------------------------------------------------------------*
	************************* END INITIALIZATION *************************
	*---------------------------------------------------------------------------*/
	return comres;
}



s8 bno055_get_heading(double *d_euler_data_h, double *d_euler_data_p, double *d_euler_data_r)
{
	/* Variable used to return value of
	communication routine*/
	s32 comres = ERROR;
	/************************* START READ RAW FUSION DATA ********
	For reading fusion data it is required to set the
	operation modes of the sensor
	operation mode can set from the register
	page - page0
	register - 0x3D
	bit - 0 to 3
	for sensor data read following operation mode have to set
	*FUSION MODE
		*0x08 - OPERATION_MODE_IMUPLUS
		*0x09 - OPERATION_MODE_COMPASS
		*0x0A - OPERATION_MODE_M4G
		*0x0B - OPERATION_MODE_NDOF_FMC_OFF
		*0x0C - OPERATION_MODE_NDOF
		based on the user need configure the operation mode*/
	comres = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	/*	API used to read Euler data output as double  - degree and radians
		float functions also available in the BNO055 API */
	comres += bno055_convert_double_euler_h_deg(d_euler_data_h);
	comres += bno055_convert_double_euler_r_deg(d_euler_data_r);
	comres += bno055_convert_double_euler_p_deg(d_euler_data_p);

	return comres;
}

s8 bno055_get_accel(s16 *acc_x, s16 *acc_y, s16 *acc_z)
{
	/* Variable used to return value of
	communication routine*/
	s8 comres = ERROR;
	/*	For reading sensor raw data it is required to set the
		operation modes of the sensor
		operation mode can set from the register
		page - page0
		register - 0x3D
		bit - 0 to 3
		for sensor data read following operation mode have to set
		 * SENSOR MODE
			*0x01 - OPERATION_MODE_ACCONLY
			*0x02 - OPERATION_MODE_MAGONLY
			*0x03 - OPERATION_MODE_GYRONLY
			*0x04 - OPERATION_MODE_ACCMAG
			*0x05 - OPERATION_MODE_ACCGYRO
			*0x06 - OPERATION_MODE_MAGGYRO
			*0x07 - OPERATION_MODE_AMG
			based on the user need configure the operation mode*/
//		comres += bno055_set_operation_mode(OPERATION_MODE_AMG);

	/*	Raw Linear accel X, Y and Z data can read from the register
		page - page 0
		register - 0x28 to 0x2D */
//		comres += bno055_read_linear_accel_x(acc_x);
//		comres += bno055_read_linear_accel_y(acc_y);
//		comres += bno055_read_linear_accel_z(acc_z);
		comres += bno055_read_accel_x(acc_x);
		comres += bno055_read_accel_y(acc_y);
		comres += bno055_read_accel_z(acc_z);

	return comres;
}


/*!
 *	@brief This API used to read	calibration status
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
unsigned char bno055_check_calibration_status()
{
	//from the datasheet:
	/*Therefore, it is highly recommended to check the magnetometer calibration status periodically.
	If the value of the two bits ‘MAG Calib Status’ is 3, then it means that the magnetometer is fully
	calibrated and ready to go. If the value is 2, then the sensor fusion’s performance is still OK. If
	the value is 1, then it is highly recommended to perform a Figure-8 motion to calibrate the
	magnetometer. And if the value is 0, this means that the magnetometer just got disturbed by
	the magnetic interference fields nearby or the environment’s magnetic fields have just
	changed. And therefore the magnetometer calibration must be performed. For further details
	please refer section ‘3.10 Calibration’ in the datasheet.*/

	unsigned char accel_calib_status = 0;
	unsigned char gyro_calib_status = 0;
	unsigned char mag_calib_status = 0;
	unsigned char sys_calib_status = 0;
	bno055_get_accel_calib_stat(&accel_calib_status);
	bno055_get_mag_calib_stat(&mag_calib_status);
	bno055_get_gyro_calib_stat(&gyro_calib_status);
	bno055_get_sys_calib_stat(&sys_calib_status);

	return accel_calib_status+gyro_calib_status+mag_calib_status+sys_calib_status;
}




//s32 bno055_data_readout_template(void)
//{
//	/* Variable used to return value of
//	communication routine*/
//	s32 comres = ERROR;
//	/* variable used to set the power mode of the sensor*/
//	u8 power_mode = BNO055_ZERO_U8X;
//	/*********read raw accel data***********/
//	/* variable used to read the accel x data */
//	s16 accel_datax = BNO055_ZERO_U8X;
//	 /* variable used to read the accel y data */
//	s16 accel_datay = BNO055_ZERO_U8X;
//	/* variable used to read the accel z data */
//	s16 accel_dataz = BNO055_ZERO_U8X;
//	/* variable used to read the accel xyz data */
//	struct bno055_accel_t accel_xyz;
//
//	/*********read raw mag data***********/
//	/* variable used to read the mag x data */
//	s16 mag_datax  = BNO055_ZERO_U8X;
//	/* variable used to read the mag y data */
//	s16 mag_datay  = BNO055_ZERO_U8X;
//	/* variable used to read the mag z data */
//	s16 mag_dataz  = BNO055_ZERO_U8X;
//	/* structure used to read the mag xyz data */
//	struct bno055_mag_t mag_xyz;
//
//	/***********read raw gyro data***********/
//	/* variable used to read the gyro x data */
//	s16 gyro_datax = BNO055_ZERO_U8X;
//	/* variable used to read the gyro y data */
//	s16 gyro_datay = BNO055_ZERO_U8X;
//	 /* variable used to read the gyro z data */
//	s16 gyro_dataz = BNO055_ZERO_U8X;
//	 /* structure used to read the gyro xyz data */
//	struct bno055_gyro_t gyro_xyz;
//
//	/*************read raw Euler data************/
//	/* variable used to read the euler h data */
//	s16 euler_data_h = BNO055_ZERO_U8X;
//	 /* variable used to read the euler r data */
//	s16 euler_data_r = BNO055_ZERO_U8X;
//	/* variable used to read the euler p data */
//	s16 euler_data_p = BNO055_ZERO_U8X;
//	/* structure used to read the euler hrp data */
//	struct bno055_euler_t euler_hrp;
//
//	/************read raw quaternion data**************/
//	/* variable used to read the quaternion w data */
//	s16 quaternion_data_w = BNO055_ZERO_U8X;
//	/* variable used to read the quaternion x data */
//	s16 quaternion_data_x = BNO055_ZERO_U8X;
//	/* variable used to read the quaternion y data */
//	s16 quaternion_data_y = BNO055_ZERO_U8X;
//	/* variable used to read the quaternion z data */
//	s16 quaternion_data_z = BNO055_ZERO_U8X;
//	/* structure used to read the quaternion wxyz data */
//	struct bno055_quaternion_t quaternion_wxyz;
//
//	/************read raw linear acceleration data***********/
//	/* variable used to read the linear accel x data */
//	s16 linear_accel_data_x = BNO055_ZERO_U8X;
//	/* variable used to read the linear accel y data */
//	s16 linear_accel_data_y = BNO055_ZERO_U8X;
//	/* variable used to read the linear accel z data */
//	s16 linear_accel_data_z = BNO055_ZERO_U8X;
//	/* structure used to read the linear accel xyz data */
//	struct bno055_linear_accel_t linear_acce_xyz;
//
//	/*****************read raw gravity sensor data****************/
//	/* variable used to read the gravity x data */
//	s16 gravity_data_x = BNO055_ZERO_U8X;
//	/* variable used to read the gravity y data */
//	s16 gravity_data_y = BNO055_ZERO_U8X;
//	/* variable used to read the gravity z data */
//	s16 gravity_data_z = BNO055_ZERO_U8X;
//	/* structure used to read the gravity xyz data */
//	struct bno055_gravity_t gravity_xyz;
//
//	/*************read accel converted data***************/
//	/* variable used to read the accel x data output as m/s2 or mg */
//	double d_accel_datax = BNO055_ZERO_U8X;
//	/* variable used to read the accel y data output as m/s2 or mg */
//	double d_accel_datay = BNO055_ZERO_U8X;
//	/* variable used to read the accel z data output as m/s2 or mg */
//	double d_accel_dataz = BNO055_ZERO_U8X;
//	/* structure used to read the accel xyz data output as m/s2 or mg */
//	struct bno055_accel_double_t d_accel_xyz;
//
//	/******************read mag converted data********************/
//	/* variable used to read the mag x data output as uT*/
//	double d_mag_datax = BNO055_ZERO_U8X;
//	/* variable used to read the mag y data output as uT*/
//	double d_mag_datay = BNO055_ZERO_U8X;
//	/* variable used to read the mag z data output as uT*/
//	double d_mag_dataz = BNO055_ZERO_U8X;
//	/* structure used to read the mag xyz data output as uT*/
//	struct bno055_mag_double_t d_mag_xyz;
//
//	/*****************read gyro converted data************************/
//	/* variable used to read the gyro x data output as dps or rps */
//	double d_gyro_datax = BNO055_ZERO_U8X;
//	/* variable used to read the gyro y data output as dps or rps */
//	double d_gyro_datay = BNO055_ZERO_U8X;
//	/* variable used to read the gyro z data output as dps or rps */
//	double d_gyro_dataz = BNO055_ZERO_U8X;
//	/* structure used to read the gyro xyz data output as dps or rps */
//	struct bno055_gyro_double_t d_gyro_xyz;
//
//	/*******************read euler converted data*******************/
//	/* variable used to read the euler h data output as degree or radians */
//	double d_euler_data_h = BNO055_ZERO_U8X;
//	/* variable used to read the euler r data output as degree or radians */
//	double d_euler_data_r = BNO055_ZERO_U8X;
//	/* variable used to read the euler p data output as degree or radians */
//	double d_euler_data_p = BNO055_ZERO_U8X;
//	/* structure used to read the euler hrp data output as as degree or radians */
//	struct bno055_euler_double_t d_euler_hpr;
//
//	/*********************read linear acceleration converted data*************************/
//	/* variable used to read the linear accel x data output as m/s2*/
//	double d_linear_accel_datax = BNO055_ZERO_U8X;
//	/* variable used to read the linear accel y data output as m/s2*/
//	double d_linear_accel_datay = BNO055_ZERO_U8X;
//	/* variable used to read the linear accel z data output as m/s2*/
//	double d_linear_accel_dataz = BNO055_ZERO_U8X;
//	/* structure used to read the linear accel xyz data output as m/s2*/
//	struct bno055_linear_accel_double_t d_linear_accel_xyz;
//
//	/********************Gravity converted data*****************************/
//	/* variable used to read the gravity sensor x data output as m/s2*/
//	double d_gravity_data_x = BNO055_ZERO_U8X;
//	/* variable used to read the gravity sensor y data output as m/s2*/
//	double d_gravity_data_y = BNO055_ZERO_U8X;
//	/* variable used to read the gravity sensor z data output as m/s2*/
//	double d_gravity_data_z = BNO055_ZERO_U8X;
//	/* structure used to read the gravity xyz data output as m/s2*/
//	struct bno055_gravity_double_t d_gravity_xyz;
//
//
///************************* START READ RAW SENSOR DATA****************/
//
///*	Using BNO055 sensor we can read the following sensor data and
//	virtual sensor data
//	Sensor data:
//		Accel
//		Mag
//		Gyro
//	Virtual sensor data
//		Euler
//		Quaternion
//		Linear acceleration
//		Gravity sensor */
///*	For reading sensor raw data it is required to set the
//	operation modes of the sensor
//	operation mode can set from the register
//	page - page0
//	register - 0x3D
//	bit - 0 to 3
//	for sensor data read following operation mode have to set
//	 * SENSOR MODE
//		*0x01 - OPERATION_MODE_ACCONLY
//		*0x02 - OPERATION_MODE_MAGONLY
//		*0x03 - OPERATION_MODE_GYRONLY
//		*0x04 - OPERATION_MODE_ACCMAG
//		*0x05 - OPERATION_MODE_ACCGYRO
//		*0x06 - OPERATION_MODE_MAGGYRO
//		*0x07 - OPERATION_MODE_AMG
//		based on the user need configure the operation mode*/
//	comres += bno055_set_operation_mode(OPERATION_MODE_AMG);
///*	Raw accel X, Y and Z data can read from the register
//	page - page 0
//	register - 0x08 to 0x0D*/
//	comres += bno055_read_accel_x(&accel_datax);
//	comres += bno055_read_accel_y(&accel_datay);
//	comres += bno055_read_accel_z(&accel_dataz);
//	comres += bno055_read_accel_xyz(&accel_xyz);
///*	Raw mag X, Y and Z data can read from the register
//	page - page 0
//	register - 0x0E to 0x13*/
//	comres += bno055_read_mag_x(&mag_datax);
//	comres += bno055_read_mag_y(&mag_datay);
//	comres += bno055_read_mag_z(&mag_dataz);
//	comres += bno055_read_mag_xyz(&mag_xyz);
///*	Raw gyro X, Y and Z data can read from the register
//	page - page 0
//	register - 0x14 to 0x19*/
//	comres += bno055_read_gyro_x(&gyro_datax);
//	comres += bno055_read_gyro_y(&gyro_datay);
//	comres += bno055_read_gyro_z(&gyro_dataz);
//	comres += bno055_read_gyro_xyz(&gyro_xyz);
//
///************************* END READ RAW SENSOR DATA****************/
//
///************************* START READ RAW FUSION DATA ********
// 	For reading fusion data it is required to set the
//	operation modes of the sensor
//	operation mode can set from the register
//	page - page0
//	register - 0x3D
//	bit - 0 to 3
//	for sensor data read following operation mode have to set
//	*FUSION MODE
//		*0x08 - OPERATION_MODE_IMUPLUS
//		*0x09 - OPERATION_MODE_COMPASS
//		*0x0A - OPERATION_MODE_M4G
//		*0x0B - OPERATION_MODE_NDOF_FMC_OFF
//		*0x0C - OPERATION_MODE_NDOF
//		based on the user need configure the operation mode*/
//	comres += bno055_set_operation_mode(OPERATION_MODE_NDOF);
///*	Raw Euler H, R and P data can read from the register
//	page - page 0
//	register - 0x1A to 0x1E */
//	comres += bno055_read_euler_h(&euler_data_h);
//	comres += bno055_read_euler_r(&euler_data_r);
//	comres += bno055_read_euler_p(&euler_data_p);
//	comres += bno055_read_euler_hrp(&euler_hrp);
///*	Raw Quaternion W, X, Y and Z data can read from the register
//	page - page 0
//	register - 0x20 to 0x27 */
//	comres += bno055_read_quaternion_w(&quaternion_data_w);
//	comres += bno055_read_quaternion_x(&quaternion_data_x);
//	comres += bno055_read_quaternion_y(&quaternion_data_y);
//	comres += bno055_read_quaternion_z(&quaternion_data_z);
//	comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
///*	Raw Linear accel X, Y and Z data can read from the register
//	page - page 0
//	register - 0x28 to 0x2D */
//	comres += bno055_read_linear_accel_x(&linear_accel_data_x);
//	comres += bno055_read_linear_accel_y(&linear_accel_data_y);
//	comres += bno055_read_linear_accel_z(&linear_accel_data_z);
//	comres += bno055_read_linear_accel_xyz(&linear_acce_xyz);
///*	Raw Gravity sensor X, Y and Z data can read from the register
//	page - page 0
//	register - 0x2E to 0x33 */
//	comres += bno055_read_gravity_x(&gravity_data_x);
//	comres += bno055_read_gravity_y(&gravity_data_y);
//	comres += bno055_read_gravity_z(&gravity_data_z);
//	comres += bno055_read_gravity_xyz(&gravity_xyz);
///************************* END READ RAW FUSION DATA  ************/
//
///******************START READ CONVERTED SENSOR DATA****************/
///*	API used to read accel data output as double  - m/s2 and mg
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_accel_x_msq(&d_accel_datax);
//	comres += bno055_convert_double_accel_x_mg(&d_accel_datax);
//	comres += bno055_convert_double_accel_y_msq(&d_accel_datay);
//	comres += bno055_convert_double_accel_y_mg(&d_accel_datay);
//	comres += bno055_convert_double_accel_z_msq(&d_accel_dataz);
//	comres += bno055_convert_double_accel_z_mg(&d_accel_dataz);
//	comres += bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
//	comres += bno055_convert_double_accel_xyz_mg(&d_accel_xyz);
//
///*	API used to read mag data output as double  - uT(micro Tesla)
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_mag_x_uT(&d_mag_datax);
//	comres += bno055_convert_double_mag_y_uT(&d_mag_datay);
//	comres += bno055_convert_double_mag_z_uT(&d_mag_dataz);
//	comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);
//
///*	API used to read gyro data output as double  - dps and rps
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_gyro_x_dps(&d_gyro_datax);
//	comres += bno055_convert_double_gyro_y_dps(&d_gyro_datay);
//	comres += bno055_convert_double_gyro_z_dps(&d_gyro_dataz);
//	comres += bno055_convert_double_gyro_x_rps(&d_gyro_datax);
//	comres += bno055_convert_double_gyro_y_rps(&d_gyro_datay);
//	comres += bno055_convert_double_gyro_z_rps(&d_gyro_dataz);
//	comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
//	comres += bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);
//
///*	API used to read Euler data output as double  - degree and radians
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_euler_h_deg(&d_euler_data_h);
//	comres += bno055_convert_double_euler_r_deg(&d_euler_data_r);
//	comres += bno055_convert_double_euler_p_deg(&d_euler_data_p);
//	comres += bno055_convert_double_euler_h_rad(&d_euler_data_h);
//	comres += bno055_convert_double_euler_r_rad(&d_euler_data_r);
//	comres += bno055_convert_double_euler_p_rad(&d_euler_data_p);
//	comres += bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
//	comres += bno055_convert_double_euler_hpr_rad(&d_euler_hpr);
//
///*	API used to read Linear acceleration data output as m/s2
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_double_linear_accel_x_msq(&d_linear_accel_datax);
//	comres += bno055_convert_double_linear_accel_y_msq(&d_linear_accel_datay);
//	comres += bno055_convert_double_linear_accel_z_msq(&d_linear_accel_dataz);
//	comres += bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);
//
///*	API used to read Gravity sensor data output as m/s2
//	float functions also available in the BNO055 API */
//	comres += bno055_convert_gravity_double_x_msq(&d_gravity_data_x);
//	comres += bno055_convert_gravity_double_y_msq(&d_gravity_data_y);
//	comres += bno055_convert_gravity_double_z_msq(&d_gravity_data_z);
//	comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);
///*-----------------------------------------------------------------------*
//************************* START DE-INITIALIZATION ***********************
//*-------------------------------------------------------------------------*/
///*	For de - initializing the BNO sensor it is required to the operation mode
//	of the sensor as SUSPEND
//	Suspend mode can set from the register
//	Page - page0
//	register - 0x3E
//	bit positions - 0 and 1*/
//	power_mode = POWER_MODE_SUSPEND; /* set the power mode as SUSPEND*/
//	comres += bno055_set_power_mode(power_mode);
//
///*---------------------------------------------------------------------*
//************************* END DE-INITIALIZATION **********************
//*---------------------------------------------------------------------*/
//return comres;
//}

/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bno055_t
*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
 s8 BNO055_I2C_routine(void) {

	my_bno055.bus_write = BNO055_I2C_bus_write;
	my_bno055.bus_read = BNO055_I2C_bus_read;
	my_bno055.delay_msec = BNO055_delay_msek;
	my_bno055.dev_addr = BNO055_I2C_ADDR1; //TODO: integarte this in the API funcition in order to chose between weather strip and mainboard sensor.

	//return BNO055_ZERO_U8X;
	return 0;
}

/************** I2C buffer length******/

#define	I2C_BUFFER_LEN 8
#define I2C0 5
 /*-------------------------------------------------------------------*
 *
 *	This is a sample code for read and write the data by using I2C
 *	Use either I2C  based on your need
 *	The device address defined in the bno055.h file
 *
 *--------------------------------------------------------------------*/

 /*	\Brief: The API is used as I2C bus write
  *	\Return : Status of the I2C write
  *	\param dev_addr : The device address of the sensor
  *	\param reg_addr : Address of the first register,
  *   will data is going to be written
  *	\param reg_data : It is a value hold in the array,
  *		will be used for write the value into the register
  *	\param cnt : The no of byte of data to be write
  */
 s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
 {
 	s32 BNO055_iERROR = BNO055_INIT_VALUE;
 	u8 array[I2C_BUFFER_LEN];
 	u8 stringpos = BNO055_INIT_VALUE;
 	bool ret;

 	array[BNO055_INIT_VALUE] = reg_addr;
 	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++) {
 		array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
 			*(reg_data + stringpos);
 	}
 	/*
 	* Please take the below APIs as your reference for
 	* write the data using I2C communication
 	* "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
 	* add your I2C write APIs here
 	* BNO055_iERROR is an return value of I2C read API
 	* Please select your valid return value
 	* In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
 	* Note :
 	* This is a full duplex operation,
 	* The first read data is discarded, for that extra write operation
 	* have to be initiated. For that cnt+1 operation done
 	* in the I2C write string function
 	* For more information please refer data sheet SPI communication:
 	*/
 //	return (s8)BNO055_iERROR;


 	  ret = suli_i2c_write(NULL, (uint8)dev_addr, (uint8 *)array, cnt+1);
 	  //vfPrintf(&sSerStream, "\n\rwrite8 : (%02X %02X)", reg, value);

 	  return (s8)ret;
 }

  /*	\Brief: The API is used as I2C bus read
  *	\Return : Status of the I2C read
  *	\param dev_addr : The device address of the sensor
  *	\param reg_addr : Address of the first register,
  *  will data is going to be read
  *	\param reg_data : This data read from the sensor,
  *   which is hold in an array
  *	\param cnt : The no of byte of data to be read
  */
 s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
 {
 //	s32 BNO055_iERROR = BNO055_INIT_VALUE;
 //	u8 array[I2C_BUFFER_LEN] = {BNO055_INIT_VALUE};
 //	u8 stringpos = BNO055_INIT_VALUE;
 //
 //	array[BNO055_INIT_VALUE] = reg_addr;
 //
 //	/* Please take the below API as your reference
 //	 * for read the data using I2C communication
 //	 * add your I2C read API here.
 //	 * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
 //	 * ARRAY, ARRAY, 1, CNT)"
 //	 * BNO055_iERROR is an return value of SPI write API
 //	 * Please select your valid return value
 //     * In the driver BNO055_SUCCESS defined as 0
 //     * and FAILURE defined as -1
 //	 */
 //	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
 //		*(reg_data + stringpos) = array[stringpos];
 //	return (s8)BNO055_iERROR;





 	  bool ret = 0;
 	  uint8 dta_send[] = {reg_addr};


 	  ret = suli_i2c_write(NULL, dev_addr, dta_send, 1);
 	  ret = suli_i2c_read(NULL, dev_addr, reg_data, cnt);

 	  return (s8)ret;

 }
 /*	Brief : The delay routine
  *	\param : delay in ms
 */
 void BNO055_delay_msek(u32 msek)
 {
 	/*Here you can write your own delay routine*/
 	vWait(msek);
 }













 /****************************************************************************
 * Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
 *
 * NineAxesMotion.cpp
 * Date: 2015/02/10
 * Revision: 3.0 $
 *
 * Usage:        Source file of the C++ Wrapper for the BNO055 Sensor API
 *
 ****************************************************************************
 *
 * Added Arduino M0/M0 Pro support
 *
 * Date: 07/27/2015
 *
 * Modified by: Arduino.org development Team.
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
 #include "NineAxesMotion.h"
 //Function Definitions
 /*******************************************************************************************
 *Description: Constructor of the class with the default initialization
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 NineAxesMotion_NineAxesMotion()
 {
 	//Blank
 }

 /*******************************************************************************************
 *Description: Function with the bare minimum initialization
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_initSensor(unsigned int address)
 {
 	//Initialize the GPIO peripheral
 	pinMode(INT_PIN, INPUT_PULLUP);		//Configure Interrupt pin
 	pinMode(RESET_PIN, OUTPUT);			//Configure Reset pin

 	//Power on the BNO055
 	resetSensor(address);
 }

 /*******************************************************************************************
 *Description: This function is used to reset the BNO055
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_resetSensor(unsigned int address)
 {
 	//Reset sequence
 	digitalWrite(RESET_PIN, LOW);		//Set the Reset pin LOW
 	delay(RESET_PERIOD);				//Hold it for a while
 	digitalWrite(RESET_PIN, HIGH);		//Set the Reset pin HIGH
 	delay(INIT_PERIOD);					//Pause for a while to let the sensor initialize completely (Anything >500ms should be fine)
 	//Initialization sequence
 	//Link the function pointers for communication (late-binding)
 	myBNO.bus_read = BNO055_I2C_bus_read;
 	myBNO.bus_write = BNO055_I2C_bus_write;
 	myBNO.delay_msec = _delay;

 	//Set the I2C address here !!! ADDR1 is the default address
 	//myBNO.dev_addr = BNO055_I2C_ADDR1;
 	myBNO.dev_addr = address;
 	//myBNO.dev_addr = BNO055_I2C_ADDR2;

 	//Initialize the BNO055 structure to hold the device information
 	bno055_init(&myBNO);

 	//Post initialization delay
 	delay(POST_INIT_PERIOD);

 	//To set the output data format to the Android style
 	bno055_set_data_output_format(ANDROID);

 	//Set the default data update mode to auto
 	dataUpdateMode = AUTO;
 }

 /*******************************************************************************************
 *Description: This function is used to set the operation mode of the BNO055
 *Input Parameters:
 *	byte operationMode: To assign which operation mode the device has to
 *				---------------------------------------------------
 *				Constant Definition			Constant Value	Comment
 *				---------------------------------------------------
 *				OPERATION_MODE_CONFIG		0x00			Configuration Mode
 *																(Transient Mode)
 *				OPERATION_MODE_ACCONLY		0x01			Accelerometer only
 *				OPERATION_MODE_MAGONLY		0x02			Magnetometer only
 *				OPERATION_MODE_GYRONLY		0x03			Gyroscope only
 *				OPERATION_MODE_ACCMAG		0x04			Accelerometer and Magnetometer only
 *				OPERATION_MODE_ACCGYRO		0x05			Accelerometer and Gyroscope only
 *				OPERATION_MODE_MAGGYRO		0x06			Magnetometer and Gyroscope only
 *				OPERATION_MODE_AMG			0x07			Accelerometer, Magnetometer and
 *																Gyroscope (without fusion)
 *				OPERATION_MODE_IMUPLUS		0x08			Inertial Measurement Unit
 *																(Accelerometer and Gyroscope
 *																	Sensor Fusion Mode)
 *				OPERATION_MODE_COMPASS		0x09			Tilt Compensated Compass
 *																(Accelerometer and Magnetometer
 *																	Sensor Fusion Mode)
 *				OPERATION_MODE_M4G			0x0A			Magnetometer and Gyroscope Sensor
 *																Fusion Mode
 *				OPERATION_MODE_NDOF_FMC_OFF	0x0B			9 Degrees of Freedom Sensor Fusion
 *																with Fast Magnetometer Calibration Off
 *				OPERATION_MODE_NDOF			0x0C			9 Degrees of Freedom Sensor Fusion
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_setOperationMode(byte operationMode)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_operation_mode(operationMode);			//Set the Operation Mode
 }

 /*******************************************************************************************
 *Description: This function is used to set the power mode
 *Input Parameters:
 *	byte powerMode: To assign the power mode the device has to switch to
 *				--------------------------------------
 *				Constant Definition		Constant Value
 *				--------------------------------------
 *				POWER_MODE_NORMAL		0x00
 *				POWER_MODE_LOWPOWER		0x01
 *				POWER_MODE_SUSPEND		0x02
 *Return Parameter:
 *******************************************************************************************/
 void NineAxesMotion_setPowerMode(byte powerMode)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_power_mode(powerMode);					//Set the Power Mode
 }

 /*******************************************************************************************
 *Description: This function is used to update the accelerometer data in m/s2
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateAccel(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_convert_float_accel_xyz_msq(&accelData);	//Read the data from the sensor
 }

 /*******************************************************************************************
 *Description: This function is used to update the magnetometer data in microTesla
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateMag(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_convert_float_mag_xyz_uT(&magData);	//Read the data from the sensor
 }

 /*******************************************************************************************
 *Description: This function is used to update the gyroscope data in deg/s
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateGyro(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_convert_float_gyro_xyz_dps(&gyroData);		//Read the data from the sensor
 }

 /*******************************************************************************************
 *Description: This function is used to update the quaternion data
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateQuat(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_read_quaternion_wxyz(&quatData);			//Read the data from the sensor
 }

 /*******************************************************************************************
 *Description: This function is used to update the euler data in degrees
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateEuler(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_convert_float_euler_hpr_deg(&eulerData);	//Read the data from the sensor
 }


 /*******************************************************************************************
 *Description: This function is used to update the linear acceleration data in m/s2
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateLinearAccel(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_convert_float_linear_accel_xyz_msq(&linearAccelData);	//Read the data from the sensor
 }

 /*******************************************************************************************
 *Description: This function is used to update the gravity acceleration data in m/s2
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateGravAccel(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_convert_float_gravity_xyz_msq(&gravAccelData);	//Read the data from the sensor
 }

 /*******************************************************************************************
 *Description: This function is used to update the calibration status
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateCalibStatus(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_get_accel_calib_stat(&calibStatus.accel);
 	comRes = bno055_get_mag_calib_stat(&calibStatus.mag);
 	comRes = bno055_get_gyro_calib_stat(&calibStatus.gyro);
 	comRes = bno055_get_sys_calib_stat(&calibStatus.system);
 }

 /*******************************************************************************************
 *Description: This function is used to write the accelerometer configurations
 *Input Parameters:
 *	uint8_t range: To assign the range of the accelerometer
 *			--------------------------------------
 *			Constant Definition		Constant Value
 *			--------------------------------------
 *			ACCEL_RANGE_2G			0X00
 *			ACCEL_RANGE_4G			0X01
 *			ACCEL_RANGE_8G			0X02
 *			ACCEL_RANGE_16G			0X03
 *	uint8_t bandwidth: To assign the filter bandwidth of the accelerometer
 *			--------------------------------------
 *			Constant Definition		Constant Value
 *			--------------------------------------
 *			ACCEL_BW_7_81HZ			0x00
 *			ACCEL_BW_15_63HZ		0x01
 *			ACCEL_BW_31_25HZ		0x02
 *			ACCEL_BW_62_5HZ			0X03
 *			ACCEL_BW_125HZ			0X04
 *			ACCEL_BW_250HZ			0X05
 *			ACCEL_BW_500HZ			0X06
 *			ACCEL_BW_1000HZ			0X07
 *	uint8_t powerMode: To assign the power mode of the accelerometer
 *			--------------------------------------
 *			Constant Definition		Constant Value
 *			--------------------------------------
 *			ACCEL_NORMAL			0X00
 *			ACCEL_SUSPEND			0X01
 *			ACCEL_LOWPOWER_1		0X02
 *			ACCEL_STANDBY			0X03
 *			ACCEL_LOWPOWER_2		0X04
 *			ACCEL_DEEPSUSPEND		0X05
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_writeAccelConfig(uint8_t range, uint8_t bandwidth, uint8_t powerMode)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_accel_range(range);
 	comRes = bno055_set_accel_bw(bandwidth);
 	comRes = bno055_set_accel_power_mode(powerMode);
 }

 /*******************************************************************************************
 *Description: This function is used to update the accelerometer configurations
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_updateAccelConfig(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_get_accel_range(&accelStatus.range);
 	comRes = bno055_get_accel_bw(&accelStatus.bandwidth);
 	comRes = bno055_get_accel_power_mode(&accelStatus.powerMode);
 }

 /*******************************************************************************************
 *Description: This function is used to control which axis of the accelerometer triggers the
 *				interrupt
 *Input Parameters:
 *	bool xStatus: To know whether the x axis has to trigger the interrupt
 *				---------------------------------------------------
 *				Constant Definition		Constant Value	Comment
 *				---------------------------------------------------
 *				ENABLE					1				Enables interrupts from that axis
 *				DISABLE					0				Disables interrupts from that axis
 *	bool yStatus: To know whether the x axis has to trigger the interrupt
 *				---------------------------------------------------
 *				Constant Definition		Constant Value	Comment
 *				---------------------------------------------------
 *				ENABLE					1				Enables interrupts from that axis
 *				DISABLE					0				Disables interrupts from that axis
 *	bool zStatus: To know whether the x axis has to trigger the interrupt
 *				---------------------------------------------------
 *				Constant Definition		Constant Value	Comment
 *				---------------------------------------------------
 *				ENABLE					1				Enables interrupts from that axis
 *				DISABLE					0				Disables interrupts from that axis
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_accelInterrupts(bool xStatus, bool yStatus, bool zStatus)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_accel_any_motion_no_motion_axis_enable(BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS, xStatus);
 	comRes = bno055_set_accel_any_motion_no_motion_axis_enable(BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS, yStatus);
 	comRes = bno055_set_accel_any_motion_no_motion_axis_enable(BNO055_ACCEL_ANY_MOTION_NO_MOTION_Z_AXIS, zStatus);
 }

 /*******************************************************************************************
 *Description: This function is used to reset the interrupt line
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_resetInterrupt(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_intr_rst(ENABLE);
 }

 /*******************************************************************************************
 *Description: This function is used to enable the any motion interrupt based on the
 *				accelerometer
 *Input Parameters:
 *	uint8_t threshold: The threshold that triggers the any motion interrupt
 *				The threshold should be entered as an integer. The corresponding value of
 *					the threshold depends on the range that has been set on the
 *					accelerometer. Below is a table showing the value of 1LSB in
 *					corresponding units.
 *				Resolution:
 *					ACCEL_RANGE_2G, 1LSB = 3.91mg = ~0.03835m/s2
 *					ACCEL_RANGE_4G, 1LSB = 7.81mg = ~0.07661m/s2
 *					ACCEL_RANGE_8G, 1LSB = 15.6mg = ~0.15303m/s2
 *					ACCEL_RANGE_16G, 1LSB = 31.3mg = ~0.30705m/s2
 *				Maximum:
 *					ACCEL_RANGE_2G, 1LSB = 996mg = ~9.77076m/s2,
 *					ACCEL_RANGE_4G, 1LSB = 1.99g = ~19.5219m/s2
 *					ACCEL_RANGE_8G, 1LSB = 3.98g = ~39.0438m/s2
 *					ACCEL_RANGE_16G, 1LSB = 7.97g = ~97.1857m/s2
 *	uint8_t duration: The duration for which the desired threshold exist
 *				The time difference between the successive acceleration signals depends
 *				on the selected bandwidth and equates to 1/(2*bandwidth).
 *				In order to suppress false triggers, the interrupt is only generated (cleared)
 *				if a certain number N of consecutive slope data points is larger (smaller)
 *				than the slope 'threshold'. This number is set by the 'duration'.
 *				It is N = duration + 1.
 *				Resolution:
 *					ACCEL_BW_7_81HZ, 1LSB = 64ms
 *					ACCEL_BW_15_63HZ, 1LSB = 32ms
 *					ACCEL_BW_31_25HZ, 1LSB = 16ms
 *					ACCEL_BW_62_5HZ, 1LSB = 8ms
 *					ACCEL_BW_125HZ, 1LSB = 4ms
 *					ACCEL_BW_250HZ, 1LSB = 2ms
 *					ACCEL_BW_500HZ, 1LSB = 1ms
 *					ACCEL_BW_1000HZ, 1LSB = 0.5ms
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_enableAnyMotion(uint8_t threshold, uint8_t duration)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_accel_any_motion_thres(threshold);
 	comRes = bno055_set_accel_any_motion_durn(duration);
 	comRes = bno055_set_intr_accel_any_motion(ENABLE);
 	comRes = bno055_set_intr_mask_accel_any_motion(ENABLE);
 }

 /*******************************************************************************************
 *Description: This function is used to disable the any motion interrupt
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_disableAnyMotion(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_intr_accel_any_motion(DISABLE);
 	comRes = bno055_set_intr_mask_accel_any_motion(DISABLE);
 }

 /*******************************************************************************************
 *Description: This function is used to enable the slow or no motion interrupt based on the
 *				accelerometer
 *Input Parameters:
 *	uint8_t threshold: The threshold that triggers the no motion interrupt
 *				The threshold should be entered as an integer. The corresponding value of
 *					the threshold depends on the range that has been set on the
 *					accelerometer. Below is a table showing the value of 1LSB in
 *					corresponding units.
 *				Resolution:
 *					ACCEL_RANGE_2G, 1LSB = 3.91mg = ~0.03835m/s2
 *					ACCEL_RANGE_4G, 1LSB = 7.81mg = ~0.07661m/s2
 *					ACCEL_RANGE_8G, 1LSB = 15.6mg = ~0.15303m/s2
 *					ACCEL_RANGE_16G, 1LSB = 31.3mg = ~0.30705m/s2
 *				Maximum:
 *					ACCEL_RANGE_2G, 1LSB = 996mg = ~9.77076m/s2,
 *					ACCEL_RANGE_4G, 1LSB = 1.99g = ~19.5219m/s2
 *					ACCEL_RANGE_8G, 1LSB = 3.98g = ~39.0438m/s2
 *					ACCEL_RANGE_16G, 1LSB = 7.97g = ~97.1857m/s2
 *	uint8_t duration: The duration for which the desired threshold should be surpassed
 *				The time difference between the successive acceleration signals depends
 *				on the selected bandwidth and equates to 1/(2*bandwidth).
 *				In order to suppress false triggers, the interrupt is only generated (cleared)
 *				if a certain number N of consecutive slope data points is larger (smaller)
 *				than the slope 'threshold'. This number is set by the 'duration'.
 *				It is N = duration + 1.
 *				Resolution:
 *					ACCEL_BW_7_81HZ, 1LSB = 64ms
 *					ACCEL_BW_15_63HZ, 1LSB = 32ms
 *					ACCEL_BW_31_25HZ, 1LSB = 16ms
 *					ACCEL_BW_62_5HZ, 1LSB = 8ms
 *					ACCEL_BW_125HZ, 1LSB = 4ms
 *					ACCEL_BW_250HZ, 1LSB = 2ms
 *					ACCEL_BW_500HZ, 1LSB = 1ms
 *					ACCEL_BW_1000HZ, 1LSB = 0.5ms
 *	bool motion: To trigger either a Slow motion or a No motion interrupt
 *				---------------------------------------------------
 *				Constant Definition		Constant Value	Comment
 *				---------------------------------------------------
 *				NO_MOTION				1				Enables the no motion interrupt
 *				SLOW_MOTION				0				Enables the slow motion interrupt
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_enableSlowNoMotion(uint8_t threshold, uint8_t duration, bool motion)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_accel_slow_no_motion_enable(motion);
 	comRes = bno055_set_accel_slow_no_motion_thres(threshold);
 	comRes = bno055_set_accel_slow_no_motion_durn(duration);
 	comRes = bno055_set_intr_accel_no_motion(ENABLE);
 	comRes = bno055_set_intr_mask_accel_no_motion(ENABLE);
 }

 /*******************************************************************************************
 *Description: This function is used to disable the slow or no motion interrupt
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_disableSlowNoMotion(void)
 {
 	BNO055_RETURN_FUNCTION_TYPE comRes = BNO055_ZERO_U8X;		//Holds the communication results
 	comRes = bno055_set_intr_accel_no_motion(DISABLE);
 	comRes = bno055_set_intr_mask_accel_any_motion(DISABLE);
 }

 /*******************************************************************************************
 *Description: This function is used to change the mode of updating the local data
 *Input Parameters: None
 *Return Parameter: None
 *******************************************************************************************/
 void NineAxesMotion_setUpdateMode(bool updateMode)
 {
 	dataUpdateMode = updateMode;
 }

 /*******************************************************************************************
 *Description: This function is used to return the x-axis of the accelerometer data
 *Input Parameters: None
 *Return Parameter:
 *	float:	X-axis accelerometer data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readAccelX(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccel();
 	}
 	return accelData.x;
 }

 /*******************************************************************************************
 *Description: This function is used to return the y-axis of the accelerometer data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Y-axis accelerometer data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readAccelY(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccel();
 	}
 	return accelData.y;
 }

 /*******************************************************************************************
 *Description: This function is used to return the z-axis of the accelerometer data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Z-axis accelerometer data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readAccelZ(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccel();
 	}
 	return accelData.z;
 }

 /*******************************************************************************************
 *Description: This function is used to return the x-axis of the gyroscope data
 *Input Parameters: None
 *Return Parameter:
 *	float:	X-axis gyroscope data in deg/s
 *******************************************************************************************/
 float NineAxesMotion_readGyroX(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGyro();
 	}
 	return gyroData.x;
 }

 /*******************************************************************************************
 *Description: This function is used to return the y-axis of the gyroscope data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Y-axis gyroscope data in deg/s
 *******************************************************************************************/
 float NineAxesMotion_readGyroY(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGyro();
 	}
 	return gyroData.y;
 }

 /*******************************************************************************************
 *Description: This function is used to return the z-axis of the gyroscope data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Z-axis gyroscope data in deg/s
 *******************************************************************************************/
 float NineAxesMotion_readGyroZ(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGyro();
 	}
 	return gyroData.z;
 }

 /*******************************************************************************************
 *Description: This function is used to return the x-axis of the magnetometer data
 *Input Parameters: None
 *Return Parameter:
 *	float:	X-axis magnetometer data in �T
 *******************************************************************************************/
 float NineAxesMotion_readMagX(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateMag();
 	}
 	return magData.x;
 }

 /*******************************************************************************************
 *Description: This function is used to return the y-axis of the magnetometer data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Y-axis magnetometer data in �T
 *******************************************************************************************/
 float NineAxesMotion_readMagY(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateMag();
 	}
 	return magData.y;
 }

 /*******************************************************************************************
 *Description: This function is used to return the z-axis of the magnetometer data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Z-axis magnetometer data in �T
 *******************************************************************************************/
 float NineAxesMotion_readMagZ(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateMag();
 	}
 	return magData.z;
 }

 /*******************************************************************************************
 *Description: This function is used to return the w-axis of the quaternion data
 *Input Parameters: None
 *Return Parameter:
 *	int16_t:	W-axis quaternion data multiplied by 1000 (for 3 decimal places accuracy)
 *******************************************************************************************/
 int16_t NineAxesMotion_readQuatW(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateQuat();
 	}
 	return quatData.w;
 }

 /*******************************************************************************************
 *Description: This function is used to return the x-axis of the quaternion data
 *Input Parameters: None
 *Return Parameter:
 *	int16_t:	X-axis quaternion data multiplied by 1000 (for 3 decimal places accuracy)
 *******************************************************************************************/
 int16_t NineAxesMotion_readQuatX(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateQuat();
 	}
 	return quatData.x;
 }

 /*******************************************************************************************
 *Description: This function is used to return the y-axis of the quaternion data
 *Input Parameters: None
 *Return Parameter:
 *	int16_t:	Y-axis quaternion data multiplied by 1000 (for 3 decimal places accuracy)
 *******************************************************************************************/
 int16_t NineAxesMotion_readQuatY(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateQuat();
 	}
 	return quatData.y;
 }

 /*******************************************************************************************
 *Description: This function is used to return the z-axis of the quaternion data
 *Input Parameters: None
 *Return Parameter:
 *	int16_t:	Z-axis quaternion data multiplied by 1000 (for 3 decimal places accuracy)
 *******************************************************************************************/
 int16_t NineAxesMotion_readQuatZ(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateQuat();
 	}
 	return quatData.z;
 }

 /*******************************************************************************************
 *Description: This function is used to return the heading(yaw) of the euler data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Heading of the euler data
 *******************************************************************************************/
 float NineAxesMotion_readEulerHeading(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateEuler();
 	}
 	return eulerData.h;
 }

 /*******************************************************************************************
 *Description: This function is used to return the roll of the euler data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Roll of the euler data
 *******************************************************************************************/
 float NineAxesMotion_readEulerRoll(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateEuler();
 	}
 	return eulerData.r;
 }

 /*******************************************************************************************
 *Description: This function is used to return the pitch of the euler data
 *Input Parameters: None
 *Return Parameter:
 *	float:	Pitch of the euler data
 *******************************************************************************************/
 float NineAxesMotion_readEulerPitch(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateEuler();
 	}
 	return eulerData.p;
 }

 /*******************************************************************************************
 *Description: This function is used to return the x-axis of the linear acceleration data
 *					(accelerometer data without the gravity vector)
 *Input Parameters: None
 *Return Parameter:
 *	float:	X-axis Linear Acceleration data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readLinearAccelX(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateLinearAccel();
 	}
 	return linearAccelData.x;
 }

 /*******************************************************************************************
 *Description: This function is used to return the y-axis of the linear acceleration data
 *					(accelerometer data without the gravity vector)
 *Input Parameters: None
 *Return Parameter:
 *	float:	Y-axis Linear Acceleration data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readLinearAccelY(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateLinearAccel();
 	}
 	return linearAccelData.y;
 }

 /*******************************************************************************************
 *Description: This function is used to return the z-axis of the linear acceleration data
 *					(accelerometer data without the gravity vector)
 *Input Parameters: None
 *Return Parameter:
 *	float:	Z-axis Linear Acceleration data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readLinearAccelZ(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateLinearAccel();
 	}
 	return linearAccelData.z;
 }

 /*******************************************************************************************
 *Description: This function is used to return the x-axis of the gravity acceleration data
 *					(accelerometer data with only the gravity vector)
 *Input Parameters: None
 *Return Parameter:
 *	float:	X-axis Gravity Acceleration data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readGravAccelX(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGravAccel();
 	}
 	return gravAccelData.x;
 }

 /*******************************************************************************************
 *Description: This function is used to return the y-axis of the gravity acceleration data
 *					(accelerometer data with only the gravity vector)
 *Input Parameters: None
 *Return Parameter:
 *	float:	Y-axis Gravity Acceleration data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readGravAccelY(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGravAccel();
 	}
 	return gravAccelData.y;
 }

 /*******************************************************************************************
 *Description: This function is used to return the z-axis of the gravity acceleration data
 *					(accelerometer data with only the gravity vector)
 *Input Parameters: None
 *Return Parameter:
 *	float:	Z-axis Gravity Acceleration data in m/s2
 *******************************************************************************************/
 float NineAxesMotion_readGravAccelZ(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGravAccel();
 	}
 	return gravAccelData.z;
 }

 /*******************************************************************************************
 *Description: This function is used to return the accelerometer calibration status
 *Input Parameters: None
 *Return Parameter:
 *	uint8_t:	Accelerometer calibration status, 0-3 (0 - low, 3 - high)
 *******************************************************************************************/
 uint8_t NineAxesMotion_readAccelCalibStatus(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateCalibStatus();
 	}
 	return calibStatus.accel;
 }

 /*******************************************************************************************
 *Description: This function is used to return the gyroscope calibration status
 *Input Parameters: None
 *Return Parameter:
 *	uint8_t:	Gyroscope calibration status, 0-3 (0 - low, 3 - high)
 *******************************************************************************************/
 uint8_t NineAxesMotion_readGyroCalibStatus(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateCalibStatus();
 	}
 	return calibStatus.gyro;
 }

 /*******************************************************************************************
 *Description: This function is used to return the magnetometer calibration status
 *Input Parameters: None
 *Return Parameter:
 *	uint8_t:	Magnetometer calibration status, 0-3 (0 - low, 3 - high)
 *******************************************************************************************/
 uint8_t NineAxesMotion_readMagCalibStatus(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateCalibStatus();
 	}
 	return calibStatus.mag;
 }

 /*******************************************************************************************
 *Description: This function is used to return the system calibration status
 *Input Parameters: None
 *Return Parameter:
 *	uint8_t:	System calibration status, 0-3 (0 - low, 3 - high)
 *******************************************************************************************/
 uint8_t NineAxesMotion_readSystemCalibStatus(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateCalibStatus();
 	}
 	return calibStatus.system;
 }

 /*******************************************************************************************
 *Description: This function is used to return the accelerometer range
 *Input Parameters: None
 *Return Parameter:
 *	uint8_t range: Range of the accelerometer
 *			--------------------------------------
 *			Constant Definition		Constant Value
 *			--------------------------------------
 *			ACCEL_RANGE_2G			0X00
 *			ACCEL_RANGE_4G			0X01
 *			ACCEL_RANGE_8G			0X02
 *			ACCEL_RANGE_16G			0X03
 *******************************************************************************************/
 uint8_t NineAxesMotion_readAccelRange(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccelConfig();
 	}
 	return accelStatus.range;
 }

 /*******************************************************************************************
 *Description: This function is used to return the accelerometer bandwidth
 *Input Parameters: None
 *Return Parameter:
 *	uint8_t bandwidth: Bandwidth of the accelerometer
 *			--------------------------------------
 *			Constant Definition		Constant Value
 *			--------------------------------------
 *			ACCEL_BW_7_81HZ			0x00
 *			ACCEL_BW_15_63HZ		0x01
 *			ACCEL_BW_31_25HZ		0x02
 *			ACCEL_BW_62_5HZ			0X03
 *			ACCEL_BW_125HZ			0X04
 *			ACCEL_BW_250HZ			0X05
 *			ACCEL_BW_500HZ			0X06
 *			ACCEL_BW_1000HZ			0X07
 *******************************************************************************************/
 uint8_t NineAxesMotion_readAccelBandwidth(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccelConfig();
 	}
 	return accelStatus.bandwidth;
 }

 /*******************************************************************************************
 *Description: This function is used to return the accelerometer power mode
 *Input Parameters: None
 *Return Parameter:
 *	uint8_t powerMode: Power mode of the accelerometer
 *			--------------------------------------
 *			Constant Definition		Constant Value
 *			--------------------------------------
 *			ACCEL_NORMAL			0X00
 *			ACCEL_SUSPEND			0X01
 *			ACCEL_LOWPOWER_1		0X02
 *			ACCEL_STANDBY			0X03
 *			ACCEL_LOWPOWER_2		0X04
 *			ACCEL_DEEPSUSPEND		0X05
 *******************************************************************************************/
 uint8_t NineAxesMotion_readAccelPowerMode(void)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccelConfig();
 	}
 	return accelStatus.powerMode;
 }


 /******************** Bridge Functions for the Sensor API to control the Arduino Hardware******************************************/
 signed char BNO055_I2C_bus_read(unsigned char dev_addr,unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
 {
 	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
 	I2C.beginTransmission(dev_addr);	//Start of transmission
 	I2C.write(reg_addr);				//Desired start register
 	comres = I2C.endTransmission();		//Stop of transmission
 	delayMicroseconds(150);				//Caution Delay
 	I2C.requestFrom(dev_addr, cnt);		//Request data
 	while(I2C.available())				//The slave device may send less than requested (burst read)
 	{
 		*reg_data = I2C.read();			//Receive a byte
 		reg_data++;						//Increment pointer
 	}
 	return comres;
 }


 signed char BNO055_I2C_bus_write(unsigned char dev_addr,unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
 {
 	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
 	I2C.beginTransmission(dev_addr);	//Start of transmission
 	I2C.write(reg_addr);				//Desired start register
 	for(unsigned char index = 0; index < cnt; index++) //Note that the BNO055 supports burst write
 	{
 		I2C.write(*reg_data);			//Write the data
 		reg_data++;						//Increment pointer
 	}
 	comres = I2C.endTransmission();		//Stop of transmission
 	delayMicroseconds(150);				//Caution Delay
 	return comres;
 }

 void _delay(u_32 period)
 {
 	delay(period);
 }


 void NineAxesMotion_begin(unsigned int address = 0x28)
 {
 	initSensor(address);
 }

 void NineAxesMotion_end()
 {

 }

 //accelerometer
 void NineAxesMotion_readAccelerometer(float& x, float& y, float& z)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccel();
 	}
 	x = accelData.x;
 	y = accelData.y;
 	z = accelData.z;
 }

 float NineAxesMotion_readAccelerometer(int axis)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccel();
 	}
 	if (axis == X_AXIS) {
 			return accelData.x;
 	} else if (axis == Y_AXIS) {
 			return accelData.y;
 	} else if (axis == Z_AXIS) {
 			return accelData.z;
 	}
 }

 void NineAxesMotion_readAccel(float& x, float& y, float& z)
 {
 	readAccelerometer(x, y, z);
 }

 float NineAxesMotion_readAccel(int axis)
 {
 	return readAccelerometer(axis);
 }

 // Grav. Acceleration
 void NineAxesMotion_readGravAcceleration(float& x, float& y, float& z)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGravAccel();
 	}
 	x = gravAccelData.x;
 	y = gravAccelData.y;
 	z = gravAccelData.z;
 }

 float NineAxesMotion_readGravAcceleration(int axis)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccel();
 	}
 	if (axis == X_AXIS) {
 			return gravAccelData.x;
 	} else if (axis == Y_AXIS) {
 			return gravAccelData.y;
 	} else if (axis == Z_AXIS) {
 			return gravAccelData.z;
 	}
 }

 void NineAxesMotion_readGravAccel(float& x, float& y, float& z)
 {
 	readGravAcceleration(x, y, z);
 }

 float NineAxesMotion_readGravAccel(int axis)
 {
 	return readGravAcceleration(axis);
 }

 //Linear Acceleration
 void NineAxesMotion_readLinearAcceleration(float& x, float& y, float& z)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateLinearAccel();
 	}
 	x = gravAccelData.x;
 	y = gravAccelData.y;
 	z = gravAccelData.z;
 }

 float NineAxesMotion_readLinearAcceleration(int axis)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateAccel();
 	}
 	if (axis == X_AXIS) {
 			return linearAccelData.x;
 	} else if (axis == Y_AXIS) {
 			return linearAccelData.y;
 	} else if (axis == Z_AXIS) {
 			return linearAccelData.z;
 	}
 }

 void NineAxesMotion_readLinearAccel(float& x, float& y, float& z)
 {
 	readLinearAcceleration(x, y, z);
 }

 float NineAxesMotion_readLinearAccel(int axis)
 {
 	return readLinearAcceleration(axis);
 }

 //Gyroscope
 void NineAxesMotion_readGyro(float& x, float& y, float& z)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGyro();
 	}
 	x = gyroData.x;
 	y = gyroData.y;
 	z = gyroData.z;
 }

 float NineAxesMotion_readGyro(int axis)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateGyro();
 	}
 	if (axis == X_AXIS) {
 			return gyroData.x;
 	} else if (axis == Y_AXIS) {
 			return gyroData.y;
 	} else if (axis == Z_AXIS) {
 			return gyroData.z;
 	}
 }

 //Magnetometer
 void NineAxesMotion_readMagnetometer(float& x, float& y, float& z)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateMag();
 	}
 	x = magData.x;
 	y = magData.y;
 	z = magData.z;
 }

 float NineAxesMotion_readMagnetometer(int axis)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateMag();
 	}
 	if (axis == X_AXIS) {
 			return magData.x;
 	} else if (axis == Y_AXIS) {
 			return magData.y;
 	} else if (axis == Z_AXIS) {
 			return magData.z;
 	}
 }

 void NineAxesMotion_readMag(float& x, float& y, float& z)
 {
 	readMagnetometer(x, y, z);
 }

 float NineAxesMotion_readMag(int axis)
 {
 	return readMagnetometer(axis);
 }

 /* Quaternion */
 void NineAxesMotion_readQuaternion(int16_t& w, int16_t& x, int16_t& y, int16_t& z)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateQuat();
 	}
 	w = quatData.w;
 	x = quatData.x;
 	y = quatData.y;
 	z = quatData.z;
 }

 int16_t NineAxesMotion_readQuaternion(int axis)
 {
 	if (dataUpdateMode == AUTO)
 	{
 		updateQuat();
 	}
 	if (axis == X_QUAT) {
 			return quatData.x;
 	} else if (axis == Y_QUAT) {
 			return quatData.y;
 	} else if (axis == Z_QUAT) {
 			return quatData.z;
 	}	else if (axis == W_QUAT) {
 			return quatData.w;
 	}
 }

 void NineAxesMotion_readQuat(int16_t& w, int16_t& x, int16_t& y, int16_t& z)
 {
 	readQuaternion(w, x, y, z);
 }

 int16_t NineAxesMotion_readQuat(int axis)
 {
 		return readQuaternion(axis);
 }
