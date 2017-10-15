/****************************************************************************
 *
 * MODULE:             SMBus functions
 *
 * COMPONENT:          $RCSfile: SMBus.c,v $
 *
 * VERSION:            $Name: RD_RELEASE_6thMay09 $
 *
 * REVISION:           $Revision: 1.2 $
 *
 * DATED:              $Date: 2008/02/29 18:02:05 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             Lee Mitchell
 *
 * DESCRIPTION:
 * A set of functions to communicate with devices on the SMBus
 *
 * CHANGE HISTORY:
 *
 * $Log: SMBus.c,v $
 * Revision 1.2  2008/02/29 18:02:05  dclar
 * dos2unix
 *
 * Revision 1.1  2006/12/08 10:51:17  lmitch
 * Added to repository
 *
 *
 *
 * LAST MODIFIED BY:   $Author: dclar $
 *                     $Modtime: $
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on
 * each copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2005, 2006. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppHardwareApi.h>

#include <suli.h>
volatile uint32 timer0_overflow_count = 0;
volatile uint32 timer0_millis = 0;



#define SERIAL_DEBUG
//#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG

# include <serial.h>
# include <fprintf.h>
extern tsFILE sDebugStream;
extern tsFILE sSerStream;

#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

static bool_t bSMBusWait(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

void vSMBusInit(void)
{

	/* run bus at 100KHz */
	//vAHI_SiMasterConfigure(TRUE, FALSE, 47);
			// 16/[(PreScaler + 1) x 5]MHz
			//		--> 31:100KHz, 7:400KHz, 47:66Khz
	vAHI_SiMasterConfigure(TRUE, FALSE, 7);

	//vAHI_SiSetLocation(TRUE);
	vfPrintf(&sSerStream, "%d %d ", bAHI_SiMasterPollArbitrationLost(),bAHI_SiMasterCheckRxNack());



}








bool_t bSMBusWrite(uint8 u8Address, uint8 u8Command, uint8 u8Length, uint8* pu8Data)
{

	bool_t bCommandSent = FALSE;

	/* Send address with write bit set */
	vAHI_SiMasterWriteSlaveAddr(u8Address, E_AHI_SI_SLAVE_RW_SET);

	vAHI_SiMasterSetCmdReg(E_AHI_SI_START_BIT,
					 E_AHI_SI_NO_STOP_BIT,
					 E_AHI_SI_NO_SLAVE_READ,
					 E_AHI_SI_SLAVE_WRITE,
					 E_AHI_SI_SEND_ACK,
					 E_AHI_SI_NO_IRQ_ACK);

	if(!bSMBusWait()) return(FALSE);

	while(bCommandSent == FALSE || u8Length > 0){

		if(!bCommandSent){

			/* Send command byte */
			vAHI_SiMasterWriteData8(u8Command);
			bCommandSent = TRUE;

		} else {

			u8Length--;

			/* Send data byte */
			vAHI_SiMasterWriteData8(*pu8Data++);

		}

		/*
		 * If its the last byte to be sent, send with
		 * stop sequence set
		 */
		if(u8Length == 0){

			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,
							 E_AHI_SI_SEND_ACK,
							 E_AHI_SI_NO_IRQ_ACK);

		} else {

			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,
							 E_AHI_SI_SEND_ACK,
							 E_AHI_SI_NO_IRQ_ACK);

		}

		if(!bSMBusWait()) return(FALSE);

	}

	return(TRUE);

}


PUBLIC bool_t bSMBusRandomRead(uint8 u8Address, uint8 u8Command, uint8 u8Length, uint8* pu8Data)
{

	/* Send address with write bit set */
	vAHI_SiMasterWriteSlaveAddr(u8Address, !E_AHI_SI_SLAVE_RW_SET);

	vAHI_SiMasterSetCmdReg(E_AHI_SI_START_BIT,
					 E_AHI_SI_NO_STOP_BIT,
					 E_AHI_SI_NO_SLAVE_READ,
					 E_AHI_SI_SLAVE_WRITE,
					 E_AHI_SI_SEND_ACK,
					 E_AHI_SI_NO_IRQ_ACK);

	if(!bSMBusWait()) return(FALSE);

	vfPrintf(&sSerStream, "\n\rbSMBusRandomRead Pass 1 %0x", u8Address);

	/* Send command byte */
	vAHI_SiMasterWriteData8(u8Command);
	vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
					 E_AHI_SI_NO_STOP_BIT,
					 E_AHI_SI_NO_SLAVE_READ,
					 E_AHI_SI_SLAVE_WRITE,
					 E_AHI_SI_SEND_ACK,
					 E_AHI_SI_NO_IRQ_ACK);

	if(!bSMBusWait()) return(FALSE);

	vfPrintf(&sSerStream, "\n\rbSMBusRandomRead Pass 2");

	while(u8Length > 0){

		u8Length--;

		/*
		 * If its the last byte to be sent, send with
		 * stop sequence set
		 */
		if(u8Length == 0){

			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,
							 E_AHI_SI_SLAVE_READ,
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_NACK,
							 E_AHI_SI_NO_IRQ_ACK);

		} else {

			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,
							 E_AHI_SI_SEND_ACK,
							 E_AHI_SI_NO_IRQ_ACK);

		}

		while(bAHI_SiMasterPollTransferInProgress()); /* busy wait */

		*pu8Data++ = u8AHI_SiMasterReadData8();

	}

	return(TRUE);

}


PUBLIC bool_t bSMBusSequentialRead(uint8 u8Address, uint8 u8Length, uint8* pu8Data)
{

	/* Send address with write bit set */
	vAHI_SiMasterWriteSlaveAddr(u8Address, !E_AHI_SI_SLAVE_RW_SET);

	vAHI_SiMasterSetCmdReg(E_AHI_SI_START_BIT,
					 E_AHI_SI_NO_STOP_BIT,
					 E_AHI_SI_NO_SLAVE_READ,
					 E_AHI_SI_SLAVE_WRITE,
					 E_AHI_SI_SEND_ACK,
					 E_AHI_SI_NO_IRQ_ACK);

	if(!bSMBusWait()) return(FALSE);


	while(u8Length > 0){

		u8Length--;

		/*
		 * If its the last byte to be sent, send with
		 * stop sequence set
		 */
		if(u8Length == 0){

			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,
							 E_AHI_SI_SLAVE_READ,
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_NACK,
							 E_AHI_SI_NO_IRQ_ACK);

		} else {

			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_SLAVE_READ,
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_ACK,
							 E_AHI_SI_NO_IRQ_ACK);

		}

		while(bAHI_SiMasterPollTransferInProgress()); /* busy wait */

		*pu8Data++ = u8AHI_SiMasterReadData8();

	}

	return(TRUE);

}

bool_t bSMBusSequentialRead_NACK(uint8 u8Address, uint8 u8Length, uint8* pu8Data)
{

	/* Send address with write bit set */
	vAHI_SiMasterWriteSlaveAddr(u8Address, !E_AHI_SI_SLAVE_RW_SET);

	vAHI_SiMasterSetCmdReg(E_AHI_SI_START_BIT,
					 E_AHI_SI_NO_STOP_BIT,
					 E_AHI_SI_NO_SLAVE_READ,
					 E_AHI_SI_SLAVE_WRITE,
					 E_AHI_SI_SEND_ACK,
					 E_AHI_SI_NO_IRQ_ACK);

	if(!bSMBusWait()) return(FALSE);


	while(u8Length > 0){

		u8Length--;

		/*
		 * If its the last byte to be sent, send with
		 * stop sequence set
		 */
		if(u8Length == 0){

			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,
							 E_AHI_SI_SLAVE_READ,
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_NACK,
							 E_AHI_SI_NO_IRQ_ACK);

		} else {

			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_SLAVE_READ,
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_NACK,
							 E_AHI_SI_NO_IRQ_ACK);

		}

		while(bAHI_SiMasterPollTransferInProgress()); /* busy wait */

		*pu8Data++ = u8AHI_SiMasterReadData8();

	}

	return(TRUE);

}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

PRIVATE bool_t bSMBusWait(void)
{

	while(bAHI_SiMasterPollTransferInProgress()); /* busy wait */

	if (bAHI_SiMasterPollArbitrationLost() | bAHI_SiMasterCheckRxNack())	{

		/* release bus & abort */
		vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
						 E_AHI_SI_STOP_BIT,
						 E_AHI_SI_NO_SLAVE_READ,
						 E_AHI_SI_SLAVE_WRITE,
						 E_AHI_SI_SEND_ACK,
						 E_AHI_SI_NO_IRQ_ACK);
		return(FALSE);
	}

	return(TRUE);

}










/*
 * delay us
 */
void suli_delay_us(uint32 us)
{
    uint32 mark_time = suli_micros();
    while(suli_micros() - mark_time < us);
}


/*
 * delay ms
 */
void suli_delay_ms(uint32 ms)
{
    uint32 mark_time = suli_millis();
    while(suli_millis() - mark_time < ms);
}


/*
 * Returns the number of milliseconds since your board began running the current program.
 * This number will overflow (go back to zero), after approximately 50 days.
 */
uint32 suli_millis()
{
    return (u16AHI_TimerReadCount(E_AHI_TIMER_0)/1000 + 60 * timer0_overflow_count);
}


/*
 * Returns the number of microseconds since your board began running the current program.
 * This number will overflow (go back to zero), after approximately 70 minutes.
 * Note: there are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.
 */
uint32 suli_micros()
{
    return (u16AHI_TimerReadCount(E_AHI_TIMER_0) + 60000 * timer0_overflow_count);
}


/*
 * I2C interface initialize.
 */
void suli_i2c_init(void * i2c_device)
{
    vAHI_SiMasterConfigure(
        TRUE,  //bPulseSuppressionEnable,
        FALSE, //bInterruptEnable,
        7);   //uint8 u8PreScaler);  //16M/((scale+1)*5) = 100k
    //vAHI_SiSetLocation(TRUE);  //D16,D17 as i2c
}


/*
 * write a buff to I2C
 * - i2c_device: i2c device pointer
 * - dev_addr: device address
 * - data: data buff
 * - len: data lenght
 */
uint8 suli_i2c_write(void * i2c_device, uint8 dev_addr, uint8 *data, uint8 len)
{
    vAHI_SiMasterWriteSlaveAddr(dev_addr, FALSE);
    // bSetSTA,  bSetSTO,  bSetRD,  bSetWR,  bSetAckCtrl,  bSetIACK);
    bAHI_SiMasterSetCmdReg(TRUE, FALSE, FALSE, TRUE, E_AHI_SI_SEND_ACK, E_AHI_SI_NO_IRQ_ACK);
    while(bAHI_SiMasterPollTransferInProgress()); //Waitforanindicationofsuccess

    int i;
    uint8 *old = data;
    for(i = 0; i < len; i++)
    {
        vAHI_SiMasterWriteData8(*data++);
        if(i == (len - 1))  //should send stop
        {
            bAHI_SiMasterSetCmdReg(FALSE, TRUE, FALSE, TRUE, E_AHI_SI_SEND_ACK, E_AHI_SI_NO_IRQ_ACK);
        } else
        {
            bAHI_SiMasterSetCmdReg(FALSE, FALSE, FALSE, TRUE, E_AHI_SI_SEND_ACK, E_AHI_SI_NO_IRQ_ACK);
        }
        while(bAHI_SiMasterPollTransferInProgress()); //Waitforanindicationofsuccess
        if(bAHI_SiMasterCheckRxNack())
        {
            bAHI_SiMasterSetCmdReg(FALSE, TRUE, FALSE, FALSE, E_AHI_SI_SEND_ACK, E_AHI_SI_NO_IRQ_ACK);
            break;
        }
    }
    return data - old;
}


/*
 * read a buff to I2C
 * - i2c_device: i2c device pointer
 * - dev_addr: device address
 * - data: data buff
 * - len: data lenght
 * return
 */
uint8 suli_i2c_read(void *i2c_device, uint8 dev_addr, uint8 *buff, uint8 len)
{
    vAHI_SiMasterWriteSlaveAddr(dev_addr, TRUE);
    // bSetSTA,  bSetSTO,  bSetRD,  bSetWR,  bSetAckCtrl,  bSetIACK);
    bAHI_SiMasterSetCmdReg(TRUE, FALSE, FALSE, TRUE, E_AHI_SI_SEND_ACK, E_AHI_SI_NO_IRQ_ACK);
    while(bAHI_SiMasterPollTransferInProgress()); //Waitforanindicationofsuccess

    int i;
    uint8 *old = buff;
    for(i = 0; i < len; i++)
    {
        if(i == (len - 1))  //should send stop, nack
        {
            bAHI_SiMasterSetCmdReg(FALSE, TRUE, TRUE, FALSE, E_AHI_SI_SEND_NACK, E_AHI_SI_NO_IRQ_ACK);
        } else
        {
            bAHI_SiMasterSetCmdReg(FALSE, FALSE, TRUE, FALSE, E_AHI_SI_SEND_ACK, E_AHI_SI_NO_IRQ_ACK);
        }
        while(bAHI_SiMasterPollTransferInProgress()); //Waitforanindicationofsuccess
        *buff++ = u8AHI_SiMasterReadData8();
    }
    return buff - old;
}























/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
