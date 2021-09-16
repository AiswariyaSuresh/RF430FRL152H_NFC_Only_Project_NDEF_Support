/*
 * NDEF.c
 *
 * RF430FRL152H NFC Only Example Project
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include "types.h"
#include "NDEF.h"
#include "patch.h"
#include <string.h>

/* Firmware System Control Byte
 *
 *     Bit 0: 	ISOBlockSize				0 - 4 byte,		1 - 8 byte
 *     Bit 1:	Page						0 - page 1, 	1 - page 0 (Effective only for 4-byte block mode)
 *     Bit 2: 	ROMEUSCISupportEnabled		0 - disabled, 	1 - enabled (Forced to 0 on RF430FRL153H)
 *     Bit 3-5: ReservedISO
 *     Bit 6: 	NFCBridgeDisable  			0 - enabled, 	1 - disabled (see note below)
 *     Bit 7:   ROMSensorSupportEnable		0 - disabled, 	1 - enabled (Forced to 0 on RF430FRL154H)
 *
 *     NFC bridge is recommended to be disabled in this project.  Unexpected behaviour can occur,
 *     trying to use it, due to the configuration being setup here.
 *
 *     If eUSCI host controller portion is needed along with the RF functionality, the default project
 *     must be used.  That is NFC cannot be supported in that application (because the I2C/SPI host controller
 *     control registers are in the same place that the NFC file needs to be).  However the rest of the FRAM
 *     memory can be used for storing and reading using ISO15693.
 */

//This project is based on the RF430FRL152H.  However it will work as well on the RF430FRL154H.
//However ROM_SENSOR_SUPPORT_DISABLED (or ROMSensorSupportEnable see above for both )must be set in the firmware system control register.  This is forced automatically on the RF430FRL154H.
//This setting is needed to disable the ROM which uses block 0... as virtual registers, however this memory is needed for NDEF purposes.
#define FIRMWARE_CONTROL_ADDRESS 	0xF867
#pragma RETAIN(Firmware_System_Control_Byte);
#pragma location = FIRMWARE_CONTROL_ADDRESS
//This variable needs to be kept declared and as "volatile" for the BlockLockROM_Patched function to work properly.  Assignment can be changed however.
volatile const u08_t Firmware_System_Control_Byte = ROM_SENSOR_SUPPORT_DISABLED + EROM_EUSCI_SUPPORT_DISABLED + NFC_BRIDGE_DISABLED + FOUR_BYTE_BLOCK + FIRST_ISO_PAGE; //0x7F,		// this value sets the firmware system control register

// ROM variables - DO NOT CHANGE !!!
// Declared to protect from use by compiler
/********************************************/
#pragma RETAIN(DS)
#pragma location = 0x1C00
u08_t DS;
#pragma RETAIN(RF)
#pragma location = 0x1C6A
const u08_t RF;
#pragma RETAIN(NRX)
#pragma location = 0x1CA4 //rx
const u08_t NRX[34];
#pragma RETAIN(NTX)
#pragma location = 0x1CC6 //tx
const u08_t NTX[33];
#pragma RETAIN(EL)
#pragma location = 0x1CF2
const u08_t EL;
#pragma RETAIN(PF)
#pragma location = 0x1C0A
const u16_t PF[48];
/********************************************/

/*
 * This is an NDEF message: www.ti.com
 * This can be made longer and edited as necessary.  However it is limited to the first page (about 968 bytes),
 * the second page is mainly used for patch firmware, but can be read over NFC as well, with proper commands.
*/

#define NDEF_START_ADDRESS	0xF868
#pragma RETAIN(NFC_NDEF_Message);
#pragma location = NDEF_START_ADDRESS;																	// the location of the address
const u08_t NFC_NDEF_Message[23] = {

		// Block 0
		0xE1, 		// NDEF Magic Number
		0x40, 		// Version Number, read/write access conditions
		0x79,  //0x7E,		// 1008 bytes / 8 = 126 blocks
		0x00,//0x04,//8 byte extended memory //0x00,		// does not support read multiple blocks (limited to only 3 blocks)

		// Block 1
		0x03,		// NDEF Message present
		0x0D,		// Length , 11 bytes-> payload of 7, pauload of 18 is 22 byte length
		0xD1,		// Record header
		0x01,		// type length

		// Block 2
		0x09,		// Payload length
		0x55,		// Record Type U (URI)
		0x01, 		// URI header identifier
		0x7A,       // 'z'
		0x74,		// 't'

		// Block 3
		0x69,		// 'i'
		0x2E,		// '.'
		0x63,		// 'c'
		0x64,       // 'd'
		0x6F,		// 'o'

		// Block 4
		0x6D,		// 'm'
		0xFE,		// TLV terminator
		0x00,		// Empty don't care
		0x00		// Empty don't care
};

/**************************************************************************************************************************************************
*  initISO15693
***************************************************************************************************************************************************
*
* Brief : Initializes the RF Stack
*
* Param[in] :   parameter - the configuration to setup the ISO15693 (option to clear the lock blocks)
*
* Param[out]:   None
*
* Return        None
**************************************************************************************************************************************************/
void initISO15693(u16_t parameters )
{

  RF13MCTL |= RF13MTXEN + RF13MRXEN + RF13MRFTOEN; 	// set up rx and tx functionality on RF13M module
  // enable interrupts  ** Do not change the following two lines, needed for proper RF stack operatoin
  RF13MINT |= RF13MRXIE + RX13MRFTOIE;  			// enable interrupts on RX and on timeout and over and under flow checking

  if (parameters & CLEAR_BLOCK_LOCKS )
  {
    memset ((u08_t *) FRAM_LOCK_BLOCKS, 0xFF, FRAM_LOCK_BLOCK_AREA_SIZE);     //block is locked with a zero bit, clears FRAM and RAM lock blocks
  }

//  BlockLockAPI(3, LOCK_BLOCK);  //Test this API
//
//  BlockLockAPI(3, CHECK_LOCK);  //Test this API
}

/**************************************************************************************************************************************************
*  BlockLockAPI
***************************************************************************************************************************************************
*
* Brief : Locks a block either in FRAM or in RAM.
*
* Param[in] :   block - The block to lock.  In 4-byte mode this will lock two blocks and depends on the page selected.
*
* Param[out]:   None
*
* Return        None
**************************************************************************************************************************************************/
u16_t BlockLockAPI(u16_t block, u08_t checkLock)
{
	u16_t locked;

	locked = BlockLockROM_Patched(block, checkLock );
	return locked;
}
