/*
 * Patch.h
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
#ifndef PATCH_H
#define PATCH_H


//****Patch functions********************************************************************/
void GetSystemInfo_Patched();
void GetMultipleBlockSecurityStatus_Patched();
u16_t BlockLockROM_Patched(u16_t blockNumber , u08_t checkLock);

void userCustomCommand();

typedef void(*DriverFunction)(void);

//------------------------------------------------------------------------------
// Driver section
//------------------------------------------------------------------------------
extern volatile const u08_t Firmware_System_Control_Byte;

#define CRC_LENGTH_IN_BUFFER          2  // the CRC bytes take 2 bytes in the packet
#define DATA_IN_LENGTH				  1  // only 1 byte of data in expected


#define DRIVER_TABLE_START 				0xFFCE               	// starting address for driver table
#define DRIVER_TABLE_KEY  				0xCECE               	// identifier indicating start and end of driver table
#define BLOCK_LOCK_ID		       		0x2600               	// Block Lock Code for ROM
#define GET_SYSTEM_INFO_ID		       	0x002B               	// Get System Info ISO15693 command ID
#define GET_MUL_BLCK_SEC_STATUS_ID		0x002C               	// Get Multiple Block Security Status ISO15693 command ID
#define USER_CUSTOM_COMMAND_ID       	0x00AA               	// user custom command, range from A0 - D0

#define NUMBER_OF_DRIVER_FUNCTIONS 		4                     	// the amount of patched functions
//------------------------------------------------------------------------------
#define CUSTOM_COMMAND 		   (DRIVER_TABLE_START-2)
#define CUSTOM_COMMAND_ADDR    (DRIVER_TABLE_START-4)

#define GET_SYSTEM_INFO_COMMAND (DRIVER_TABLE_START-6)  				// DIGITAL_SENSOR_DRIVER_ID, see below
#define GET_SYSTEM_INFO_ADDR    (DRIVER_TABLE_START-8)

#define GET_MULTIPLE_BLOCK_SECURITY_STATUS_COMMAND (DRIVER_TABLE_START-10)                		// INIT_DIGITAL_SENSOR_DRIVER_ID, see below
#define GET_MULTIPLE_BLOCK_SECURITY_STATUS_ADDR    (DRIVER_TABLE_START-12)

#define BLOCK_LOCK_ROM_COMMAND (DRIVER_TABLE_START-14)                		// INIT_DIGITAL_SENSOR_DRIVER_ID, see below
#define BLOCK_LOCK_ROM_ADDR    (DRIVER_TABLE_START-16)

#define DRIVER_TABLE_END  (DRIVER_TABLE_START-2-(NUMBER_OF_DRIVER_FUNCTIONS*4))

#endif
