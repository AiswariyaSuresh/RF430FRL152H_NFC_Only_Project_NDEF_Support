/*
 * Patch.c
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

#include "patch.h"
#include "NDEF.h"
#include "types.h"
#include <rf430frl152h.h>

/*******************************Driver/Patch Table Format*******************************/
/*
 *   Address	Value 			Comment
 *
 *   0xFFCE     0xCECE      	The driver table start key, always same address (0xFFCE)
 *
 *   0xFFCC		0x1B00			The command ID of the digital sensor sampling function
 *	 0xFFCA		Address			The address of the driver sensor sampling function in FRAM
 *
 *   0xFFC8		0x0100			The digital sensor function driver initialization function
 *   0xFFC6		Address			The address of the driver function initialization in FRAM
 *
 *
 *   Optional:
 *   0xFFC4		ID				Another driver/patch function ID
 *   0xFFC2		Address			Address of the function above
 *
 *      *          *			Pairs
 *      *		   *
 *
 *   End optional
 *
 *   0xFFC4		0xCECE			Ending key
 *****************************************************************************************/
  /* If start key not present in starting location, table does not exist
   *  If it does, a ROM routine will parse it and setup the calls to be made to the
   *  appropriate address when needed.
   */
 /*****************************************************************************************/

//Start key
#pragma RETAIN(START_KEY);
#pragma location = DRIVER_TABLE_START
const u16_t START_KEY = DRIVER_TABLE_KEY;

////Custom Command
#pragma RETAIN(CustomCommandID);
#pragma location = CUSTOM_COMMAND														// the location of the command ID
const u16_t  CustomCommandID = USER_CUSTOM_COMMAND_ID;              					// the function identifier

// Function address
#pragma RETAIN(CustomCommandAddress);
#pragma location = CUSTOM_COMMAND_ADDR														// the location of the address
const DriverFunction CustomCommandAddress = (DriverFunction)&userCustomCommand;     	// the location the function is in

//First ID, address pair
#pragma RETAIN(GetSystemInfoID);
#pragma location = GET_SYSTEM_INFO_COMMAND														// the location of the command ID
const u16_t  GetSystemInfoID = GET_SYSTEM_INFO_ID;              								// the function identifier

#pragma RETAIN(GetSystemInfoAddress);
#pragma location = GET_SYSTEM_INFO_ADDR														// the location of the address
const DriverFunction GetSystemInfoAddress = (DriverFunction)&GetSystemInfo_Patched;     	// the location the function is in

//Second ID, address pair
#pragma RETAIN(GetMultipleBlockSecurityStatusID);
#pragma location = GET_MULTIPLE_BLOCK_SECURITY_STATUS_COMMAND														// the location of the command ID
const u16_t  GetMultipleBlockSecurityStatusID = GET_MUL_BLCK_SEC_STATUS_ID;              							// the function identifier

#pragma RETAIN(GetMultipleBlockSecurityStatusAddress);
#pragma location = GET_MULTIPLE_BLOCK_SECURITY_STATUS_ADDR																	// the location of the address
const DriverFunction GetMultipleBlockSecurityStatusAddress = (DriverFunction)&GetMultipleBlockSecurityStatus_Patched;     	// the location the function is in

//Third ID, address pair
#pragma RETAIN(BlockLockROMID);
#pragma location = BLOCK_LOCK_ROM_COMMAND													// the location of the command ID
const u16_t  BlockLockROMID = BLOCK_LOCK_ID;              									// the function identifier

#pragma RETAIN(BlockLockROMAddress);
#pragma location = BLOCK_LOCK_ROM_ADDR														// the location of the address
const DriverFunction BlockLockROMAddress = (DriverFunction)&BlockLockROM_Patched;     		// the location the function is in

//Ending key
#pragma RETAIN(END_KEY);
#pragma location = DRIVER_TABLE_END
const u16_t END_KEY = DRIVER_TABLE_KEY;

//No memory used here, these variables point to the UID
#pragma RETAIN(UID_0);
#pragma location = 0x1A08
const u08_t UID_0;

#pragma RETAIN(UID_1);
#pragma location = 0x1A09
const u08_t UID_1;

#pragma RETAIN(UID_2);
#pragma location = 0x1A0A
const u08_t UID_2;

#pragma RETAIN(UID_3);
#pragma location = 0x1A0B
const u08_t UID_3;

#pragma RETAIN(UID_4);
#pragma location = 0x1A0C
const u08_t UID_4;

#pragma RETAIN(UID_5);
#pragma location = 0x1A0D
const u08_t UID_5;
#define  UID_6 0x07
#define  UID_7 0xE0


/**************************************************************************************************************************************************
*  GetSystemInfo_Patched
***************************************************************************************************************************************************
*
* Brief : 	Needed for NFC NDEf functionality
*			The Get System Info needed to be patched because in the ROM function the block numbers are declared as
*			being 243, however in the NDEF message only even numbers of blocks are allowed
*			If there is a mismatch between the NDEF block number declaration and the Get System Info
*			the NDEF detection will fail as a result.
*			One less block number is declared here 242 (0xF2).
*			This function is called by the ROM RF stack.
*
* Param[in] :   parameter - the configuration to setup the ISO15693 (option to clear the lock blocks)
*
* Param[out]:   None
*
* Return        None
**************************************************************************************************************************************************/
#pragma RETAIN(GetSystemInfo_Patched);
void GetSystemInfo_Patched()
{
    RF13MTXF_L = 0;                 // flags
    RF13MTXF_L = BIT2;              // VICC Memory size present
    RF13MTXF_L = UID_0;
    RF13MTXF_L = UID_1;
    RF13MTXF_L = UID_2;
    RF13MTXF_L = UID_3;
    RF13MTXF_L = UID_4;
    RF13MTXF_L = UID_5;
    RF13MTXF_L = UID_6;
    RF13MTXF_L = UID_7;
    RF13MTXF_L = 0xF3-2;	// 0 = 1 block per spec
    RF13MTXF_L = *(unsigned char *)(0x1CE7)-1;//BLOCK_SIZE - 1;  // 0 = 1 byte per spec
}

/**************************************************************************************************************************************************
*  BlockLockROM_Patched
***************************************************************************************************************************************************
*
* Brief : 	The ROM equivalent function did not have proper lock block operation in 4-byte block mode.
* 			This functions addresses the issue.
* 			Note: This function depends on Firmware_System_Control_Byte variable to be kept declared
* 			and as a "volatile".
* 			Do not call this function directly.  Rather use the associated API called: BlockLockAPI(...)
*			This function is called by the ROM RF stack.
*			Due to constraint on how much memory is used two 4-byte blocks are locked for each lock command.
*			if block 0 or block 1 is locked both block 0 and block 1 are locked.  Same for other blocks.
*			Paging is supported.
*
* Param[in] :   blockNumber
*				checkLock   - if 0 will perform a lock.  If 1 will return a status if the block is locked
*
* Param[out]:   None
*
* Return        Valid only when checkLock is set to 1
* 				0 - block not locked
* 				1 - block is locked
**************************************************************************************************************************************************/
#pragma RETAIN(BlockLockROM_Patched);
u16_t BlockLockROM_Patched(u16_t blockNumber , u08_t checkLock)
{
	if ((Firmware_System_Control_Byte & FOUR_BYTE_BLOCK_MASK) == FOUR_BYTE_BLOCK && blockNumber < FRAM_BLOCKS_8)
	{
		asm (" push.w R10 ");
		asm (" mov.w R12, R15 ");
		asm (" mov.w R12, R14 ");
		asm (" add.w #0xFA00, R14 ");
		asm (" call #0x5CCC"); 			// shifts block number register R12
		asm (" mov.w R12, R10 ");		// copy result
		if ((Firmware_System_Control_Byte & FIRST_ISO_PAGE_MASK) == FIRST_ISO_PAGE)
		{
			asm (" add.w #0xF840, R10");
		}
		else
		{
			asm (" add.w #0xF850, R10");
		}
		asm (" mov.b #0x1, R14 " ); //correction
		asm (" br #0x544E ");
    }
    else
    {
    	asm (" br #0x542C ");  //Call the ROM function, no fixes necessary
    }

	{
		return 0; 	// will not be reached
	}
}

/**************************************************************************************************************************************************
*  GetMultipleBlockSecurityStatus_Patched
***************************************************************************************************************************************************
*
* Brief : 	Needed for NFC NDEF functionality
*			Get Multiple Block Security status was needed to be added because it is not supported by the ROM RF stack
*			It is needed in the NDEF detection process
*			This function is called by the ROM RF stack.
*
* Param[in] :   None
*
* Param[out]:   None
*
* Return        None
**************************************************************************************************************************************************/

#pragma RETAIN(GetMultipleBlockSecurityStatus_Patched);
void GetMultipleBlockSecurityStatus_Patched()
{
	u08_t startblockNumber;
	u08_t numberBlocks;
	u08_t i;

	startblockNumber = RF13MRXF_L;
	numberBlocks = RF13MRXF_L;

	RF13MTXF_L = 0;

    for( i = startblockNumber; i < (startblockNumber + numberBlocks + 1); i++ )
    {
    	if (RF13MFIFOFL_H > 20)
    	{
    		while (RF13MFIFOFL_H > 8);
    	}
    	//in 4 byte block mode, each lock block command locks 8 bytes.  So block 0 lock will lock memory 0xF868-0xF86F...
        if (BlockLockAPI( i, CHECK_LOCK ) ) //call patched function
        {
			RF13MTXF_L = LOCKED_FLAG;
        }
        else
        {
			RF13MTXF_L = 0;
        }
    }
}

/**************************************************************************************************************************************************
*  userCustomCommand
***************************************************************************************************************************************************
*
* Brief : This function is called by the RF stack whenever a custom command by its ID number is transmitted
*
* Param[in] :   None
*
* Param[out]:   None
*
* Return        None
*
* This is an example only, and the user if free to modify as needed.
*
* Operation: Example with TRF7970AEVM
* Use Test tab to send following sequence: 18 02 AA 07 10 10
* 18 - TRF7970AEVM Host command (omit for other readers - not sent out over RF)
* 02 - High speed mode selection (start of actuall RF packet)
* AA - The actual custom command
* 07 - TI Manufacturer ID (need by this IC)
* 01 - Set Error LED to on  (0x00 to be off)
**************************************************************************************************************************************************/
void userCustomCommand()
{
    u08_t control;

    if( RF13MFIFOFL_L == CRC_LENGTH_IN_BUFFER + DATA_IN_LENGTH)         // CRC_LENGTH + 1 byte expected
    {
        control = RF13MRXF_L;  // pull one byte from the recieve FIFO

        P1DIR |= BIT4; // set ALARM LED as an output

        if (control)
        {
        	P1OUT |= BIT4;		// turn on ALARM LED
        }
        else
        {
        	P1OUT &= ~BIT4;  	// turn off ALARM LED
        }

        //Device has 32 byte RX FIFO and 32 byte TX FIFO, this includes the CRC bytes

        //use RF13MRXF to receive two bytes
        //use RF13MRXF_L to receive one byte
        //to receive more than one byte simply continue to read the RF13MRXF_L register.
        //The limit is 32 bytes, but in reality it is less due to protocol overhead
       RF13MTXF_L = 0x0;      // no error, send out
       //To transmit more than one byte repeatedly write data into RF13MTXF_L for each data byte/word and it will go into the FIFO to be transmitted
    }
    else
    {
       RF13MTXF_L = 0x1;    // an error response
    }
}

