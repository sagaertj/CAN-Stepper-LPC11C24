#include <string.h>
#include "LPC11xx.h"                    /* LPC11xx definitions                */

//LPC1113FHN33/201 LPC1100 24 kB FLASH 4 kB  RAM

#define FLASH_BLOCKSIZE		0x0100
#define FLASH_SECTORSIZE	0x1000

//Sector 	size 	start			-end
//0 			4 kB 	0x0000 0000 -0x0000 0FFF
//1 			4 kB 	0x0000 1000 -0x0000 1FFF
//2 			4 kB 	0x0000 2000 -0x0000 2FFF
//3 			4 kB 	0x0000 3000 -0x0000 3FFF
//4 			4 kB 	0x0000 4000 -0x0000 4FFF
//5 			4 kB 	0x0000 5000 -0x0000 5FFF
//6			4 Kb	0x0000 6000 -0x0000 6FFF
//7			4 Kb	0x0000 7000 -0x0000 7FFF


#define SECTOR7	7

#define SECTOR5_ADDR  0x00005000
#define SECTOR6_ADDR  0x00006000
#define SECTOR7_ADDR  0x00007000

#define BLOCK_NUM		0


#define TRUE 1
#define FALSE 0


#define DISABLEIRQ __disable_irq();
#define ENABLEIRQ  __enable_irq();

#define IAP_LOCATION 0x1fff1ff1
typedef void (*IAP)(unsigned int [], unsigned int []);

// IAP function
static IAP mIAPEntry = (IAP)IAP_LOCATION;

// allocate memory for non-volatile memory so it isn't used by the linker
// for something else
static unsigned char mSectorMemory5[FLASH_SECTORSIZE] __attribute__((at(SECTOR7_ADDR)));



#define CMD_SUCCESS 							0						//Command is executed successfully.
#define INVALID_COMMAND 					1						//Invalid command.
#define SRC_ADDR_ERROR 						2						//Source address is not on a word boundary.
#define DST_ADDR_ERROR 						3						//Destination address is not on a correct boundary.
#define SRC_ADDR_NOT_MAPPED 				4 						//Source address is not mapped in the memory map. Count value is taken in to consideration where applicable.
#define DST_ADDR_NOT_MAPPED 				5 						//Destination address is not mapped in the memory map. Count value is taken in to consideration where applicable.
#define COUNT_ERROR 							6						//Byte count is not multiple of 4 or is not a permitted value.
#define INVALID_SECTOR 						7						//Sector number is invalid.
#define ECTOR_NOT_BLANK 					8 						//Sector is not blank.
#define SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION 9			//Command to prepare sector for write operation was not executed.
#define COMPARE_ERROR 						10						//Source and destination data is not same.
#define BUSY 									11						//Flash programming hardware interface is busy.



// IAP Command CODEs
#define PREPARE_SECTOR					50
#define RAM_TO_FLASH						51
#define ERASE_SECTOR						52
#define BLANK_CHECK						53
#define READ_PART_ID						54
#define READ_BOOTCODEVERSION			55
#define COMPARE							56
#define REINVOKE_ISP						57
#define READ_UID							58
//#define ERASE_PAGE						59 //not supported in LPC11C24

// CPU clock in kHz
#define CPU_CLK 48000


/**************************************************************************
DOES:    Erases a sector
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
int EraseSector(void)
{
	unsigned int Command[5], Result[5];
	
	//prepare
	Command[0] = PREPARE_SECTOR;
	Command[1] = SECTOR7;
	Command[2] = SECTOR7;
	
	DISABLEIRQ;
	mIAPEntry(Command, Result);
	ENABLEIRQ;
	
	if (Result[0] != CMD_SUCCESS)
		return FALSE;
	
	// erase sector
	Command[0] = ERASE_SECTOR;
	Command[1] = SECTOR7;
	Command[2] = SECTOR7;
	Command[3] = CPU_CLK;

	DISABLEIRQ;
	mIAPEntry(Command, Result);
	ENABLEIRQ;

	if (Result[0] != CMD_SUCCESS)
		return FALSE;
	
	return TRUE;
}

int ReadBlock( int readcnt,unsigned char * buffer)
{
//	unsigned char * pflash;
	unsigned short Byte;

//	pflash=(unsigned char *) SECTOR5_ADDR + (BLOCK_NUM * FLASH_BLOCKSIZE);
  
	for (Byte = 0; Byte < readcnt; Byte++)
  		buffer[Byte] = mSectorMemory5[Byte];

 	return TRUE;
}

/**************************************************************************
DOES:    Writes a buffer of max 256 bytes into a specific sector at block 0 offset
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
int WriteBlock(int writecnt,unsigned char * buffer)
{
  unsigned short Byte;
  unsigned char Buffer[FLASH_BLOCKSIZE];
  unsigned int Command[5], Result[5];
 
  // set up buffer to contain 0xFF
  for (Byte = 0; Byte < FLASH_BLOCKSIZE; Byte++)
  		Buffer[Byte] = 0xFF;

  for (Byte = 0; Byte < writecnt; Byte++)
  		Buffer[Byte] = buffer[Byte];

	// prepare sector
	Command[0] = PREPARE_SECTOR;
	Command[1] = SECTOR7;
	Command[2] = SECTOR7;
	DISABLEIRQ;
	mIAPEntry(Command, Result);
	ENABLEIRQ;

	if (Result[0] != CMD_SUCCESS)
		return FALSE; 
	
	// write to sector
	Command[0] = RAM_TO_FLASH;
	Command[1] = (unsigned int)(SECTOR5_ADDR + (BLOCK_NUM * FLASH_BLOCKSIZE));//   - (Offset % 256));
	Command[2] = (unsigned int)Buffer;
	Command[3] = FLASH_BLOCKSIZE;
	Command[4] = CPU_CLK;

	DISABLEIRQ;
	mIAPEntry(Command, Result);
	ENABLEIRQ;

	if (Result[0] != CMD_SUCCESS)
		return FALSE;
	
	// verify
	Command[0] = COMPARE;
	Command[1] = (unsigned int)(SECTOR5_ADDR + (BLOCK_NUM * FLASH_BLOCKSIZE));
	Command[2] = (unsigned int)(Buffer);
	Command[3] = FLASH_BLOCKSIZE;

	DISABLEIRQ;
	mIAPEntry(Command, Result);
	ENABLEIRQ;

	if (Result[0] != CMD_SUCCESS)
		return FALSE;
	
	return TRUE;
}



