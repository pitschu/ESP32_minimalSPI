/*****************************************************
 *
 *	Control program for the PitSchuLight TV-Backlight
 *	(c) Peter Schulten, Mülheim, Germany
 *	peter_(at)_pitschu.de
 *
 *	Die unveränderte Wiedergabe und Verteilung dieses gesamten Sourcecodes
 *	in beliebiger Form ist gestattet, sofern obiger Hinweis erhalten bleibt.
 *
 * 	Ich stelle diesen Sourcecode kostenlos zur Verfügung und biete daher weder
 *	Support an noch garantiere ich für seine Funktionsfähigkeit. Außerdem
 *	übernehme ich keine Haftung für die Folgen seiner Nutzung.

 *	Der Sourcecode darf nur zu privaten Zwecken verwendet und modifiziert werden.
 *	Darüber hinaus gehende Verwendung bedarf meiner Zustimmung.
 *
 *	History
 *	09.06.2013	pitschu		Start of work
 *	19.11.2013	pitschu 	first release
 *	05.05.2014	pitschu	v1.1 added new params: ledsX/Y, AGC
 *	24.07.2014	pitschu v1.2 added dynFramesLimit (Params version 135)
 */

/*
 * Some comments on this package:
 *
 */
#define DEBUG 2
#define DEBUG_COLOR	_ANSI_LRED_
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ctype.h"

#include "appl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_spi_flash.h"
#include "flashparams.h"

// pitschu: part of RAM used for FLASH params
// mark parameters in any source file with:
//		<any type>	__attribute__((section(".para.PARARAM"))) NameOfParameterVariable = <defult value>;

extern unsigned long _spararam;		// defined in linker script files
extern unsigned long _epararam;

unsigned long _sparaflash;		// defined by partition data
unsigned long _eparaflash;

#define FLASH_BASE				0		// 0x40200000
#define PARAM_PAGE_START		(((uint32_t)_sparaflash - FLASH_BASE) / SPI_FLASH_SEC_SIZE)
#define PARAM_PAGE_END  		(((uint32_t)_eparaflash - FLASH_BASE) / SPI_FLASH_SEC_SIZE)
#define PARAM_FLASH_START		(((uint32_t)_sparaflash - FLASH_BASE))
#define PARAM_FLASH_END			(((uint32_t)_eparaflash - FLASH_BASE)-16)		// last flash byte usable for parameters

static uint32_t		actualCRC = 0;
static uint32_t		flashCRC = 0;
static uint32_t		blockAddr;
static uint16_t		blockSize;			// # of bytes in parameter flash block (without CRC bytes)

#define	FLASH_INVALBLOCK		((long)('#'<<24) |	(long)('+'<<16) | (long)('#'<<8) | (long)('+'<<0) )
#define	FLASH_SIGNATURE			((long)('P'<<24) |	(long)('.'<<16) | (long)('S'<<8) | (long)('.'<<0) )
#define FLASH_VERSION			134
#define	FLASH_SIGNATURE_P		(PARAM_FLASH_END + 4)		// 4 bytes (should be 'P.S.')
#define FLASH_VERSION_P			(PARAM_FLASH_END + 8)		// 4 bytes version ID 0x0001 ...


#define ROUND4UP(x)				(((uint32_t)(x)+3)&(uint32_t)(~3))


typedef struct {
	uint8_t *paraP;
	short	 paraSize;
} flashParam_t;

flashParam_t flashParams[] = {

// Add what ever parameter you want to be saved to flash
		{(uint8_t*)134, (short)134},	// will be filled in InitFlashParamBlock() with reserved CCRAM parameter block

		/* add more params here if you don´t want to use the section(".data.PARARAM") */

		{(uint8_t*)0, (short)0},		// End Of List marker
};


xTaskHandle 		taskHandle_FlashParam = NULL;
void 				Flash_task (void *pvParameters);



void findFlashParamPartition()
{
    uint8_t buf[200];
    esp_partition_t pararam_partition;

    const esp_partition_t *esp_current_partition = esp_ota_get_boot_partition();

    if (esp_current_partition->type != ESP_PARTITION_TYPE_APP) {
        LOG_DEBUG("Error esp_current_partition->type is %d != ESP_PARTITION_TYPE_APP\n", esp_current_partition->type);
        return;
    }

    esp_partition_t find_partition;

    find_partition.type = 0x40;				// find our first PARAM partition
    find_partition.subtype = 0x80;

    const esp_partition_t *partition = esp_partition_find_first(find_partition.type, find_partition.subtype, NULL);
    if (partition == NULL)
    {
    	LOG_DEBUG("Could not find part type %02x, sub=%02x\n", find_partition.type, find_partition.subtype);
    	return;
    }
//    assert(partition != NULL);
    memcpy(&pararam_partition, (void*)partition, sizeof(esp_partition_t));

    LOG_DEBUG("Found FLASH partition\n");
    LOG_DEBUG("  label = '%s'\n", pararam_partition.label);
    LOG_DEBUG("  at addr=%08x, size=%d\n", pararam_partition.address, pararam_partition.size);

    spi_flash_read(pararam_partition.address, buf, 128);
    hexDump((uint8_t*)buf, 128);

    _sparaflash = pararam_partition.address;
    _eparaflash = _sparaflash + pararam_partition.size;
}




void Flash_task (void *pvParameters)
{
	uint32_t	flashTimer = 0;
	uint32_t	lastSysTimer = xTaskGetTickCount();
	uint32_t sysTick;

	LOG_DEBUG("FlashTask started at tickecnt=%d\n", lastSysTimer);

	while (1)
	{
		vTaskDelay(5000 / portTICK_PERIOD_MS);		// check every 5 seconds

		sysTick = xTaskGetTickCount();
		if (sysTick < lastSysTimer)	// sysTick wrapped
		{
			if (flashTimer)
				flashTimer = xTaskGetTickCount() + (portTICK_PERIOD_MS * 10);
		}
		lastSysTimer = sysTick;

		int r = checkForParamChanges();
		/*
		 * r = 0		when
		 */
		if (r < 2)								// RAM and FLASH are the same
			flashTimer = 0;
		else if (((r == 1) && flashTimer > 0) || ((r >= 2) && flashTimer == 0))		// some parameter was changed -> delay flash write; maybe other changes follow
			flashTimer = sysTick + (10000 / portTICK_PERIOD_MS);		// timeout after 10 seconds

		if (flashTimer && ((xTaskGetTickCount() > flashTimer) || (flashTimer - xTaskGetTickCount()) > (10000 / portTICK_PERIOD_MS)))
		{
			flashTimer = 0;
			whatChangedInRAM();
			updateAllParamsToFlash(0);			// check for changes and write params to flash
		}
	}
}



void Flash_Init(void)		// init hardware and start LED matrix driver task
{
	findFlashParamPartition ();

	initFlashParamBlock();			// check for valid flash parameter block

	checkForParamChanges();		// calc param CRC for later checks

	xTaskCreate(Flash_task, "Flash", 2048, NULL, FLASH_TASK_PRIO, &taskHandle_FlashParam);
}





#define CRC_INITIAL		0xA45E
#define FLASH_ACC		1
#define RAM_ACC			0

static 	uint32_t CalcRAMCRC(uint32_t crc, uint32_t *data, uint16_t len)
{
	uint16_t tmp;
	uint16_t i;
	uint8_t j;
	uint32_t localData;		// do NOT access <data> directly as FLASH access must be 4-byte aligned !!!
	uint8_t *cp = (uint8_t *)&localData;

	localData = *data;

	i = 0;
	while (i < len)
	{
		tmp = *cp++ << 8;

	    for (j=0; j<8; j++)
		{
	        if((crc^tmp) & 0x8000)
				crc = (crc<<1) ^ 0x1021;
	        else
				crc = crc << 1;

			tmp = tmp << 1;
	    }

	    i++;
	    if ((i & 3) == 0)		// get next 4byte chunk
	    {
	    	data++;
    		localData = *data;

	    	cp = (uint8_t *)&localData;
	    }
	}

	return crc;
}



static 	uint32_t CalcFLASHCRC(uint32_t crc, uint32_t adr, uint16_t len)
{
	uint16_t tmp;
	uint16_t i;
	uint8_t j;
	uint32_t localData;		// do NOT access <data> directly as FLASH access must be 4-byte aligned !!!
	uint8_t *cp = (uint8_t *)&localData;

	spi_flash_read((uint32_t)adr, (uint32_t*)&localData, sizeof(uint32_t));

	i = 0;
	while (i < len)
	{
		tmp = *cp++ << 8;

	    for (j=0; j<8; j++)
		{
	        if((crc^tmp) & 0x8000)
				crc = (crc<<1) ^ 0x1021;
	        else
				crc = crc << 1;

			tmp = tmp << 1;
	    }

	    i++;
	    if ((i & 3) == 0)		// get next 4byte chunk
	    {
	    	adr += 4;
    		spi_flash_read((uint32_t)adr, (uint32_t*)&localData, sizeof(uint32_t));

	    	cp = (uint8_t *)&localData;
	    }
	}

	return crc;
}




uint32_t whatChangedInRAM (void)
{
	uint16_t i;
	uint32_t localRAM;		// do NOT access <data> directly as FLASH access must be 4-byte aligned !!!
	uint8_t *cpR = (uint8_t *)&localRAM;
	uint32_t localFLASH;		// do NOT access <data> directly as FLASH access must be 4-byte aligned !!!
	uint8_t *cpF = (uint8_t *)&localFLASH;

	uint32_t flashAddr = blockAddr + 4;
	if (flashAddr == 0)					// no valid block found -> cannot load from flash
	{
		PRT_DEBUG("whatChangedInRAM: No block found -> return err\n");
		return (-1);
	}

	uint32_t *ramAddr = (uint32_t*)flashParams[0].paraP;
	uint16_t len = (uint16_t)flashParams[0].paraSize;

	spi_flash_read((uint32_t)flashAddr, (uint32_t*)&localFLASH, sizeof(uint32_t));
	LOG_DEBUG("whatChangedInRAM compare flash=%08x, ram=%08x, len=%d\n", flashAddr, (int)ramAddr, len);

	localRAM = *ramAddr;
	i = 0;
	while (i < len)
	{
		if (*cpR != *cpF)
		{
			PRT_DEBUG("RAM was changed at %p + %d F=%08x, R=%08x\n", ramAddr, i%3, localFLASH, localRAM);
			break;
		}

		cpR++;
		cpF++;
	    i++;

	    if ((i & 3) == 0)		// get next 4byte chunk
	    {
	    	ramAddr++;
	    	localRAM = *ramAddr;
	    	cpR = (uint8_t *)&localRAM;

	    	flashAddr += 4;
    		spi_flash_read((uint32_t)flashAddr, (uint32_t*)&localFLASH, sizeof(uint32_t));

	    	cpF = (uint8_t *)&localFLASH;
	    }
	}

	return (uint32_t)ramAddr;

}



int calcParameterBlockSize ()
/*
 * add all sizeof() values in the flashParams array; add 4 bytes of CRC and 1 "valid byte"
 */
{
	uint8_t *s;
	int i, j;

	i = 0;
	j = 0;
	while ((s = flashParams[i].paraP) != (uint8_t*)0)
	{
		j += flashParams[i].paraSize;
		j = ROUND4UP(j);
		i++;
	}

	j += sizeof(flashCRC) + 4;			// add 4 for CRC and 4 for "valid indicator" byte
	LOG_DEBUG("calcParameterBlockSize = %d\n", j);

	return (j);
}




uint32_t findActualParameterFlashBlock ()
/*
 * <blockAddr> is set to the start addr (= "valid byte" of actual parameter block;
 * The return value points to the flash byte just behind the block
 */
{
	uint32_t s;
	uint32_t calcCRC;
	uint32_t l;
	int i;


	blockSize = calcParameterBlockSize();
// walk thru the stored blocks and check the "valid byte" and their CRC.
// If it matches with the stored CRC, we found the actual block.

	i = 0;
	s = (uint32_t)PARAM_FLASH_START;

	while (s < (uint32_t)(PARAM_FLASH_END - blockSize))
	{
		blockAddr = s;

		spi_flash_read((uint32_t)blockAddr, (uint32_t*)&l, sizeof(uint32_t));

		if (l != ~0)		// "valid byte" was flashed -> block invalid/old
		{
			s += blockSize;				// goto next block
		}
		else		// candidate found; calc it´s CRC
		{
			calcCRC = CRC_INITIAL;
			s += 4;				// parameter area start´s after the "valid byte"
			i = 0;
			while ((flashParams[i].paraP) != (uint8_t*)0)
			{
				calcCRC = CalcFLASHCRC(calcCRC, (uint32_t)s, flashParams[i].paraSize);		// calc CRC from flash bytes
				s += flashParams[i].paraSize;
				s = ROUND4UP(s);
				i++;
			}

			spi_flash_read((uint32_t)s, (uint32_t*)&flashCRC, sizeof(uint32_t));
			s += 4;

			if (calcCRC == flashCRC)		// flash content seems to be consistent -> found it
			{
				break;
			}
		}
	}

	if (s >= (uint32_t)(PARAM_FLASH_END - blockSize))		// no valid block found -> flash full; should be erased
	{
		PRT_DEBUG("findActualParameterFlashBlock: no valid block found; searched up to %08X\n", (unsigned int)s);
		return (0);
	}

	PRT_DEBUG("findActualParameterFlashBlock: found block at %08X, next at %08X\n", (unsigned int)blockAddr, (unsigned int)s);
	return (s);				// points to byte after found block

}




int initFlashParamBlock (void)
/*
 * check the signature and version bytes at end of flash. If they do not match then erase flash and store RAM values
 */
{
	uint32_t s;
	uint32_t v;

	flashParams[0].paraP = (uint8_t*)&_spararam;
	flashParams[0].paraSize = (short)((uint8_t*)&_epararam - (uint8_t*)&_spararam);

	spi_flash_read((uint32_t)FLASH_SIGNATURE_P, (uint32_t*)&s, sizeof(uint32_t));
	spi_flash_read((uint32_t)FLASH_VERSION_P, (uint32_t*)&v, sizeof(uint32_t));

	if (s != FLASH_SIGNATURE || v != FLASH_VERSION)
	{
		PRT_DEBUG ("initFlashParamBlock: wrong signature/ version -> call updateAllParamsToFlash with forced erase\n");
		updateAllParamsToFlash (FACTORY_KEY);		// force erase and save ram vars to flash
	}
	else
	{
		PRT_DEBUG ("initFlashParamBlock: spara=%p, para size=%d\n", flashParams[0].paraP, flashParams[0].paraSize);

		if (readAllParamsFromFlash() != 0)	// get saved values from flash; if error init flash
		{
			PRT_DEBUG ("initFlashParamBlock: no flash values found; erasing flash and write RAM vars\n");
			updateAllParamsToFlash (FACTORY_KEY);		// force erase and save ram vars to flash
		}
	}

	return (0);
}




int checkForParamChanges (void)
/*
 *  Check for changed RAM variables by comparing last CRC with actual CRC
 */
{
	uint8_t *s;
	uint32_t lastCRC = actualCRC;
	int i;

	actualCRC = CRC_INITIAL;
	i = 0;
	while ((s = flashParams[i].paraP) != (uint8_t*)0)
	{
		actualCRC = CalcRAMCRC(actualCRC, (uint32_t*)s, flashParams[i].paraSize);		// calc CRC from flash bytes
		i++;
	}

	i = 0;

	if (lastCRC != actualCRC)		// RAM vars where changed -> restart timer
		i = 1;

	if (flashCRC != actualCRC)		// but if RAM == FLASH -> no need to flash
		i += 2;

	TRC_DEBUG("checkForParamChanges: actual=%08x, last=%08x, flash=%08x  -> ret=%d\n", actualCRC, lastCRC, flashCRC, i);
	return (i);
}




int readAllParamsFromFlash (void)
/*
 * search actual block in flash and load all params to RAM variables
 */
{
	uint32_t s;
	uint32_t 	calcCRC;
	uint8_t *d;
	int i;

	s = (uint32_t)findActualParameterFlashBlock();
	if (s == 0)					// no valid block found -> cannot load from flash
	{
		PRT_DEBUG("readAllParamsFromFlash: No block found -> return err\n");
		return (-1);
	}

	PRT_DEBUG("readAllParamsFromFlash at %08X\n", (unsigned int)blockAddr);
	calcCRC = CRC_INITIAL;
	i = 0;
	s = blockAddr + 4;			// first byte is "valid byte"; skip it

	while ((flashParams[i].paraP) != (uint8_t*)0)
	{
		calcCRC = CalcFLASHCRC(calcCRC, (uint32_t)s, flashParams[i].paraSize);		// calc CRC from flash bytes
		s += flashParams[i].paraSize;
		s = ROUND4UP(s);
		i++;
	}

	spi_flash_read((uint32_t)s, (uint32_t*)&flashCRC, sizeof(uint32_t));

	if (calcCRC == flashCRC)		// flash content seems to be consistent -> load it
	{
		i = 0;
		s = (blockAddr + 4);

		while (( d = flashParams[i].paraP) != (uint8_t*)0)
		{
			spi_flash_read((uint32_t)s, (uint32_t*)d, flashParams[i].paraSize);

			s += flashParams[i].paraSize;
			s = ROUND4UP(s);

			i++;
		}
	}
	actualCRC = flashCRC;

	return (0);
}


// invalidates all FLASH params. Call should be followed by a RESTART
void invalidateFlashParams (int key)
{
	if (key == FACTORY_KEY)
	{
		uint32_t l = 0x5555;
		spi_flash_write(FLASH_SIGNATURE_P, (uint32_t *)&l, sizeof(l));
		l = 0xAAAA;
		spi_flash_write(FLASH_VERSION_P, (uint32_t *)&l, sizeof(l));

	}
}


int updateAllParamsToFlash (int forceErase)
/*
 * Writes all RAM variables into a new flash block. If no free block is available or the <forceErase> is != 0
 * then erase the flash sector first and save RAM variables in a new block.
 */
{
	uint8_t *d;
	uint32_t s;
	int i;
	uint32_t l;

	PRT_DEBUG("updateAllParamsToFlash with forceErase=%d\n", forceErase);

	if (forceErase == FACTORY_KEY || (s = findActualParameterFlashBlock()) == 0)		// no valid block found or force erase
	{
		s = PARAM_PAGE_START;

		PRT_DEBUG ("Param flash sparaflash=%08lx, eparaflash=%08lx\n", _sparaflash, _eparaflash);
		while (s < PARAM_PAGE_END)
		{
			PRT_DEBUG ("Erase parameter flash block %02x\n", (int)s);

			spi_flash_erase_sector(s);

			vTaskDelay(1);
			s++;
		}

		l = FLASH_SIGNATURE;
		spi_flash_write(FLASH_SIGNATURE_P, (uint32_t *)&l, sizeof(l));
		l = FLASH_VERSION;
		spi_flash_write(FLASH_VERSION_P, (uint32_t *)&l, sizeof(l));

		s = 0;
	}

	checkForParamChanges ();		// calculate CRC

	if (s != 0)			// if valid block found then invalidate that block
	{
		l = FLASH_INVALBLOCK;
		spi_flash_write((uint32_t)blockAddr, (uint32_t *)&l, sizeof(l));	// set "invalid byte" of old block
		blockAddr = s;									// save new block just behind old one
	}
	else
		blockAddr = PARAM_FLASH_START;	// save at start of flash sector

	PRT_DEBUG("updateAllParamsToFlash to %08X\n", (unsigned int)blockAddr);
	i = 0;
	d = (uint8_t*)(blockAddr + 4);					// skip "invalid byte"
	while ((s = (uint32_t)flashParams[i].paraP) != 0)
	{
		spi_flash_write((uint32_t)d, (uint32_t *)s, (uint32_t)flashParams[i].paraSize);
		d += (uint32_t)flashParams[i].paraSize;
		d = (uint8_t*)ROUND4UP(d);
		i++;
	}

	// write CRC to flash
	flashCRC = actualCRC;
	spi_flash_write((uint32_t)d, (uint32_t *)&flashCRC, sizeof(flashCRC));

	return (0);
}



