/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ctype.h"
#include "errno.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event_loop.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "hwcrypto/sha.h"
#include "driver/uart.h"
#include <mdns.h>
#include "driver/spi_master.h"

#define DEBUG		2
#define DEBUG_COLOR _ANSI_WHITE_
#include "appl.h"
#include "nvs_flash.h"

/* changed in MASTER */
/* make a hot fix */

static const char* TAG = "PIT-1";
esp_partition_t operate_partition;

portMUX_TYPE pitschu_spinlock = portMUX_INITIALIZER_UNLOCKED;

//-----------------------------------------------------------------------------------------------------

#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_CLK  GPIO_NUM_14

#define LEDMX_PIN_BUSY     GPIO_NUM_22
#define LEDMX_PIN_CS       GPIO_NUM_25
#define LEDMX_PIN_CONFRST  GPIO_NUM_23			// pull LOW to restart FPGA config
#define LEDMX_PIN_RESET    GPIO_NUM_26			// pull LOW to restart FPGA user code

#define LMX_CMD_PWM_LEDDCR0		0xF8		// 0b10000111		Enable; 125Hz; Mode=LFSR; prescaler high bits=11
#define LMX_CMD_PWM_LEDDBR		0xF9		// 0b11111111		prescaler = 1023 (=64kHz at 64MHz sys clock)
#define LMX_CMD_PWM_LEDDONR		0xFA		// 0b00010000		Blink ON time = 0.5 sec
#define LMX_CMD_PWM_LEDDOFR		0xFB		// 0b00010000		Blink OFF time = 0.5 sec
#define LMX_CMD_PWM_LEDDBCRR	0xF5		// 0b11100001		Breath both edges; 0.256 sec
#define LMX_CMD_PWM_LEDDBCFR	0xF6		// 0b00000000		Breath off
#define LMX_CMD_PWM_LEDDPWRR	0xF1		// RED = 20
#define LMX_CMD_PWM_LEDDPWRG	0xF2		// GREEN = 50
#define LMX_CMD_PWM_LEDDPWRB	0xF3		// BLUE = 50

static uint8_t	pwmInitArray[] = {
		LMX_CMD_PWM_LEDDCR0,	0b10000111,  //		Enable; 125Hz; Mode=LFSR; prescaler high bits=11
		LMX_CMD_PWM_LEDDBR,		0b11111111,	//		prescaler = 1023 (=64kHz at 64MHz sys clock)
		LMX_CMD_PWM_LEDDONR,	0b00010000,	//		Blink ON time = 0.5 sec
		LMX_CMD_PWM_LEDDOFR,	0b00010000, //		Blink OFF time = 0.5 sec
		LMX_CMD_PWM_LEDDBCRR,	0b11100001, //		Breath both edges; 0.256 sec
		LMX_CMD_PWM_LEDDBCFR,	0b00000000, //		Breath off
		LMX_CMD_PWM_LEDDPWRR,	0x20,
		LMX_CMD_PWM_LEDDPWRG,	0x50,
		LMX_CMD_PWM_LEDDPWRB,	0x50,

		0, 0
};


static spi_device_handle_t spiLEDMX;

int LEDMX_SPIinit(void);
void wrReg (uint8_t reg, uint8_t data);
uint8_t rdReg (uint8_t reg);
void SPIwrBurst(uint8_t * data, uint16_t numberByte);
void rdBurst(uint8_t *data, uint16_t numberByte);



typedef struct {
	uint8_t		R;
	uint8_t		G;
	uint8_t		B;
} rgbValue_t;

#define CV	33

rgbValue_t colors[] = {
		{0, 0, 0},
		{CV, 0, 0},
		{0, CV, 0},
		{0, 0, CV},

		{CV, 0, CV},
		{CV, CV, 0},
		{0, CV, CV},
		{CV, CV, CV},
};


void Main_task (void *pvParameters)
{
	int x, y;
	uint8_t R, G, B;
	uint8_t buf[2000];
	uint16_t colorIdx = 0;
	uint16_t pixelDep = 4;

	LOG_DEBUG("Entering MAIN task\n");

	volatile long w;

	buf[0] = 0x81;		// set defaults
	SPIwrBurst(&buf[0], 1);

	x = 0;
	while (pwmInitArray[x])
	{
		SPIwrBurst(&pwmInitArray[x], 2);
		x += 2;
	}

	R = 0; G = 0; B = 0;
	while (1)			// main loop
	{
		buf[0] = 0x82;		// set bit planes (4 ... 8)
		buf[1] = pixelDep;
		SPIwrBurst(&buf[0], 2);

		buf[0] = 0x50;		// set flip rows
		buf[1] = 3;
		buf[2] = 3;
		buf[3] = 0;
		buf[4] = 3;
		SPIwrBurst(&buf[0], 5);

		R = colors[colorIdx].R;
		G = colors[colorIdx].G;
		B = colors[colorIdx].B;
		LOG_DEBUG("Using %d bit planes, colors R=%d, G=%d, B=%d\n", pixelDep, R, G, B);

// fill screen pixel by pixel
		for (y = 0; y < 64; y++)
		{
			for (x = 0; x < 64; x++)
			{
				buf[0] = 0x70;		// set brightness
				buf[1] = y;
				SPIwrBurst(&buf[0], 2);

				buf[0] = 0x01;
				buf[1] = x;
				buf[2] = y;
				buf[3] = R;
				buf[4] = G;
				buf[5] = B;
				SPIwrBurst(&buf[0], 6);
	//			vTaskDelay(1);
				for (w = 0; w < 2000L; w++)
					;
			}
		}

		vTaskDelay(300);
// fill screen from left to right with RGB fill
		for (y = 0; y < 64; y++)
		{
			buf[0] = 0x70;		// set brightness
			buf[1] = y;
			SPIwrBurst(&buf[0], 2);

			buf[0] = 0x02;		// set cursor
			buf[1] = 0x01; // auto inc X
			buf[2] = 5;
			buf[3] = y;
			SPIwrBurst(&buf[0], 4);

			uint8_t *cp = &buf[1];
			buf[0] = 0x03;		// cmd: draw man pixels

			for (x = 0; x < 64; x++)
			{
				*cp++ = 22; *cp++ = y+5; *cp++ = x*2;
			}
			SPIwrBurst(&buf[0], 1 + 3 * 64);

		}

		vTaskDelay(300);
		buf[0] = 0x70;		// set brightness
		buf[1] = 63;
		SPIwrBurst(&buf[0], 2);

		buf[0] = 0x40;		// blank screen
		buf[1] = 0; // R G B
		buf[2] = 1;
		buf[3] = 0;
		SPIwrBurst(&buf[0], 4);
		vTaskDelay(1);

		buf[0] = 0x50;		// set flip rows
		buf[1] = 0;
		buf[2] = 3;
		buf[3] = 0;
		buf[4] = 3;
		SPIwrBurst(&buf[0], 5);

		buf[0] = 0x02;		// set cursor
		buf[1] = 0x03; // auto inc X+y
		buf[2] = 0;
		buf[3] = 0;
		SPIwrBurst(&buf[0], 4);

		buf[0] = 0x04;		// draw diagonal line
		buf[1] = 64; // LEN
		buf[2] = 8; // R G B
		buf[3] = 4;
		buf[4] = 10;
		SPIwrBurst(&buf[0], 5);

		buf[0] = 0x02;		// set cursor
		buf[1] = 0x02; // auto inc X+y
		buf[2] = 32;
		buf[3] = 0;
		SPIwrBurst(&buf[0], 4);

		buf[0] = 0x04;		// draw vertical line
		buf[1] = 64; // LEN
		buf[2] = 10; // R G B
		buf[3] = 8;
		buf[4] = 4;
		SPIwrBurst(&buf[0], 5);

		buf[0] = 0x02;		// set cursor
		buf[1] = 0x01; // auto inc X+y
		buf[2] = 0;
		buf[3] = 32;
		SPIwrBurst(&buf[0], 4);

		buf[0] = 0x04;		// draw horizontal line
		buf[1] = 64; // LEN
		buf[2] = 8; // R G B
		buf[3] = 0;
		buf[4] = 4;
		SPIwrBurst(&buf[0], 5);

		vTaskDelay(600);

		buf[0] = 0x70;		// set brightness
		buf[1] = 63;
		SPIwrBurst(&buf[0], 2);

		buf[0] = 0x40;		// blank screen
		buf[1] = 1; // R G B
		buf[2] = 2;
		buf[3] = 0;
		SPIwrBurst(&buf[0], 4);
		vTaskDelay(1);

		for (y = 0; y < 64; y++)
		{
			buf[0] = 0x70;		// set brightness
			buf[1] = y;
			SPIwrBurst(&buf[0], 2);

			buf[0] = 0x02;		// set cursor
			buf[1] = 0x01; // auto inc X
			buf[2] = 5;
			buf[3] = y;
			SPIwrBurst(&buf[0], 4);

			buf[0] = 0x04;		// draw LEN pixs in direction X
			buf[1] = y/2; // LEN
			buf[2] = 0x22; // R G B
			buf[3] = y+25;
			buf[4] = y*2;
			SPIwrBurst(&buf[0], 5);


			buf[0] = 0x02;		// set cursor
			buf[1] = 0x02; // auto inc X
			buf[2] = y;
			buf[3] = 20;
			SPIwrBurst(&buf[0], 4);

			buf[0] = 0x04;		// draw LEN pixs in direction X
			buf[1] = y/2; // LEN
			buf[2] = 0x22; // R G B
			buf[3] = y+32;
			buf[4] = y*2;
			SPIwrBurst(&buf[0], 5);


			buf[0] = 0x02;		// set cursor
			buf[1] = 0x03; // auto inc X and Y
			buf[2] = y;
			buf[3] = 63-y;
			SPIwrBurst(&buf[0], 4);

			buf[0] = 0x04;		// draw LEN pixs in direction X
			buf[1] = y/3; // LEN
			buf[2] = 0x44; // R G B
			buf[3] = y+8;
			buf[4] = y/2;
			SPIwrBurst(&buf[0], 5);

			vTaskDelay(2);

		}

		vTaskDelay(200);
		buf[0] = 0x70;		// set brightness
		buf[1] = pixelDep *2;
		SPIwrBurst(&buf[0], 2);

		for (x = 0; x < 32; x++)
		{
			buf[0] = 0x50;		// set flip rows
			buf[1] = x;
			buf[2] = 2;
			buf[3] = 2;
			buf[4] = 0;
			SPIwrBurst(&buf[0], 5);

			vTaskDelay(2);
		}
		for (x = 31; x >= 0; x--)
		{
			buf[0] = 0x50;		// set flip rows
			buf[1] = x;
			buf[2] = 0;
			buf[3] = 2;
			buf[4] = 2;
			SPIwrBurst(&buf[0], 5);

			vTaskDelay(2);
		}
		vTaskDelay(300);


		for (x = 0; x < 120; x++)
		{
			buf[0] = 0x70;		// set brightness
			buf[1] = 40;
			SPIwrBurst(&buf[0], 2);

			buf[0] = 0x40;		// blank screen
			buf[1] = 0x22; // R G B
			buf[2] = x+5;
			buf[3] = x*2;
			SPIwrBurst(&buf[0], 4);
			vTaskDelay(1);

			buf[0] = 0x40;		// blank screen
			buf[1] = 0; // R G B
			buf[2] = 5;
			buf[3] = 5;
			SPIwrBurst(&buf[0], 4);
			vTaskDelay(1);
		}

		vTaskDelay(300);

		if (++colorIdx >= 8)
			colorIdx = 0;

		pixelDep++;
		if (pixelDep > 8)
			pixelDep = 4;

	}
}


void app_main()
{
	extern void IRdecoder_Init();
	extern int barometerInit(void);

	ESP_LOGI(TAG, "Appl Main starts after 3 secs delay");
	vTaskDelay(100);

    esp_err_t ret;
    spi_bus_config_t buscfg;

    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 0;		// used default 4K

    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 2);	// DMA seems to work now; use chan 1
    LOG_DEBUG("HSPI-Host inited, ret=%d\n", ret);

    gpio_pullup_en(PIN_NUM_MISO);	// for SD card Data Out pin

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip rev=%d, with %d CPU cores, WiFi%s%s, ",
    		chip_info.revision, chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	strcpy (logTimeBuf, "00:00:00");

    nvs_flash_init();

	LEDMX_SPIinit();

    xTaskCreate(&Main_task, "MAIN", 4000, (void*)NULL, 8, NULL);

}



int LEDMX_SPIinit(void)
{

	uint16_t i;
	gpio_config_t io_conf;
	uint8_t buf[8];
	spi_device_interface_config_t devcfg;

	memset ((void*)&devcfg, 0, sizeof (devcfg));
	devcfg.clock_speed_hz = 10000000L;               //Clock out at 10 MHz
	devcfg.mode = 0;                                //SPI mode 0
	devcfg.spics_io_num = LEDMX_PIN_CS;	               //CS pin
	devcfg.queue_size = 4;
	devcfg.cs_ena_posttrans = 6;		// delay SEL after last bit transfered

	//Attach the LCD to the SPI bus
	int ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spiLEDMX);
	assert(ret==ESP_OK);
	gpio_set_level(LEDMX_PIN_CS, 1);

	io_conf.pin_bit_mask = ((uint64_t)1 << (LEDMX_PIN_BUSY));
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;		// GPIO_PIN_INTR_NEGEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_config(&io_conf);

	io_conf.pin_bit_mask = ((uint64_t)1 << (LEDMX_PIN_CONFRST));	// FPGA configure
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;		// GPIO_PIN_INTR_NEGEDGE;
	io_conf.mode = GPIO_MODE_OUTPUT_OD;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	io_conf.pull_down_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	io_conf.pin_bit_mask = ((uint64_t)1 << (LEDMX_PIN_RESET));	// FPGA user restart
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;		// GPIO_PIN_INTR_NEGEDGE;
	io_conf.mode = GPIO_MODE_OUTPUT_OD;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	io_conf.pull_down_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	gpio_install_isr_service(0);

	gpio_set_level(LEDMX_PIN_CONFRST, 0);
	vTaskDelay(5);
	gpio_set_level(LEDMX_PIN_CONFRST, 1);
	vTaskDelay(5);

	gpio_set_level(LEDMX_PIN_RESET, 0);
	vTaskDelay(5);
	gpio_set_level(LEDMX_PIN_RESET, 1);
	vTaskDelay(5);

	buf[0] = 0x70;		// set brightness
	buf[1] = 20;
	SPIwrBurst(&buf[0], 2);

	buf[0] = 0x82;		// set bit planes (4 ... 8) and matrix orientation
	buf[1] = 7;
	buf[1] = 0;
	SPIwrBurst(&buf[0], 3);

    return 0;
}



void SPIwrBurst(uint8_t * data, uint16_t numberByte)
{
	spi_transaction_t trans;
	int num = numberByte;
	int i;

	while (gpio_get_level(LEDMX_PIN_BUSY) != 0)		// wait while LEDmx is busy with last command
		vTaskDelay(1);

	while (num > 0)
	{
		i = num;

		trans.addr = 0;
		trans.cmd = 0;
		trans.flags = 0;
		trans.user = NULL;
		trans.length = (i) * 8;
		trans.rxlength = 0;
		trans.tx_buffer = data;
		trans.rx_buffer = NULL;

		spi_device_transmit(spiLEDMX, &trans);

		num -= i;
		data += i;
	}
}





void hexDump(const uint8_t *p, int size)
{
	uint16_t i;
	char buf[80];
	char cbuf[20];
	char *cp = &buf[0];
	char *ccp = &cbuf[0];

	strcpy(cp, "0000:");
	cp += strlen(cp);
	for (i = 0; i < size; i++)
	{
		if (i && ((i % 16) == 0))
		{
			if (cp > &buf[0])
			{
				*ccp = '\0';
				LOG_DEBUG("%-54s %s\n", buf, cbuf);
			}
			cp = &buf[0];
			ccp = &cbuf[0];
			sprintf(cp, "%04x:", i);
			cp += strlen(cp);
		}
		*ccp++ = (isprint(*p) ? *p : '.');
		sprintf(cp, " %02x", *p++);
		cp += strlen(cp);
	}
	if (cp > &buf[0])
	{
		*ccp = '\0';
		LOG_DEBUG("%-54s %s\n", buf, cbuf);
	}
}


#if 0

/////////////////////////////////////////////
//      Conways Game of Life
/////////////////////////////////////////////
#define     RGB(r, g, b)                (((r & 0xFF) << 16) | ((g & 0xFF) << 8) | ((b & 0xFF)))
#define 	MAP_888_to_565(c)			(uint16_t)(((c >> 8) & 0xF8) | ((c >> 5) & 0x7E) | ((c >> 3) & 0x1F))
#define 	MAP_565_to_888(c)			(rgb_t)((((rgb_t)(c) & 0xf800) << 8) | ((c & 0x03E0) << 5) | ((c & 0x1f) << 3))

#define     NONE_COLOR                  RGB(0x60, 0x78, 0x58)
#define     BLACK                       RGB(0x00, 0x00, 0x00)
#define     DKGRAY                      RGB(0x40, 0x40, 0x40)
#define     GRAY                        RGB(0x80, 0x80, 0x80)
#define     LTGRAY                      RGB(0xA0, 0xA0, 0xA0)
#define     WHITE                       RGB(0xFF, 0xFF, 0xFF)
#define     DKRED                       RGB(0x60, 0x00, 0x00)
#define     RED                         RGB(0xFF, 0x00, 0x00)
#define     LTRED                       RGB(0xFF, 0x40, 0x40)
#define     DKGREEN                     RGB(0x00, 0x70, 0x00)
#define     GREEN                       RGB(0x00, 0xEF, 0x00)
#define     LTGREEN                     RGB(0x50, 0xFF, 0x50)
#define     DKBLUE                      RGB(0x00, 0x00, 0x60)
#define     BLUE                        RGB(0x10, 0x10, 0xFF)
#define     LTBLUE                      RGB(0x60, 0x60, 0xFF)
#define     DKYELLOW                    RGB(0x80, 0x80, 0x00)
#define     YELLOW                      RGB(0xFF, 0xFF, 0x00)
#define     LTYELLOW                    RGB(0xFF, 0xFF, 0x80)
#define     DKMAGENTA                   RGB(0x60, 0x00, 0x60)
#define     MAGENTA                     RGB(0xFF, 0x00, 0xFF)
#define     LTMAGENTA                   RGB(0xFF, 0x60, 0xFF)
#define     DKCYAN                      RGB(0x00, 0x60, 0x60)
#define     CYAN                        RGB(0x00, 0xB0, 0xB0)
#define     LTCYAN                      RGB(0x60, 0xFF, 0xFF)
#define     GOLD                        RGB(0x90, 0x90, 0x30)

typedef uint32_t	rgb_t;
static rgb_t	gol_ColorLiveCell = RGB(50,255,0);
static rgb_t	gol_ColorDeadCell = RGB(0,0,0);

#define MATRIXSIZE	64

#if MATRIXSIZE == 64
#define   MIN_X               0
#define   MIN_Y               0
#define   MAX_X               63                   // Display Max Koords
#define   MAX_Y               63
#define   MAX_ZELLEN          100                  // Anzahl der zum Start bevölkerten Felder

#define LIMIT_MODS      25
#define MIN_CELLS       50
#define N_NEW_CELLS     10
#elif MATRIXSIZE == 32
#define   MIN_X               0
#define   MIN_Y               0
#define   MAX_X               31                   // Display Max Koords
#define   MAX_Y               31
#define   MAX_ZELLEN          40                  // Anzahl der zum Start bevölkerten Felder

#define LIMIT_MODS      16
#define MIN_CELLS       30
#define N_NEW_CELLS     6
#endif

#define AGE_LEVEL_1     10
#define AGE_LEVEL_2     20
#define AGE_LEVEL_3     40

#define   COL_LIVE_CELL       gol_ColorLiveCell
#define   COL_DEAD_CELL       gol_ColorDeadCell

// Zustände der nächsten Generation
#define DEAD    0
#define ALIVE   1

// 2 Spielfelder alt+neu
struct feld_t
{
	uint8_t last[MAX_X + 1][MAX_Y + 1];
	uint8_t next[MAX_X + 1][MAX_Y + 1];
};

// Pixelfeld wegen Groesse im SDRAM anlegen
struct feld_t feld;

uint32_t cells;

/*
 23/3
 Regel       Geburt:                     = 3 Nachbarn
 Stirbt Einsamkeit:          < 2 Nachbarn
 Stirbt Überbevölkerjung:    > 3 Nachbarn
 Bleibt am Leben:            2 oder 3 Nachbarn
 */


// Setze ein zufälliges Startmuster
static void fill_feld_random()
{
	uint16_t k;
	uint16_t x, y;

	k = MAX_ZELLEN;
	LEDmx_ClearScreen();
	do
	{
		x = GetRandomNumber() % (MAX_X-1);
		y = GetRandomNumber() % (MAX_Y-1);
		if (feld.last[x][y] != ALIVE)
		{
			k--;
			// Markiere Spielfeld
			feld.last[x][y] = ALIVE;
			feld.last[x+1][y] = ALIVE;
			feld.last[x+1][y+1] = ALIVE;
			// Zeichne Pixel auf LCD
		}
	} while (k > 0);
	LEDmx_Picture2PWM();
}



static void redrawLifeScreen(void)
{
	uint16_t x, y;

	for (x = MIN_X; x <= MAX_X; x++)
	{
		for (y = MIN_Y; y <= MAX_Y; y++)
		{
			// Zeiger auf aktuelles Feldelement
			switch (feld.last[x][y])
			{
			case DEAD:
				LEDmx_DrawPixel888(x, y, COL_DEAD_CELL);
				break;
			case ALIVE + 1:
				LEDmx_DrawPixel888(x, y, COL_LIVE_CELL);
				break;
			}
			if (feld.last[x][y] >= AGE_LEVEL_2)
				LEDmx_DrawPixel888(x, y, RGB(255, (AGE_LEVEL_3- feld.last[x][y])*10, 0));
			else if (feld.last[x][y] >= AGE_LEVEL_1)
				LEDmx_DrawPixel888(x, y, RGB(0, (feld.last[x][y] - AGE_LEVEL_1)*10, 255));
		}
	}
	LEDmx_Picture2PWM();
}


// Berechnet die Alterung einer Zelle
static void alterung(uint8_t* ptr, int16_t x, int16_t y)
{
	if ((*ptr)++ >= AGE_LEVEL_3)
	{
		*ptr = DEAD;
		LEDmx_DrawPixel888(x, y, BLACK);
	}
	else if (*ptr >= AGE_LEVEL_2)
	{
		LEDmx_DrawPixel888(x, y, RGB(255, (AGE_LEVEL_3-*ptr)*10, 0));
	}
	else if (*ptr >= AGE_LEVEL_1)
		LEDmx_DrawPixel888(x, y, RGB(0, (*ptr - AGE_LEVEL_1)*10, 255));
}



// Berechne die nächste Generation
static uint16_t calc_nextgen(uint32_t* cells)
{
	int16_t x, y;             // Laufvariablen Nachbarn
	uint16_t xpos, ypos;    // Laufvariablen Feld
	int16_t x1, y1;           // Hilfskoordinaten
	uint16_t changes;
	uint8_t *foo;            // Hilfsvariable

	// Kalkulationsfeld löschen
	memset(feld.next, DEAD, sizeof(feld.next));
	changes = 0;

	// Durchsuche Feld nach LIVE Zellen
	for (xpos = MIN_X; xpos <= MAX_X; xpos++)
	{
		for (ypos = MIN_Y; ypos <= MAX_Y; ypos++)
		{
			if (feld.last[xpos][ypos] >= ALIVE)
			{
				(*cells)++;

				// Ermittle den Zustand der 8 Nachbarn
				for (y = ypos - 1; y <= ypos + 1; y++)
				{
					for (x = xpos - 1; x <= xpos + 1; x++)
					{
						// Koord.transformation auf gegenüber liegenden Seiten
						x1 = x;
						y1 = y;

						if (x < 0)
							x1 = MAX_X;   // x Koordinate
						else if (x > MAX_X)
							x1 = 0;

						if (y < 0)
							y1 = MAX_Y;    // y Koordinate
						else if (y > MAX_Y)
							y1 = 0;

						// Erhöhe Nachbarpunkte um 1
						feld.next[x1][y1]++;
					}
				}
				// Auf Zell Koordinate wieder 1 abziehen
				feld.next[xpos][ypos]--;
			}
		}
	}

	// Aktualisiere das alte Feld mit Informationen
	// aus dem temporären Kalkulationsfeld
	for (x = MIN_X; x <= MAX_X; x++)
	{
		for (y = MIN_Y; y <= MAX_Y; y++)
		{
			// Zeiger auf aktuelles Feldelement
			foo = &feld.last[x][y];

			switch (feld.next[x][y])
			{
			case 2: // Hier Code fuer Zellalterung einfügen
				if (*foo >= ALIVE)
					alterung(foo, x, y);

				break;
			case 3:
				// Zelle war vorher tot und wird lebendig
				if (*foo == DEAD)
				{
					*foo = ALIVE;
					LEDmx_DrawPixel888(x, y, COL_LIVE_CELL);

					changes++;
				}
				// Zelle lebte und bleibt am leben
				else
					alterung(foo, x, y);

				break;
			default:
				// Zelle war vorher lebendig und stirbt
				if (*foo >= ALIVE)
				{
					*foo = DEAD;
					LEDmx_DrawPixel888(x, y, BLACK);
					changes++;
				}
			}
		}
	}

	return (changes);

}

// Prüft, ob Feld nur noch aus unv. Strukturen besteht
/*
 Prinzip:
 Array wird durchsucht aus wieviel verschiedenen Zahlen
 es besteht. Bei > 4 ist die Welt noch im Fluss, bei
 < 4 befinden sich die immer gleichen Muster im Bild

 */

static void check_blinker(uint32_t changes)
{
	uint16_t x, y, i;

	// Hand Gottes: Neue Zellen wahllos entstehen lassen
	if (changes < LIMIT_MODS || cells < MIN_CELLS)
	{
		// Wenn Zellenzahl < LimitMods, dann erzeuge N neue Zellen aus dem Nichts
		for (i = 0; i < N_NEW_CELLS; i++)
		{
			x = GetRandomNumber() % (MAX_X-1);
			y = GetRandomNumber() % (MAX_Y-1);
			feld.last[x][y] = ALIVE;
			feld.last[x+1][y] = ALIVE;
			feld.last[x+1][y+1] = ALIVE;
		}
	}

}




// Hauptroutine
void life(uint16_t cmd)		// 0 = init; >0 -> do N steps
{
	unsigned int changes;

	if (cmd == 0)
	{
		// Feld 1 löschen
		memset(feld.last, DEAD, sizeof(feld.last));
		fill_feld_random();         //  Feld zufaellig bespielen
		return;
	}

	// Spiel laufen lassen

	// Naechste Generation berechnen und zeichnen
	cells = 0;
	changes = calc_nextgen(&cells);
	check_blinker(changes);

//		redrawLifeScreen();
	LEDmx_Picture2PWM();
}

#endif
