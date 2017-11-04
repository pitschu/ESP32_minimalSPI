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
#include "driver/spi_master.h"
#include "soc/soc.h"

#include <mdns.h>


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



void Bouncer(void *param)
{
	int i;

	i = (int)ballColor.R + COL_CHG;
	ballColor.R = putInRange(i, 64, MAX_PWM);
	i = (int)ballColor.G + COL_CHG;
	ballColor.G = putInRange(i, 64, MAX_PWM);
	i = (int)ballColor.B + COL_CHG;
	ballColor.B = putInRange(i, 64, MAX_PWM);

	i = RAND_MAX;

	if (ballDir > 0)
	{
		if (++ballPos >= ledsPhys)
		{
			ballPos = ledsPhys-1;
			ballDir = 0;
		}
		else
		{
			for (i = 0; i < TAIL_LEN; i++)
			{
				int j = ballPos - i;
				if (j >= 0)
				{
					ws2812setLed(j, j, ((int)ballColor.R * tailMult[i]) / 100,
							((int)ballColor.G * tailMult[i]) / 100,
							((int)ballColor.B * tailMult[i]) / 100);
				}
			}
		}
	}
	else
	{
		if (--ballPos <= 0)
		{
			ballPos = 0;
			ballDir = 1;
		}
		else
		{
			for (i = 0; i < TAIL_LEN; i++)
			{
				int j = ballPos + i;
				if (j < ledsPhys)
				{
					ws2812setLed(j, j, ((int)ballColor.R * tailMult[i]) / 100,
							((int)ballColor.G * tailMult[i]) / 100,
							((int)ballColor.B * tailMult[i]) / 100);
				}
			}
		}
	}
}




uint32_t findHTMLpartition()
{
    uint8_t buf[200];

    const esp_partition_t *esp_current_partition = esp_ota_get_boot_partition();

    if (esp_current_partition->type != ESP_PARTITION_TYPE_APP) {
        ESP_LOGE(TAG, "Error esp_current_partition->type is %d != ESP_PARTITION_TYPE_APP", esp_current_partition->type);
        return false;
    }

    esp_partition_t find_partition;

    find_partition.type = 0x50;				// find our first HTML partition
    find_partition.subtype = 0x90;

    const esp_partition_t *partition = esp_partition_find_first(find_partition.type, find_partition.subtype, NULL);
    if (partition == NULL)
    {
    	ESP_LOGE(TAG, "Could not find part type %02x, sub=%02x", find_partition.type, find_partition.subtype);
    	memset (&operate_partition, 0, sizeof(esp_partition_t));
    	return NULL;
    }
//    assert(partition != NULL);
    memcpy(&operate_partition, (void*)partition, sizeof(esp_partition_t));

    ESP_LOGI(TAG, "Found HTML partition");
    ESP_LOGI(TAG, "  label = '%s'", operate_partition.label);
    ESP_LOGI(TAG, "  at addr=%08x, size=%d", operate_partition.address, operate_partition.size);

    spi_flash_read(operate_partition.address, buf, 128);
    hexDump((uint8_t*)buf, 128);

    return operate_partition.address;
}



void Main_task (void *pvParameters)
{
	LOG_DEBUG("Entering MAIN task\n");

	while (1)			// main loop
	{
		gpio_set_level(GPIO_NUM_2, 1);
		vTaskDelay(10);
		gpio_set_level(GPIO_NUM_2, 0);

	}
}


#define TP_PIN_IRQ      GPIO_NUM_35
#define TP_PIN_CS       GPIO_NUM_33

#define TP_CS_HIGH()    {}      // gpio_set_level(TP_PIN_CS, 1); used only when GPIO_0 is used as SEL pin
#define TP_CS_LOW()     {}      //gpio_set_level(TP_PIN_CS, 0)

static spi_device_handle_t spiTP;


static uint16_t WR_CMD(uint8_t cmd)
{
    uint16_t buf;
    spi_transaction_t trans;

    trans.addr = 0;
    trans.cmd = 0;
    trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans.user = NULL;
    trans.length = 3 * 8;
    trans.rxlength = 0;

    trans.tx_data[0] = cmd;
    trans.tx_data[1] = 0;
    trans.tx_data[2] = 0;

    TP_CS_LOW();
    spi_device_transmit(spiTP, &trans);
    TP_CS_HIGH();

//    spi_device_queue_trans(spiTP, &trans, 1000/portTICK_PERIOD_MS);
//    spi_device_get_trans_result(spiTP, &trans, 1000/portTICK_PERIOD_MS);
    buf = (((uint16_t) trans.rx_data[1] << 8) | ((uint16_t) trans.rx_data[2])) >> 3;

    if (trans.rx_data[1] || trans.rx_data[2])
    {
        LOG_DEBUG("WR cmd=%02x: val=%d\n", cmd, (int)buf);
    }

    return buf;
}


/* Send multiple byte */
static void  xmit_spi_multi (const BYTE *buff,  /* Pointer to the data */
                                UINT btx)           /* Number of bytes to send (even number) */
{
    uint8_t *cp = (uint8_t*)buff;
    spi_transaction_t trans;
    spi_transaction_t *recv;
    uint32_t num = btx;
    int i;

    while (num > 0)
    {
        if (num > 32)   // ESP32 can transfer only 32 bytes if DMA is not used
            i = 32;
        else
            i = num;

        trans.addr = 0;
        trans.cmd = 0;
        trans.flags = 0;
        trans.user = NULL;
        trans.length = (i) * 8;
        trans.rxlength = 0;
        trans.tx_buffer = cp;
        trans.rx_buffer = NULL;

        int ret = spi_device_queue_trans(spiSDCARD, &trans, 10);
        if (ret!=ESP_OK)
        {
            PRT_DEBUG("SPI xmit_spi_multi: spi_device_queue_trans ret=%d\n", ret);
        }

        ret = spi_device_get_trans_result(spiSDCARD, &recv, 10);
        if (ret!=ESP_OK)
        {
            PRT_DEBUG("SPI xmit_spi_multi: spi_device_get_trans_result ret=%d\n", ret);
        }

        num -= i;
        cp += i;
    }
}




void app_main()
{
	ESP_LOGI(TAG, "Appl Main starts after 3 secs delay");
	vTaskDelay(300);
#if 1  /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
#endif
	strcpy (logTimeBuf, "00:00:00");

    nvs_flash_init();

	Flash_Init ();

    xTaskCreate(&Main_task, "MAIN", 4000, (void*)NULL, 8, NULL);

	uint8_t *fs = (uint8_t*)findHTMLpartition();
	ESP_LOGI(TAG, "file sys @%p", fs);
//	espFsInit((void*)((uint32_t)fs));

    gpio_config_t io_conf;

    io_conf.intr_type = 	GPIO_PIN_INTR_DISABLE;
    io_conf.mode = 			GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 	GPIO_SEL_2;
    io_conf.pull_down_en = 	0;
    io_conf.pull_up_en = 	0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_2, 0);
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



int otherMacroExpand(HttpdConnData *connData, char *token, void **arg)
{
	char buff[200];

	if (token == NULL)
		return HTTPD_CGI_DONE;

	strcpy(buff, "");

	if (strcmp(token, "devname") == 0)
	{
		strcpy(buff, deviceName);
	}
	else if (strcmp(token, "MCpage") == 0)
	{
		strcpy(buff, "-");
	}
	else if (strcmp(token, "MLpage") == 0)
	{
		strcpy(buff, "-");
	}


	LOG_DEBUG("expand token '%s' to '%s'\n", token, buff);

	if (*buff)
		httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}




