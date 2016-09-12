/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * TROUBLESHOOTING:
 *
 * ERROR: '.irom0.text' will not fit in region `irom0_0_seg'
 * http://bbs.espressif.com/viewtopic.php?f=7&t=1339
 * Howdy, have you tried modifying the eagle.app.v6.ld linker control configuration
 * file and lowering the start location of irom0_0_seg and increasing its length.
 * The default on an Espressif SDK system is only 245,000 bytes out of the possible 512,000.
 * How much flash does your ESP8266 actually have? What are your current values of irom0_0seg origin and length?
 */

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "ir_receiver.h"

static void ICACHE_FLASH_ATTR buttonPressedCb(button_t pressedButton, bool reapetedCode)
{
	switch( pressedButton )
	{
	case CH_MIN:
		INFO("CH-\r\n");
		break;

	case CH:
		INFO("CH\r\n");
		break;

	case CH_PLUS:
		INFO("CH+\r\n");
		break;

	case PREV:
		INFO("PREV\r\n");
		break;

	case NEXT:
		INFO("NEXT\r\n");
		break;

	case PLAY_PAUSE:
		INFO("PLAY/PAUSE\r\n");
		break;

	case VOL_MIN:
		INFO("VOL-\r\n");
		break;

	case VOL_PLUS:
		INFO("VOL+\r\n");
		break;

	case EQ:
		INFO("EQ\r\n");
		break;

	case NUM_0:
		INFO("0\r\n");
		break;

	case NUM_100_PLUS:
		INFO("100+\r\n");
		break;

	case NUM_200_PLUS:
		INFO("200+\r\n");
		break;

	case NUM_1:
		INFO("1\r\n");
		break;

	case NUM_2:
		INFO("2\r\n");
		break;

	case NUM_3:
		INFO("3\r\n");
		break;

	case NUM_4:
		INFO("4\r\n");
		break;

	case NUM_5:
		INFO("5\r\n");
		break;

	case NUM_6:
		INFO("6\r\n");
		break;

	case NUM_7:
		INFO("7\r\n");
		break;

	case NUM_8:
		INFO("8\r\n");
		break;

	case NUM_9:
		INFO("9\r\n");
		break;

	default:
		INFO("Other button\r\n");
	}

	return;
}

MQTT_Client mqttClient;
static void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status)
{
  if (status == STATION_GOT_IP) {
    MQTT_Connect(&mqttClient);
  } else {
    MQTT_Disconnect(&mqttClient);
  }
}

static void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args)
{
  MQTT_Client* client = (MQTT_Client*)args;
  INFO("MQTT: Connected\r\n");
  MQTT_Subscribe(client, "/mqtt/topic/0", 0);
  MQTT_Subscribe(client, "/mqtt/topic/1", 1);
  MQTT_Subscribe(client, "/mqtt/topic/2", 2);

  MQTT_Publish(client, "/mqtt/topic/0", "hello0", 6, 0, 0);
  MQTT_Publish(client, "/mqtt/topic/1", "hello1", 6, 1, 0);
  MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);

}

static void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args)
{
  MQTT_Client* client = (MQTT_Client*)args;
  INFO("MQTT: Disconnected\r\n");
}

static void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args)
{
  MQTT_Client* client = (MQTT_Client*)args;
  INFO("MQTT: Published\r\n");
}

static void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
  char *topicBuf = (char*)os_zalloc(topic_len + 1),
        *dataBuf = (char*)os_zalloc(data_len + 1);

  MQTT_Client* client = (MQTT_Client*)args;
  os_memcpy(topicBuf, topic, topic_len);
  topicBuf[topic_len] = 0;
  os_memcpy(dataBuf, data, data_len);
  dataBuf[data_len] = 0;
  INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
  os_free(topicBuf);
  os_free(dataBuf);
}

void ICACHE_FLASH_ATTR print_info()
{
  INFO("\r\n\r\n[INFO] BOOTUP...\r\n");
  INFO("[INFO] SDK: %s\r\n", system_get_sdk_version());
  INFO("[INFO] Chip ID: %08X\r\n", system_get_chip_id());
  INFO("[INFO] Memory info:\r\n");
  system_print_meminfo();

  INFO("[INFO] -------------------------------------------\n");
  INFO("[INFO] Build time: %s\n", BUID_TIME);
  INFO("[INFO] -------------------------------------------\n");
}


static void ICACHE_FLASH_ATTR app_init(void)
{
  uart_init(BIT_RATE_115200, BIT_RATE_115200);

  print_info();

  ir_receiver_init( buttonPressedCb );

  MQTT_InitConnection(&mqttClient, MQTT_HOST, MQTT_PORT, DEFAULT_SECURITY);
  MQTT_InitClient(&mqttClient, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, MQTT_KEEPALIVE, MQTT_CLEAN_SESSION);
  MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
  MQTT_OnConnected(&mqttClient, mqttConnectedCb);
  MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
  MQTT_OnPublished(&mqttClient, mqttPublishedCb);
  MQTT_OnData(&mqttClient, mqttDataCb);

  WIFI_Connect(STA_SSID, STA_PASS, wifiConnectCb);
}

void user_init(void)
{
  system_init_done_cb(app_init);
}
