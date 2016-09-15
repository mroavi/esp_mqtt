/* IoT
 *
 */

/*
 * SENSORS USED:
 * 	- TSOP4038 	- IR Receiver Module (https://learn.adafruit.com/ir-sensor)
 * 	- INMP401 	- SparkFun MEMS Microphone Breakout (https://www.sparkfun.com/products/9868)
 *  - HC-SR501 	- PIR MOTION DETECTOR (https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor/overview)
 *  - AS6200	- Temperature sensor (http://ams.com/eng/Products/Environmental-Sensors/Temperature-Sensors/AS6200)
 */

/*
 * TROUBLESHOOTING:
 *
 * ERROR: '.irom0.text' will not fit in region `irom0_0_seg'
 * http://bbs.espressif.com/viewtopic.php?f=7&t=1339
 *
 * ANSWER:
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
#include "temp_sensor_as6200.h"

MQTT_Client mqttClient;

/* MQTT QoS */
int mqttQoS = 2;

/* MQTT retain */
int mqttRetain = 0;

/* Buffer that stores the MQTT message (payload) */
char mqttMessage[50] = "";

/* Buffer that stores the MQTT topic string */
char mqttTopic[50] = MQTT_CLIENT_ID"/";

/* List of MQTT subtopics in use */
char soundSubtopic[] 		= "/sound";
char temperatureSubtopic[] 	= "/temperature";
char motionSubtopic[] 		= "/motion";
char distanceSubtopic[]		= "/distance";
char irReceiver[]			= "/ir_receiver";

/*
 * This software timer controls the frequency
 * at which the ADC is read.
 */
static volatile os_timer_t readAdc;

/*
 * This function pointer stores the address of
 * a function that is implemented inside the
 * IR Reciever module. It should be called for
 * every GPIO edge triggered interrupt.
 */
void (*gpioEdgeInterrupt)(void * arg);

/*
 * Publishes an MQTT messages each time that
 * the PIR sensor detects motion.
 */
static void motionDetected(void *arg);

/*
 * This callback gets executed whenever
 * ANY GPIO generates an interrupt.
 * It distinguishes which GPIO generated
 * the interrupt in order to perform the
 * required action.
 */
static void gpioCallback(void *arg);

/*
 * An ADC measurement is made every time this timer triggers.
 * The ADC input is connected to a:
 * SparkFun MEMS Microphone Breakout - INMP401 (ADMP401)
 * https://www.sparkfun.com/products/9868
 */
void readAdcTimerCallback(void *arg);

/*
 *
 */
static void ICACHE_FLASH_ATTR buttonPressedCb( const char * pressedButton, bool reapetedCode);

/*
 *
 */
static void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status);

/*
 * MQTT Callbacks
 */
static void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args);
static void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args);
static void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args);
static void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len);

void ICACHE_FLASH_ATTR print_info();

static void ICACHE_FLASH_ATTR app_init(void);




static void gpioCallback(void *arg)
{
	uint16 gpio_status = 0;

	/* get the GPIO interrupt status register */
	gpio_status = GPIO_REG_READ( GPIO_STATUS_ADDRESS );

	/* clear the status */
	GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, gpio_status);

	/* did GPIO12 (connected to IR receiver) generate the ISR? */
	if( gpio_status & BIT(12) )
	{
#if 0
		INFO("Edge interrupt on GPIO12!\r\n");
#endif
		gpioEdgeInterrupt( arg );
	}

	/* did GPIO 15 (connected to PIR Motion detector) generate the ISR? */
	if( gpio_status & BIT(15) )
	{
#if 0
		INFO("Edge interrupt on GPIO15!\r\n");
#endif
		motionDetected( arg );
	}
}



static void ICACHE_FLASH_ATTR buttonPressedCb( const char * pressedButton, bool reapetedCode)
{
	char * ptr;

	int pressedButtonLen = os_strlen(pressedButton);
	float temperature;	// in Celsius
	int volume; 		// 10 bit resolution
	uint32 motion; 		// 0 -> NO ; 1 -> YES

#if 1
		os_printf("Initial MQTT topic string: %s\r\n", mqttTopic);
#endif

	/* Locate first slash '/' in MQTT topic string */
	ptr = strchr( mqttTopic, '/');

	/* Overwrite what follows the first '/' with topic substring */
	os_strcpy(ptr, irReceiver);

#if 0
		os_printf("mqttTopic starting address: %d\r\n", mqttTopic);
		os_printf("mqttTopic first '/' address: %d\r\n", ptr);
		os_printf("Resulting MQTT topic string: %s\r\n", mqttTopic);
#endif

	/* Publish the MQTT message */
	MQTT_Publish(&mqttClient, mqttTopic, pressedButton, pressedButtonLen, mqttQoS, mqttRetain);
	INFO("MQTT: Publish topic: %s, data: %s, data length: %d\r\n", mqttTopic, pressedButton, pressedButtonLen );

	//-----------------------------------------------------------------------------
	// Temperature
	//-----------------------------------------------------------------------------
	if ( !os_strcmp(pressedButton, "1") )
	{
		temperature = as6200_read_temperature();

		/* Locate the first slash '/' in MQTT topic string */
		ptr = strchr( mqttTopic, '/');

		/* Overwrite from '/' onwards with the topic substring */
		os_strcpy(ptr, temperatureSubtopic);

		/* Copy message to the message buffer (terminating NULL included) */
		os_sprintf(mqttMessage, "Temperature: %d degrees Celsius", (int)temperature);

		/* Publish the MQTT message */
		MQTT_Publish(&mqttClient, mqttTopic, mqttMessage, os_strlen(mqttMessage), mqttQoS, mqttRetain);

#if 1
		os_printf("Temperature: %d\r\n", (int)temperature);
#endif
	}
	//-----------------------------------------------------------------------------
	// Sound
	//-----------------------------------------------------------------------------
	else if ( !os_strcmp(pressedButton, "2") )
	{
		volume = system_adc_read();

		/* Locate the first slash '/' in MQTT topic string */
		ptr = strchr( mqttTopic, '/');

		/* Overwrite from '/' onwards with the topic substring */
		os_strcpy(ptr, soundSubtopic);

		/* Copy message to the message buffer (terminating NULL included) */
		os_sprintf(mqttMessage, "Sound volume: %d (ADC reading)", volume);

		/* Publish the MQTT message */
		MQTT_Publish(&mqttClient, mqttTopic, mqttMessage, os_strlen(mqttMessage), mqttQoS, mqttRetain);

#if 1
		os_printf("Sound volume: %d\r\n", volume);
#endif
	}
	//-----------------------------------------------------------------------------
	// Motion
	//-----------------------------------------------------------------------------
	else if ( !os_strcmp(pressedButton, "3") )
	{
		motion = ( gpio_input_get() & ( (1 << GPIO_ID_PIN(15)) )  ) != 0;

		/* Locate the first slash '/' in MQTT topic string */
		ptr = strchr( mqttTopic, '/');

		/* Overwrite from '/' onwards with the topic substring */
		os_strcpy(ptr, motionSubtopic);

		/* Copy message to the message buffer (terminating NULL included) */
		os_sprintf(mqttMessage, "Motion: %d", motion);

		/* Publish the MQTT message */
		MQTT_Publish(&mqttClient, mqttTopic, mqttMessage, os_strlen(mqttMessage), mqttQoS, mqttRetain);

#if 1
		os_printf("Motion: %d\r\n", motion);
#endif
	}


	return;
}


void readAdcTimerCallback(void *arg)
{
	char * ptr;
	uint16 i;

	/* Current ADC value (volume level of the most recent ADC measurement)*/
	uint16 adc_value;

	/* Previous ADC value (volume level of previous ADC capture)*/
	static uint16 prev_adc_value;

	/* The absolute value of the difference of the current and previous ADC values */
	uint16 abs_val_diff = 0;

	/* Changes of volume below the threshold are ignored */
	static int threshold = 50;

	/* Read ADC */
	adc_value = system_adc_read();

	/* Calculate the absolute value of the previous and current ADC values */
	abs_val_diff = ( adc_value > prev_adc_value ) ? adc_value - prev_adc_value : prev_adc_value - adc_value;

	/* Save adc_value */
	prev_adc_value = adc_value;

	/* Is the measured value larger than the threshold? */
	if ( abs_val_diff >= threshold )
	{
		/* Yes, then publish an MQTT message with the sound volume */

		//TODO: send date and time

		/* Locate the first slash '/' in MQTT topic string */
		ptr = strchr( mqttTopic, '/');

		/* Overwrite from '/' onwards with the topic substring */
		os_strcpy(ptr, soundSubtopic);

		/* Copy message to the message buffer (terminating NULL included) */
		os_sprintf(mqttMessage, "Sound volume change: %d (ADC reading)", abs_val_diff);

		/* Publish the MQTT message */
		MQTT_Publish(&mqttClient, mqttTopic, mqttMessage, os_strlen(mqttMessage), mqttQoS, mqttRetain);

#if 1
		os_printf("Sound volume change: %d\r\n", abs_val_diff);
#endif

		INFO("MQTT: Publish topic: %s, data: %s, data length: %d\r\n", topic, message, messageLen );
	}
}


static void motionDetected(void *arg)
{
	char * ptr;

	/* Locate the first slash '/' in MQTT topic string */
	ptr = strchr( mqttTopic, '/');

	/* Overwrite from '/' onwards with the topic substring */
	os_strcpy(ptr, motionSubtopic);

	/* Copy message to the message buffer (terminating NULL included) */
	os_sprintf(mqttMessage, "Motion detected");

	/* Publish the MQTT message */
	MQTT_Publish(&mqttClient, mqttTopic, mqttMessage, os_strlen(mqttMessage), mqttQoS, mqttRetain);

	INFO("MQTT: Publish topic: %s, data: %s, data length: %d\r\n", mqttTopic, mqttMessage, os_strlen(mqttMessage) );
}


static void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status)
{
  if (status == STATION_GOT_IP)
  {
    MQTT_Connect(&mqttClient);
  }
  else
  {
    MQTT_Disconnect(&mqttClient);
  }
}

static void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args)
{
  MQTT_Client* client = (MQTT_Client*)args;
  INFO("MQTT: Connected\r\n");
//  MQTT_Subscribe(client, "/mqtt/topic/0", 0);
//  MQTT_Subscribe(client, "/mqtt/topic/1", 1);
//  MQTT_Subscribe(client, "/mqtt/topic/2", 2);
//
//  MQTT_Publish(client, "/mqtt/topic/0", "hello0", 6, 0, 0);
//  MQTT_Publish(client, "/mqtt/topic/1", "hello1", 6, 1, 0);
//  MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);
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

	//*****************************************************************************
	//
	// ADC TIMER (based on software timer)
	//
	//*****************************************************************************

	/* disarm timer */
	os_timer_disarm((ETSTimer*)&readAdc);

	/* set callback */
	os_timer_setfn((ETSTimer*)&readAdc, (os_timer_func_t *) readAdcTimerCallback, NULL);

	/* arm the timer -> os_timer_arm(<pointer>, <period in ms>, <fire periodically>) */
	os_timer_arm((ETSTimer*)&readAdc, 10, 1);


	//*****************************************************************************
	//
	// PIR MOTION DETECTOR (based on GPIO edge interrupt)
	//
	//*****************************************************************************

	/* Set GPIO in GPIO mode */
	PIN_FUNC_SELECT( PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15 );

	/* Set GPIO as input */
	GPIO_DIS_OUTPUT( GPIO_ID_PIN(15) );

	gpioEdgeInterrupt = get_function_pointer();

	/* Disable all GPIO interrupts */
	ETS_GPIO_INTR_DISABLE();

	/* Set a GPIO callback function */
	ETS_GPIO_INTR_ATTACH( gpioCallback, NULL );

	/* Trigger GPIO interrupt on rising edge */
	gpio_pin_intr_state_set( GPIO_ID_PIN(15), GPIO_PIN_INTR_POSEDGE );

	/* Enable GPIO interrupts */
	ETS_GPIO_INTR_ENABLE();


	//*****************************************************************************
	//
	// TEMPERATURE SENSOR (uses i2c)
	// https://esp8266hints.wordpress.com/2015/06/04/sdk-i2c-code-todays-duh-story/
	//*****************************************************************************

	as6200_init();
	as6200_config( 0x4060);

	//*****************************************************************************
	//
	// MQTT CLIENT
	//
	//*****************************************************************************

	MQTT_InitConnection(&mqttClient, MQTT_HOST, MQTT_PORT, DEFAULT_SECURITY);
	MQTT_InitClient(&mqttClient, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, MQTT_KEEPALIVE, MQTT_CLEAN_SESSION);
	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);


	//*****************************************************************************
	//
	// WiFi STATION MODE
	//
	//*****************************************************************************

	WIFI_Connect(STA_SSID, STA_PASS, wifiConnectCb);
}

void user_init(void)
{
	system_init_done_cb(app_init);
}
