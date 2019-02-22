/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

 /*!
  * @file bsec_iot_example.ino
  *
  * @brief
  * Example for using of BSEC library in a fixed configuration with the BME680 sensor.
  * This works by running an endless loop in the bsec_iot_loop() function.
  */

  /*!
   * @addtogroup bsec_examples BSEC Examples
   * @brief BSEC usage examples
   * @{*/

   /**********************************************************************************************************************/
   /* header files */
   /**********************************************************************************************************************/

#include "bsec_integration.h"
#include <Wire.h>
#include "OneWireHub.h"
#include "DS18B20.h"  // Digital Thermometer, 12bit

constexpr uint8_t pin_led{LED_BUILTIN};
constexpr uint8_t pin_onewire{D3};

auto hub = OneWireHub(pin_onewire);

auto ds1 = DS18B20(DS18B20::family_code, 0x00, 0x00, 0xB2, 0x18, 0xDA, 0x00);
auto ds2 = DS18B20(DS18B20::family_code, 0x00, 0x00, 0xA2, 0x18, 0xDA, 0x01);
auto ds3 = DS18B20(DS18B20::family_code, 0x00, 0x00, 0x22, 0x18, 0xDA, 0x02);


/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
	Wire.beginTransmission(dev_addr);
	Wire.write(reg_addr);    /* Set register address to start writing to */

	/* Write the data */
	for (int index = 0; index < data_len; index++)
	{
		Wire.write(reg_data_ptr[index]);
	}

	return (int8_t)Wire.endTransmission();
}

/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
	int8_t comResult = 0;
	Wire.beginTransmission(dev_addr);
	Wire.write(reg_addr);                    /* Set register address to start reading from */
	comResult = Wire.endTransmission();

	delayMicroseconds(150);                 /* Precautionary response delay */
	Wire.requestFrom(dev_addr, (uint8_t)data_len);    /* Request data */

	int index = 0;
	while (Wire.available())  /* The slave device may send less than requested (burst read) */
	{
		reg_data_ptr[index] = Wire.read();
		index++;
	}

	return comResult;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
void sleep(uint32_t t_ms)
{
	delay(t_ms);
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
	return (int64_t)millis() * 1000;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
				  float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
				  float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
	Serial.print("[");
	Serial.print(timestamp / 1e6);
	Serial.print("] T: ");
	Serial.print(temperature);
	Serial.print("| rH: ");
	Serial.print(humidity);
	Serial.print("| IAQ: ");
	Serial.print(iaq);
	Serial.print(" (");
	Serial.print(iaq_accuracy);
	Serial.print("| Static IAQ: ");
	Serial.print(static_iaq);
	Serial.print("| CO2e: ");
	Serial.print(co2_equivalent);
	Serial.print("| bVOC: ");
	Serial.println(breath_voc_equivalent);

	ds1.setTemperature(temperature);
	ds2.setTemperature(humidity);
	ds3.setTemperature(iaq);
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
	// ...
	// Load a previous library state from non-volatile memory, if available.
	//
	// Return zero if loading was unsuccessful or no state was available, 
	// otherwise return length of loaded state string.
	// ...
	return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
	// ...
	// Save the string some form of non-volatile memory, if possible.
	// ...
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
	// ...
	// Load a library config from non-volatile memory, if available.
	//
	// Return zero if loading was unsuccessful or no config was available, 
	// otherwise return length of loaded config string.
	// ...
	return 0;
}

bool blinking(void)
{
	constexpr  uint32_t interval = 1000;          // interval at which to blink (milliseconds)
	static uint32_t nextMillis = millis();     // will store next time LED will updated

	if (millis() > nextMillis)
	{
		nextMillis += interval;             // save the next time you blinked the LED
		static uint8_t ledState = LOW;      // ledState used to set the LED
		if (ledState == LOW)    ledState = HIGH;
		else                    ledState = LOW;

		digitalWrite(pin_led, ledState);

		return 1;
	}
	return 0;
}

/* Timestamp variables */
int64_t time_stamp = 0;
int64_t time_stamp_interval_ms = 0;

/* Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs*/
bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];

/* Number of inputs to BSEC */
uint8_t num_bsec_inputs = 0;

/* BSEC sensor settings struct */
bsec_bme_settings_t sensor_settings;

/* Save state variables */
uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
uint8_t work_buffer[BSEC_MAX_STATE_BLOB_SIZE];
uint32_t bsec_state_len = 0;
uint32_t n_samples = 0;

bsec_library_return_t bsec_status = BSEC_OK;

void loop()
{
	hub.poll();

	if (get_timestamp_us() * 1000 >= sensor_settings.next_call)
	{
		bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
	}

	// Blink triggers the state-change
	if (blinking())
	{
		// Set temp
		static float temperature = 20.0;
		temperature += 0.1;
		if (temperature > 120) temperature = 20.0;
		//ds1.setTemperature(temperature - 20);
		//ds2.setTemperature(temperature + 10);
		/*ds3.setTemperature(temperature);*/

		//Serial.println(temperature);
	}
}

/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
void setup()
{
	return_values_init ret;

	/* Init I2C and serial communication */
	Wire.begin();
	Wire.begin(D1, D2); // sda, scl
	Serial.begin(115200);

	hub.attach(ds1);
	hub.attach(ds2);
	hub.attach(ds3);

	/* Call to the function which initializes the BSEC library
	 * Switch on low-power mode and provide no temperature offset */
	ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 5.0f, bus_write, bus_read, sleep, state_load, config_load);
	if (ret.bme680_status)
	{
		/* Could not intialize BME680 */
		Serial.println("Error while initializing BME680");
		return;
	}
	else if (ret.bsec_status)
	{
		/* Could not intialize BSEC library */
		Serial.println("Error while initializing BSEC library");
		return;
	}

	/* Call to endless loop function which reads and processes data based on sensor settings */
	/* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
	//bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
}
