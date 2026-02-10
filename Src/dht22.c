/*******************************************************************************
 * @file        dht22.c
 * @brief       DHT22 (AM2302) temperature and humidity sensor driver
 * @author      Dominik
 *
 * @description
 * Low-level driver implementing the proprietary single-wire communication
 * protocol of the DHT22 digital temperature and humidity sensor.
 *
 * Responsibilities:
 *   - Generate start signal
 *   - Detect sensor response
 *   - Measure pulse widths of incoming data bits
 *   - Read raw 40-bit frame (5 bytes)
 *   - Validate checksum
 *   - Convert raw values to physical units (°C, %RH)
 *
 * Timing:
 *   The protocol requires microsecond-accurate timing. This driver uses
 *   TIM9 configured as a free-running 1 MHz counter (1 µs resolution) to
 *   measure LOW/HIGH pulse durations.
 *
 * Hardware:
 *   - Data line connected to PE5 (bidirectional, open-drain style)
 *   - GPIO direction is switched dynamically (output for start signal,
 *     input for sensor response)
 *
 * Design notes:
 *   - Blocking implementation (polling-based timing)
 *   - Intended for periodic measurements (not continuous streaming)
 *   - Driver is hardware-specific but exposes a clean high-level API
 *
 * Public API:
 *   dht22_read()              -> performs full measurement
 *   dht22_status_to_str()     -> converts status codes to text
 *
 ******************************************************************************/

/* --- INCLUDES --- */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f446xx.h"
#include "dht22.h"

/* --- PRIVATE (STATIC) GLOBALS --- */
static dht22_status_t status;
extern uint8_t error_bit;

/* --- PRIVATE FUNCTION PROTOTYPES --- */
/**
 * @brief  Sends the start signal to the DHT22 sensor.
 *
 * @param  none
 *
 * @return none
 *
 * @note   Pulls the data line low for at least 1ms, then releases it.
 */
static void dht22_start(void);

/**
 * @brief  Waits for the DHT22 sensor's response after the start signal.
 *
 * @param  none
 *
 * @return dht22_status_t  Status code (OK or error).
 *
 * @note   Waits for the sensor to pull the line low and then high, with timeout.
 */
static dht22_status_t dht22_wait_for_response(void);

/**
 * @brief  Reads the raw 5-byte data frame from the DHT22 sensor.
 *
 * @param  frame  Output buffer for 5 bytes of sensor data.
 *
 * @return dht22_status_t  Status code (OK or error).
 *
 * @note   Reads 40 bits (humidity, temperature, checksum) from the sensor.
 */
static dht22_status_t dht22_read_raw(uint8_t frame[5]);

/**
 * @brief  Validates the checksum of the received data frame.
 *
 * @param  frame  Input 5-byte data frame.
 *
 * @return bool   true if checksum is valid, false otherwise.
 *
 * @note   Sums the first 4 bytes and compares to the 5th byte.
 */
static bool dht22_validate_checksum(const uint8_t frame[5]);

/**
 * @brief  Decodes the raw data frame into temperature and humidity values.
 *
 * @param  frame        Input 5-byte data frame.
 * @param  temperature  Output pointer for temperature value (°C).
 * @param  humidity     Output pointer for humidity value (%RH).
 *
 * @return none
 *
 * @note   Converts raw bytes to float values, handles negative temperatures.
 */
static void dht22_decode(const uint8_t frame[5], float *temperature, float *humidity);

/**
 * @brief  Configures PE5 as output with pull-up for DHT22 communication.
 *
 * @param  none
 *
 * @return none
 *
 * @note   Used to drive the data line low during start signal.
 */
static void dht22_set_pin_output(void);

/**
 * @brief  Configures PE5 as input with pull-up for DHT22 communication.
 *
 * @param  none
 *
 * @return none
 *
 * @note   Used to release the data line and allow sensor to drive it.
 */
static void dht22_set_pin_input(void);

/* --- PUBLIC FUNCTION DEFINITIONS --- */

/**
 * @brief  Reads DHT22 sensor data (temperature & humidity).
 *
 * @param  temperature  Pointer to store the measured temperature (°C).
 * @param  humidity     Pointer to store the measured humidity (%RH).
 *
 * @return dht22_status_t   Status code indicating result of the operation.
 *
 * @note   Initiates a read sequence and decodes the result if successful.
 */
dht22_status_t dht22_read(float *temperature, float *humidity)
{
	// validate pointers
	if (temperature == NULL || humidity == NULL)
		return DHT22_ERR_PARAM;

	uint8_t frame[5];
	dht22_start();
	status = dht22_wait_for_response();
	if (status != DHT22_OK)
		return status;
	status = dht22_read_raw(frame);
	if (status != DHT22_OK)
			return status;
	if (!dht22_validate_checksum(frame))
		return DHT22_ERR_CHECKSUM;
	dht22_decode(frame, temperature, humidity);
	return DHT22_OK;
}

/**
 * @brief  Converts a DHT22 status code to a human-readable string.
 *
 * @param  status  Status code to convert.
 *
 * @return const char*  Pointer to a static string describing the status.
 *
 * @note   Useful for UART logging or debugging.
 */
const char* dht22_status_to_str(dht22_status_t status) {
    switch (status) {
        case DHT22_OK:               return "DHT22: OK\r\n";
        case DHT22_ERR_NO_RESPONSE:  return "DHT22 ERROR: No response (Check wiring)\r\n";
        case DHT22_ERR_TIMEOUT:      return "DHT22 ERROR: Communication timeout\r\n";
        case DHT22_ERR_CHECKSUM:     return "DHT22 ERROR: Checksum mismatch (Bad data)\r\n";
        case DHT22_ERR_PARAM:        return "DHT22 ERROR: Invalid parameters passed\r\n";
        default:                     return "DHT22 ERROR: Unknown status\r\n";
    }
}

/* --- PRIVATE FUNCTION DEFINITIONS --- */
static void dht22_start(void)
{
	// start pulse - set SDA low
	dht22_set_pin_output();
	GPIOE->ODR &= ~GPIO_ODR_ODR_5;

	// wait 1000µs
	uint16_t start_cnt = TIM9->CNT;
	while (((TIM9->CNT - start_cnt) & 0xFFFF) < DHT22_START_PERIOD_US)
		;

	// release SDA - configure PE5 as input with Pull-up resistor
	dht22_set_pin_input();
}

static dht22_status_t dht22_wait_for_response(void)
{
	uint16_t start_cnt = TIM9->CNT;
	// Wait for the release of the bus with timeout
	while (GPIOE->IDR & GPIO_IDR_IDR_5)
	{
		if (((TIM9->CNT - start_cnt) & 0xFFFF) > DHT22_RESPONSE_TIMEOUT)
		{
			// timeout reached
			return DHT22_ERR_NO_RESPONSE;
		}
	}

	// Wait for the sensor to respond with ~80 µs LOW state
	start_cnt = TIM9->CNT;
	while (!(GPIOE->IDR & GPIO_IDR_IDR_5))
	{
		if (((TIM9->CNT - start_cnt) & 0xFFFF) > DHT22_RESPONSE_TIMEOUT)
		{
			// timeout reached
			return DHT22_ERR_NO_RESPONSE;
		}
	}
//	uint16_t sensor_response_low = (TIM9->CNT - start_cnt) & 0xFFFF;

	// Wait for the sensor to respond with ~80 µs HIGH state
	start_cnt = TIM9->CNT;
	while (GPIOE->IDR & GPIO_IDR_IDR_5)
	{
		if (((TIM9->CNT - start_cnt) & 0xFFFF) > DHT22_RESPONSE_TIMEOUT)
		{
			// timeout reached
			return DHT22_ERR_NO_RESPONSE;
		}
	}

//	uint16_t sensor_response_high = (TIM9->CNT - start_cnt) & 0xFFFF;

	// validate response signal
//	if (sensor_response_low > DHT22_RESPONSE_LOW_TIME_MIN
//			&& sensor_response_low < DHT22_RESPONSE_LOW_TIME_MAX
//			&& sensor_response_high > DHT22_RESPONSE_HIGH_TIME_MIN
//			&& sensor_response_high < DHT22_RESPONSE_HIGH_TIME_MAX)
//		return 1;
//	else
//		return 0;

	return DHT22_OK;

}

static dht22_status_t dht22_read_raw(uint8_t frame[5])
{
	// validate pointer
	if (frame == NULL)
			return DHT22_ERR_PARAM;

	uint64_t data_bits = 0;
	uint16_t start_cnt;
	uint16_t pulse_width;

	for (uint8_t i = 0; i < 40; i++)
	{
		// 1. Wait for LOW (start of bit)
		start_cnt = TIM9->CNT;
		while (GPIOE->IDR & GPIO_IDR_IDR_5)
		{
			if ((uint16_t) (TIM9->CNT - start_cnt) > DHT22_SIGNAL_LOW_TIME_MAX)
			{
				error_bit = i;
				return DHT22_ERR_TIMEOUT;
			}
		}
		// Wait for HIGH (start of pulse)
		start_cnt = TIM9->CNT;
		while (!(GPIOE->IDR & GPIO_IDR_IDR_5))
		{
			if (((TIM9->CNT - start_cnt)) > DHT22_SIGNAL_LOW_TIME_MAX)
			{
				error_bit = i;
				// timeout reached
				return DHT22_ERR_TIMEOUT;

			}
		}
		// Measure HIGH pulse width
		start_cnt = TIM9->CNT;
		while (GPIOE->IDR & GPIO_IDR_IDR_5)
		{
			// check if the time for the longest bit duration has passed (high state)
			if (((TIM9->CNT - start_cnt)) > DHT22_SIGNAL_1_HIGH_TIME_MAX)
			{
				error_bit = i;
				// timeout reached
				return DHT22_ERR_TIMEOUT;

			}
		}

		pulse_width = (uint16_t) (TIM9->CNT - start_cnt);
		// Store bit (threshold ~50 µs)
		if (pulse_width < DHT22_BIT_THRESHOLD)
			data_bits &= ~(0x1ULL << (39 - i));
		else
			data_bits |= (0x1ULL << (39 - i));
	}

	frame[0] = (data_bits >> 32) & 0xFF;
	frame[1] = (data_bits >> 24) & 0xFF;
	frame[2] = (data_bits >> 16) & 0xFF;
	frame[3] = (data_bits >> 8) & 0xFF;
	frame[4] = (data_bits) & 0xFF;

	return DHT22_OK;
}

static bool dht22_validate_checksum(const uint8_t frame[5])
{
	// validate pointer
	if (frame == NULL)
			return false;

	// Verify checksum
	if ((uint8_t) (frame[0] + frame[1] + frame[2] + frame[3]) != frame[4])
	{
		return false;
	}
	return true;
}

static void dht22_decode(const uint8_t frame[5], float *temperature, float *humidity)
{
	// validate pointers
	if (!frame || !temperature || !humidity)
		return;

	*humidity = ((frame[0] << 8) | frame[1]) / 10.0f;
	// Extract magnitude by masking the sign bit (0x7F = 0111 1111)
	*temperature = (((frame[2] & 0x7F) << 8) | frame[3]) / 10.0f;
	// Apply negative sign if the 8th bit of frame[2] is set
	if (frame[2] & 0x80) {
	    *temperature *= -1.0f;
	}
}

static void dht22_set_pin_output(void)
{
	// set PE5 as output with Pull-up resistor
	GPIOE->MODER &= ~(GPIO_MODER_MODE5_Msk);
	GPIOE->MODER |= (GPIO_MODER_MODE5_0);
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);
	GPIOE->PUPDR |= (GPIO_PUPDR_PUPD5_0);
}

static void dht22_set_pin_input(void)
{
	GPIOE->MODER &= ~(GPIO_MODER_MODE5_Msk);
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);
	GPIOE->PUPDR |= (GPIO_PUPDR_PUPD5_0);
}
