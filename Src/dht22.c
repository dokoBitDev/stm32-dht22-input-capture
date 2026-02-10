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

/* --- ISR State Machine States --- */
typedef enum
{
	IDLE = 0,
	WAIT_RESPONSE_LOW,
	WAIT_RESPONSE_HIGH,
	READ_BITS,
	ERROR
} dht22_isr_state_t;

/* --- PRIVATE (STATIC) GLOBALS --- */
static volatile dht22_status_t dht22_status;
static volatile dht22_isr_state_t dht22_state = IDLE;
static volatile uint64_t dht22_raw_data = 0;
static uint16_t last_timestamp = 0;

extern volatile bool dht22_done_flag;

/* --- PRIVATE FUNCTION PROTOTYPES --- */

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
 * @brief  Configures PE5 as output with pull-up for DHT22 communication
 *
 * @param  none
 *
 * @return none
 *
 * @note   Used to drive the data line low during start signal
 */
static void dht22_set_pin_output(void);

/**
 * @brief  Configures PE5 as Alternate Function (AF3)
 *
 * @param  none
 *
 * @return none
 *
 * @note   TODO: Used to configure the TIM9_CH1 as input capture on AF3 on PE5
 */
static void dht22_set_pin_AF3(void);

/* --- PUBLIC FUNCTION DEFINITIONS --- */
void dht22_init(void)
{
	// No initialization required for now
	dht22_state = IDLE;
	dht22_status = DHT22_OK;
	dht22_raw_data = 0;
	last_timestamp = 0;
	dht22_done_flag = false;
}

dht22_start_status_t dht22_start_read(void)
{
	if (dht22_state != IDLE)
		return DHT22_START_BUSY;

	dht22_raw_data = 0;
	dht22_status = DHT22_OK;
	dht22_done_flag = false;
	dht22_state = WAIT_RESPONSE_LOW;

	// Generate DHT22 start pulse (~1 ms LOW)
	dht22_set_pin_output();
	GPIOE->ODR &= ~GPIO_ODR_ODR_5;
	uint16_t start_cnt = TIM9->CNT;
	while ((TIM9->CNT - start_cnt) < DHT22_START_PERIOD_US);

	// Release bus (HIGH) before switching to alternate function
	GPIOE->ODR |= GPIO_ODR_ODR_5;
	dht22_set_pin_AF3();

	// Configure TIM9 polarity (capture first falling edge)
	TIM9->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
	TIM9->CCER |= TIM_CCER_CC1P;

	TIM9->CNT = 0;
	TIM9->SR = 0;
	last_timestamp = 0;

	// Enable CC1 capture
	TIM9->CCER |= TIM_CCER_CC1E;

	return DHT22_START_OK;
}

void dht22_tim9_isr(void)
{
	static uint64_t raw_frame = 0;
	static uint8_t bit_no = 0;
	static bool skip_first_edge = false; // NEW: flag to skip dummy edge

	/* Only handle CC1 capture events */
	if (!(TIM9->SR & TIM_SR_CC1IF))
		return;

	TIM9->SR &= ~TIM_SR_CC1IF; // Clear flag to avoid re-triggering

	uint16_t timestamp = TIM9->CCR1;
	uint16_t pulse_width = timestamp - last_timestamp;
	last_timestamp = timestamp;

	bool falling = (TIM9->CCER & TIM_CCER_CC1P);

	switch (dht22_state)
	{
	case IDLE:
		TIM9->CCER &= ~TIM_CCER_CC1E;
		break;

	case WAIT_RESPONSE_LOW:
		if (!falling)
		{
			dht22_status = DHT22_ERR_NO_RESPONSE;
			dht22_state = ERROR;
			break;
		}
		dht22_state = WAIT_RESPONSE_HIGH;
		break;

	case WAIT_RESPONSE_HIGH:
		if (falling)
		{
			dht22_status = DHT22_ERR_TIMEOUT;
			dht22_state = ERROR;
			break;
		}
		// validate RESPONSE_LOW pulse
		if (pulse_width < DHT22_RESPONSE_LOW_TIME_MIN || pulse_width > DHT22_RESPONSE_LOW_TIME_MAX)
		{
			dht22_status = DHT22_ERR_TIMEOUT;
			dht22_state = ERROR;
			break;
		}
		raw_frame = 0;
		bit_no = 0;
		skip_first_edge = true; // NEW: skip the first edge in READ_BITS
		dht22_state = READ_BITS;
		break;

	case READ_BITS:
		// The first edge after entering READ_BITS is the end of the response high pulse (dummy edge)
		if (skip_first_edge)
		{
			skip_first_edge = false;
			break; // Ignore this edge, do not process as data bit
		}
		// rising edge
		if (!falling)
		{
			/* rising edge → LOW period */
			if (pulse_width < DHT22_SIGNAL_LOW_TIME_MIN || pulse_width > DHT22_SIGNAL_LOW_TIME_MAX)
			{
				dht22_status = DHT22_ERR_TIMEOUT;
				dht22_state = ERROR;
			}
			break;
		}
		// Only process if we have not yet collected 40 bits
		if (bit_no < 40)
		{
			if (pulse_width > DHT22_BIT_THRESHOLD)
				raw_frame |= (1ULL << (39 - bit_no));
			bit_no++;
		}

		if (bit_no == 40)
		{
			TIM9->CCER &= ~TIM_CCER_CC1E;	// disable CC1 capture
			dht22_raw_data = raw_frame; 	// global result
			dht22_status = DHT22_OK;
			dht22_done_flag = true;
			dht22_state = IDLE;
			return;
		}
		break;

	case ERROR:
		// disable CC1 capture
		TIM9->CCER &= ~TIM_CCER_CC1E;
		dht22_done_flag = true;
		dht22_state = IDLE;
		return;
	}
	// Toggle edge polarity for next capture
	TIM9->CCER ^= TIM_CCER_CC1P;
}

dht22_status_t dht22_get_result(float *temperature, float *humidity)
{
	if (!temperature || !humidity)
		return DHT22_ERR_PARAM;

	uint8_t frame[5];
	frame[0] = (dht22_raw_data >> 32) & 0xFF;
	frame[1] = (dht22_raw_data >> 24) & 0xFF;
	frame[2] = (dht22_raw_data >> 16) & 0xFF;
	frame[3] = (dht22_raw_data >> 8) & 0xFF;
	frame[4] = (dht22_raw_data) & 0xFF;

	if (!dht22_validate_checksum(frame))
		return DHT22_ERR_CHECKSUM;
	dht22_decode(frame, temperature, humidity);
	return DHT22_OK;
}

const char* dht22_status_to_str(dht22_status_t status)
{
	switch (status)
	{
	case DHT22_OK:
		return "DHT22: OK\r\n";
	case DHT22_ERR_NO_RESPONSE:
		return "DHT22 ERROR: No response (Check wiring)\r\n";
	case DHT22_ERR_TIMEOUT:
		return "DHT22 ERROR: Communication timeout\r\n";
	case DHT22_ERR_CHECKSUM:
		return "DHT22 ERROR: Checksum mismatch (Bad data)\r\n";
	case DHT22_ERR_PARAM:
		return "DHT22 ERROR: Invalid parameters passed\r\n";
	default:
		return "DHT22 ERROR: Unknown status\r\n";
	}
}

/* --- PRIVATE FUNCTION DEFINITIONS --- */
static bool dht22_validate_checksum(const uint8_t frame[5])
{
	if (!frame)
		return false;
	if ((uint8_t) (frame[0] + frame[1] + frame[2] + frame[3]) != frame[4])
		return false;
	return true;
}

static void dht22_decode(const uint8_t frame[5], float *temperature, float *humidity)
{
	if (!frame || !temperature || !humidity)
		return;

	*humidity = ((frame[0] << 8) | frame[1]) / 10.0f;
// Extract magnitude by masking the sign bit (0x7F = 0111 1111)
	*temperature = (((frame[2] & 0x7F) << 8) | frame[3]) / 10.0f;
// Apply negative sign if the 8th bit of frame[2] is set
	if (frame[2] & 0x80)
		*temperature *= -1.0f;
}

static void dht22_set_pin_output(void)
{
	GPIOE->MODER &= ~(GPIO_MODER_MODE5_Msk);
	GPIOE->MODER |= (GPIO_MODER_MODE5_0);
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);
	GPIOE->PUPDR |= (GPIO_PUPDR_PUPD5_0);
}

static void dht22_set_pin_AF3(void)
{
	GPIOE->MODER &= ~(GPIO_MODER_MODE5_Msk);
	GPIOE->MODER |= (GPIO_MODER_MODE5_1);
	GPIOE->AFR[0] &= ~(GPIO_AFRL_AFRL5);
	GPIOE->AFR[0] |= (0x3UL << GPIO_AFRL_AFSEL5_Pos);
}
