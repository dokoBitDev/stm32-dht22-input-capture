/*******************************************************************************
 * @file        dht22.h
 * @brief       Public interface for DHT22 sensor driver
 * @author      Dominik
 ******************************************************************************/

#ifndef DHT22_H_
#define DHT22_H_

/* --- INCLUDES --- */
#include <stdbool.h>

/* --- DEFINITIONS & MACROS --- */
/* --- PUBLIC CONSTANTS --- */
// all in microseconds (µs)
#define DHT_TIME_TOLERANCE				10
#define DHT22_START_PERIOD_US			1000
#define DHT22_RESPONSE_LOW_TIME_MIN		(75 - DHT_TIME_TOLERANCE)
#define DHT22_RESPONSE_LOW_TIME_MAX		(85 + DHT_TIME_TOLERANCE)
#define DHT22_RESPONSE_HIGH_TIME_MIN	(75 - DHT_TIME_TOLERANCE)
#define DHT22_RESPONSE_HIGH_TIME_MAX	(85 + DHT_TIME_TOLERANCE)
#define DHT22_BIT_THRESHOLD 			50
#define DHT22_SIGNAL_LOW_TIME_MIN		(45 - DHT_TIME_TOLERANCE)
#define DHT22_SIGNAL_LOW_TIME_MAX		(55 + DHT_TIME_TOLERANCE)

/* --- PUBLIC TYPE DEFINITIONS --- */
/**
 * @brief Result codes for DHT22 sensor operations.
 */
typedef enum
{
	DHT22_OK = 0,
	DHT22_ERR_NO_RESPONSE,
	DHT22_ERR_TIMEOUT,
	DHT22_ERR_CHECKSUM,
	DHT22_ERR_PARAM
} dht22_status_t;

/**
 * @brief status of the DHT22 start function
 */
typedef enum
{
	DHT22_START_OK = 0,
	DHT22_START_BUSY
} dht22_start_status_t;

/* --- PUBLIC VARIABLES --- */
/* --- PUBLIC FUNCTION PROTOTYPES --- */

/**
 * @brief  Initialize DHT22 driver state.
 *
 * @param	none
 * @return 	none
 */
void dht22_init(void);

/**
 * @brief  Start a DHT22 measurement (non-blocking).
 *
 *	set PE5 as GPIO output, generates a 1ms start signal on PE5, releases bus,
 *	switches PE5 to AF3, reset TIM9 counter, clear capture flags,
 *	enable TIM9 capture interrupt, set internal state to WAIT_RESPONSE
 *
 * @param	none
 * @return DHT22_START_OK if started, DHT22_START_BUSY if busy.
 */
dht22_start_status_t dht22_start_read(void);

/**
 * @brief  DHT22 TIM9 input capture ISR handler.
 *
 * @param	none
 * @return 	none
 */
void dht22_tim9_isr(void);

/**
 * @brief  Get the result of the last DHT22 measurement.
 * @param[out] temperature  Pointer to store temperature (°C).
 * @param[out] humidity     Pointer to store humidity (%RH).
 * @return dht22_status_t   Status code.
 */
dht22_status_t dht22_get_result(float *temperature, float *humidity);

/**
 * @brief  Convert DHT22 status code to human-readable string.
 * @param[in] status    Status code.
 * @return const char*  Static string.
 */
const char* dht22_status_to_str(dht22_status_t status);

#endif /* DHT22_H_ */
