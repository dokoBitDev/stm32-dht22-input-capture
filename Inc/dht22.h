/*******************************************************************************
 * @file        dht22.h
 * @brief       Public interface for DHT22 sensor driver
 * @author      Dominik
 *
 * Provides a simple, hardware-abstracted API for reading temperature and
 * humidity values from a DHT22 sensor.
 *
 * The implementation handles all protocol timing, decoding, and checksum
 * validation internally. The application only interacts with high-level
 * measurement functions and status codes.
 ******************************************************************************/


#ifndef DHT22_H_
#define DHT22_H_

/* --- INCLUDES --- */
#include <stdbool.h>

/* --- DEFINITIONS & MACROS --- */
/* --- PUBLIC CONSTANTS --- */
#define DHT22_START_PERIOD_US			1000	// [µs]
#define DHT22_RESPONSE_TIMEOUT 			300		// [µs]
#define DHT22_RESPONSE_LOW_TIME_MIN		75		// [µs]
#define DHT22_RESPONSE_LOW_TIME_MAX		85		// [µs]
#define DHT22_RESPONSE_HIGH_TIME_MIN	75		// [µs]
#define DHT22_RESPONSE_HIGH_TIME_MAX	85		// [µs]
#define DHT22_BIT_THRESHOLD 			50		// [µs]
#define DHT22_SIGNAL_LOW_TIME			50		// [µs]
#define DHT22_SIGNAL_LOW_TIME_MIN		48		// [µs]
#define DHT22_SIGNAL_LOW_TIME_MAX		55 + 5	// [µs]; added tolerance
#define DHT22_SIGNAL_0_HIGH_TIME		26		// [µs]
#define DHT22_SIGNAL_0_HIGH_TIME_MIN	22		// [µs]
#define DHT22_SIGNAL_0_HIGH_TIME_MAX	30		// [µs]
#define DHT22_SIGNAL_1_HIGH_TIME		70		// [µs]
#define DHT22_SIGNAL_1_HIGH_TIME_MIN	68		// [µs]
#define DHT22_SIGNAL_1_HIGH_TIME_MAX	75 + 5	// [µs]; added tolerance
#define DHT22_SENSOR_RELEASE_BUS		50		// [µs]

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

/* --- PUBLIC FUNCTION PROTOTYPES --- */
/**
 * @brief  Reads DHT22 sensor data (temperature & humidity).
 *
 * Initiates a read sequence with the DHT22 sensor and retrieves
 * temperature and humidity values if successful.
 *
 * @param[out] temperature  Pointer to store the measured temperature (°C).
 * @param[out] humidity     Pointer to store the measured humidity (%RH).
 * @return dht22_status_t   Status code indicating result of the operation.
 */
dht22_status_t dht22_read(float *temperature, float *humidity);

/**
 * @brief  Converts a DHT22 status code to a human-readable string.
 *
 * Useful for UART logging or debugging.
 *
 * @param[in] status    Status code to convert.
 * @return const char*  Pointer to a static string describing the status.
 */
const char* dht22_status_to_str(dht22_status_t status);

#endif /* DHT22_H_ */
