/*******************************************************************************
 * @file        main.c
 * @brief       Simple weather station using the DHT22 (AM2302) sensor.
 * @author      Dominik
 *
 * @description
 * This application runs on the STM32F446ZE microcontroller and periodically
 * measures temperature and humidity using a DHT22 digital sensor.
 *
 * The DHT22 uses a proprietary single-wire protocol that requires precise
 * microsecond timing. Timing is generated using an STM32 general-purpose
 * timer configured as a free-running microsecond counter (TIM9).
 *
 * A second timer (TIM6) generates a periodic interrupt every 5 seconds to
 * trigger a new measurement. After each successful read, the decoded values
 * are transmitted to a PC over USART3 via the ST-Link Virtual COM port.
 *
 * Program flow:
 *   - TIM6 interrupt sets a measurement flag
 *   - main loop calls the DHT22 driver
 *   - driver performs protocol timing using TIM9
 *   - results are validated (checksum) and printed via UART
 *
 ******************************************************************************/

/**
 ******************************************************************************
 * Pin Mapping (STM32 NUCLEO-F446ZE):
 *
 * PE5   DHT22 data line (single-wire, bidirectional) - SDA
 * PD8   USART3_TX (ST-Link VCP)
 * PD9   USART3_RX (ST-Link VCP)
 * PB0   LD1 (status LED)
 ******************************************************************************
 */

/**
 ******************************************************************************
 * Timer configuration:
 *
 * TIM6
 *   Periodic 5 s interrupt used to schedule sensor measurements.
 *
 * TIM9
 *   Free-running 1 MHz counter (1 µs resolution) used for precise timing
 *   of the DHT22 protocol.
 ******************************************************************************
 */

/**
 ******************************************************************************
 * UART Configuration:
 *
 * - USART3 is used for communication with the PC.
 * - USART3 is connected via the ST-Link Virtual COM Port.
 * - Commands are ASCII strings terminated with '\n'.
 * - Maximum command length: 10 characters.
 *
 * Clock source:
 * - USART3 is clocked from the APB1 peripheral clock (PCLK1).
 * - On NUCLEO-F446ZE (default): USART3 clock = 16 MHz
 *
 * Pin Configuration:
 * - PD8   USART3_TX
 * - PD9   USART3_RX
 *
 * UART parameters:
 * - Baud rate: 115200
 * - Data bits: 8
 * - Parity: None
 * - Stop bits: 1
 * - Hardware flow control: None
 ******************************************************************************
 */

/* --- INCLUDES --- */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "stm32f446xx.h"
#include "main.h"
#include "clock.h"
#include "dht22.h"

/* --- DEFINITIONS & MACROS --- */
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* --- GLOBAL VARIABLES --- */
static void uart_send_str(const char *s);
volatile bool dht22_read_flag = false;
volatile uint8_t error_bit;

/* --- FUNCTION PROTOTYPES --- */

/**
 * @brief  Initializes the system clock and peripheral clocks.
 * @note   Configures clocks for GPIO, TIM6, TIM9, USART3, etc.
 */
void Clock_Init(void);

/**
 * @brief  Initializes GPIO pins for DHT22 and LEDs.
 * @note   Sets PE5 for DHT22, configures onboard LEDs.
 */
void GPIO_init(void);

/**
 * @brief  Initializes USART3 for UART communication.
 * @note   Configures PD8/PD9 for USART3, sets baud rate, enables interrupts.
 */
void USART3_init(void);

/**
 * @brief  Initializes TIM6 for periodic 5s interrupts.
 * @note   Used to trigger DHT22 measurements.
 */
void TIM6_init(void);

/**
 * @brief  Initializes TIM9 as a free-running microsecond timer.
 * @note   Used for DHT22 protocol timing.
 */
void TIM9_init(void);

/**
 * @brief  TIM6 global interrupt handler.
 * @note   Sets flag for DHT22 measurement, toggles LED, clears interrupt.
 */
void TIM6_DAC_IRQHandler(void);

/**
 * @brief  TIM1 Break and TIM9 global interrupt handler.
 * @note   Currently not used.
 */
void TIM1_BRK_TIM9_IRQHandler(void);

/**
 * @brief  USART3 global interrupt handler.
 * @note   Currently not used.
 */
void USART3_IRQHandler(void);

/* --- MAIN FUNCTION --- */
int main(void)
{
	Clock_Init();
	GPIO_init();
	USART3_init();
	TIM6_init();
	TIM9_init();

	// start periodic timer -> counter enable
	TIM6->CR1 |= TIM_CR1_CEN;

	uart_send_str("================================\r\n");
	uart_send_str("DHT22\r\n");
	uart_send_str("================================\r\n");
	/* Loop forever */
	for (;;)
	{
		if (dht22_read_flag)
		{
			dht22_read_flag = false;

			float temperature = 0.0f, humidity = 0.0f;
			dht22_status_t status = dht22_read(&temperature, &humidity);

			char msg[64];
			if (status == DHT22_OK)
			{
				// send data over UART
				sprintf(msg, "Temperature: %.1f C, Humidity: %.1f%%\r\n", temperature, humidity);
				uart_send_str(msg);
			}
			else
			{
				// Send error message over UART
				snprintf(msg, sizeof(msg), "%s", dht22_status_to_str(status));
				uart_send_str(msg);
				// handle error (optional)
			}
		}
	}
}

void Clock_Init(void)
{
// Enable clock for GPIOE on AHB1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	// Enable clock for TIM6 on APB1
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Enable clock for TIM9 on APB2
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

	// enable USART3 clock on APB1
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	// Enable clock for GPIOB on AHB1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// Enable clock for GPIOD on AHB1 (needed for USART3 pins PD8/PD9)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
}
void GPIO_init(void)
{
// set PE5 as output with Pull-up resistor
	GPIOE->MODER &= ~(GPIO_MODER_MODE5_Msk);
	GPIOE->MODER |= (GPIO_MODER_MODE5_0);
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);
	GPIOE->PUPDR |= (GPIO_PUPDR_PUPD5_0);

//	// set PE5 as AF3 --> TIM9_CH1
//	GPIOE->MODER &= ~(GPIO_MODER_MODE5_Msk);
//	GPIOE->MODER |= (GPIO_MODER_MODE5_1);
//	GPIOE->AFR[0] &= ~(GPIO_AFRL_AFRL5);
//	GPIOE->AFR[0] |= (0x3UL << GPIO_AFRL_AFSEL5_Pos);

// Set integrated LEDs as output
	GPIOB->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE7 | GPIO_MODER_MODE14);	// clear bits
	GPIOB->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE7_0 | GPIO_MODER_MODE14_0);// 01: General purpose output mode

// Ensure LEDs are off initially
	GPIOB->ODR &= ~(GPIO_ODR_OD0 | GPIO_ODR_OD7 | GPIO_ODR_OD14);
}
void USART3_init(void)
{
// Set PD8 & PD9 as AF pins (10: Alternate function mode)
	GPIOD->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9); // clear bits
	GPIOD->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1); 	// PD8/PD9 -> AF
// configure AF7 (USART3) for PD8 (AFRH nibble 0) and PD9 (AFRH nibble 1)
	GPIOD->AFR[1] &= ~(GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1);		// clear bits
	GPIOD->AFR[1] |= (0x7UL << GPIO_AFRH_AFSEL8_Pos | 0x7UL << GPIO_AFRH_AFSEL9_Pos);

// baud: 115200	bps; for PCLK1=16MHz USARTDIV=8.6875
	USART3->BRR = (0x8U << 4U) | 0xBU;

// Oversampling mode
	USART3->CR1 &= ~USART_CR1_OVER8;	// 0: oversampling by 16
// enable USART
	USART3->CR1 |= USART_CR1_UE;		// 1: USART enabled
// word length
	USART3->CR1 &= ~USART_CR1_M;		// 0: 1 Start bit, 8 Data bits, n Stop bit
// Parity control
	USART3->CR1 &= ~USART_CR1_PCE;		// 0: Parity control disabled
// STOP bits
	USART3->CR2 &= ~USART_CR2_STOP;		// 00: 1 Stop bit
// Transmitter enable
	USART3->CR1 |= USART_CR1_TE;		// 1: Transmitter is enabled
// Enable USART3 RX (for completeness, even if not used)
	USART3->CR1 |= USART_CR1_RE; // 1: Receiver is enabled

	// If not using interrupts for RX/TX, do not enable IRQ
	// NVIC_EnableIRQ(USART3_IRQn);

	NVIC_EnableIRQ(USART3_IRQn);
}

void TIM6_init(void)
{
// get APB1 clock speed
	uint32_t pclk1 = get_pclk1_hz();

// Prescaler value
	TIM6->PSC = (pclk1 / 1000) - 1;	// get TIM counter clock at 1 KHz -> 1 ms period

// generate interrupt every 5s
	TIM6->ARR = APP_MEASUREMENT_PERIOD_MS - 1;	// Auto-reload value
// Update interrupt enable
	TIM6->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void TIM9_init(void)
{
	// get APB2 clock speed
	uint32_t pclk2 = get_pclk2_hz();

	// get timer clock speed
	uint32_t tim9_clk = get_timxclk_from_pclk(pclk2);

	// Prescaler value
	TIM9->PSC = (tim9_clk / 1000000) - 1;	// get TIM counter clock at 1 MHz -> 1 µs period

	// set large ARR
	TIM9->ARR = 0xFFFF;

	// enable counter
	TIM9->CR1 |= TIM_CR1_CEN;
}

// TIM6 global interrupt, DAC1 and DAC2 underrun error interrupt
void TIM6_DAC_IRQHandler(void)
{
// set flag
	dht22_read_flag = true;

// toggle integrated LED LD1 (green)
	GPIOB->ODR ^= GPIO_ODR_OD0;

// clear Update interrupt flag
	TIM6->SR &= ~(TIM_SR_UIF);
}

// TIM1 Break interrupt and TIM9 global interrupt
//void TIM1_BRK_TIM9_IRQHandler(void)
//{
//
//}

// USART3 global interrupt
//void USART3_IRQHandler(void)
//{
//
//}

// Blocking transmit helper
static void uart_send_str(const char *s)
{
	if (!s)
		return; // Null pointer check
	while (*s)
	{
		// Wait for TXE (polling, blocking)
		while (!(USART3->SR & USART_SR_TXE));
		USART3->DR = (uint8_t) (*s++);
	}
	// Wait for TC (optional, ensures all data sent)
	while (!(USART3->SR & USART_SR_TC));
}

