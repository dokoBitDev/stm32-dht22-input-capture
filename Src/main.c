/*******************************************************************************
 * @file        main.c
 * @brief       Simple weather station using the DHT22 (AM2302) sensor.
 * @author      Dominik
 *
 * @description
 * TODO
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
 *   TODO
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
volatile bool dht22_req_flag = false;
volatile bool dht22_done_flag = false;

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
 * @brief  Initializes TIM9 in input capture mode.
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
 * @note   TODO.
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
	dht22_init();

	// Start periodic timer
	TIM6->CR1 |= TIM_CR1_CEN;

	uart_send_str("================================\r\n");
	uart_send_str("DHT22\r\n");
	uart_send_str("================================\r\n");

	for (;;)
	{
		if (dht22_req_flag)
		{
			dht22_req_flag = false;
			// Send start pulse to DHT22
			if (dht22_start_read() == DHT22_START_OK)
			{
				// Measurement started, wait for completion in main loop
			}
		}

		if (dht22_done_flag)
		{
			dht22_done_flag = false;
			float temperature = 0.0f, humidity = 0.0f;
			dht22_status_t status = dht22_get_result(&temperature, &humidity);

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
	// Enable clocks for GPIOE, GPIOB, GPIOD, TIM6, TIM9, USART3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
}

void GPIO_init(void)
{
	// PE5: DHT22 data line as output with pull-up
	GPIOE->MODER &= ~(GPIO_MODER_MODE5_Msk);
	GPIOE->MODER |= (GPIO_MODER_MODE5_0);
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);
	GPIOE->PUPDR |= (GPIO_PUPDR_PUPD5_0);

	// PB0, PB7, PB14: Onboard LEDs as output
	GPIOB->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE7 | GPIO_MODER_MODE14);
	GPIOB->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE7_0 | GPIO_MODER_MODE14_0);

	// Ensure LEDs are off initially
	GPIOB->ODR &= ~(GPIO_ODR_OD0 | GPIO_ODR_OD7 | GPIO_ODR_OD14);
}

void USART3_init(void)
{
	// PD8 & PD9 as AF7 (USART3)
	GPIOD->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	GPIOD->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
	GPIOD->AFR[1] &= ~(GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1);
	GPIOD->AFR[1] |= (0x7UL << GPIO_AFRH_AFSEL8_Pos | 0x7UL << GPIO_AFRH_AFSEL9_Pos);

	// Baud: 115200, PCLK1=16MHz, USARTDIV=8.6875
	USART3->BRR = (0x8U << 4U) | 0xBU;
	USART3->CR1 &= ~USART_CR1_OVER8; // Oversampling by 16
	USART3->CR1 |= USART_CR1_UE;     // USART enable
	USART3->CR1 &= ~USART_CR1_M;     // 8 data bits
	USART3->CR1 &= ~USART_CR1_PCE;   // Parity disabled
	USART3->CR2 &= ~USART_CR2_STOP;  // 1 stop bit
	USART3->CR1 |= USART_CR1_TE;     // Transmitter enable
	USART3->CR1 |= USART_CR1_RE;     // Receiver enable

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
	uint32_t pclk2 = get_pclk2_hz();
	uint32_t tim9_clk = get_timxclk_from_pclk(pclk2);
	//Prescaler value
	TIM9->PSC = (tim9_clk / 1000000) - 1;	// get TIM counter clock at 1 MHz -> 1 µs period
	/* ARR */
	TIM9->ARR = 0xFFFF;
	// Input capture mode: CC1 as input, mapped to TI1
	TIM9->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM9->CCMR1 |= TIM_CCMR1_CC1S_0;

	/* Input Capture polarity config:
	 * 	// CC1P: Capture/Compare 1 output Polarity.
	 * 	// CC1NP: Capture/Compare 1 complementary output Polarity
	 * CC1NP:CC1P | Edge   | Note
	 * -----------|--------|--------------------------------------------------
	 *   0  :  0  | Rising | Standard
	 *   0  :  1  | Falling| Standard
	 *   1  :  1  | Both   | AVOID: Ambiguous timing/type, inconsistent HW support
	 *
	 * PRO TIP: For protocol decoding, manually toggle polarity in the ISR
	 * instead of using "Both Edges" to maintain state machine clarity. */

	// Set polarity to falling edge
	TIM9->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC1P);
	TIM9->CCER |= TIM_CCER_CC1P;

	// Clear flags and enable interrupt
	TIM9->SR = 0;
	TIM9->DIER |= TIM_DIER_CC1IE;

	// enable NVIC
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

	// Enable counter
	TIM9->CR1 |= TIM_CR1_CEN;
}

// TIM6 global interrupt: triggers DHT22 measurement and toggles LED
void TIM6_DAC_IRQHandler(void)
{
	dht22_req_flag = true;

// toggle integrated LED LD1 (green)
	GPIOB->ODR ^= GPIO_ODR_OD0;
	TIM6->SR &= ~(TIM_SR_UIF); // Clear interrupt flag
}

// TIM1 Break and TIM9 global interrupt: handles DHT22 protocol timing
void TIM1_BRK_TIM9_IRQHandler(void)
{
	dht22_tim9_isr();
}

// USART3 global interrupt (not used)
void USART3_IRQHandler(void)
{
	// No USART3 RX/TX interrupt logic implemented.
}

// Blocking transmit helper for UART
static void uart_send_str(const char *s)
{
	if (!s)
		return; // Null pointer check
	while (*s)
	{
		// Wait for TXE (polling, blocking)
		while (!(USART3->SR & USART_SR_TXE));
		USART3->DR = (uint8_t)(*s++);
	}
// Wait for TC (optional, ensures all data sent)
	while (!(USART3->SR & USART_SR_TC));
}

