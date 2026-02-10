/*******************************************************************************
 * @file        clock.h
 * @brief       Public clock utility interface for STM32F446
 * @author      Dominik
 *
 * @description
 * Provides functions to query effective clock frequencies of the STM32
 * clock tree. These helpers simplify peripheral configuration by removing
 * hardcoded assumptions about bus or timer frequencies.
 *
 * The module only reads RCC configuration and performs calculations.
 * It does NOT configure or change any clocks.
 *
 * Typical usage:
 *
 *   uint32_t pclk1 = get_pclk1_hz();
 *   TIM6->PSC = (pclk1 / 1000) - 1;
 *
 * This ensures correct behavior even if prescalers or PLL settings change.
 ******************************************************************************/

#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdint.h>

#define HSI_VALUE 16000000UL	// 16MHz
#define HSE_VALUE 8000000UL   	// 8Mhz

/**
 * @brief  Returns the AHB prescaler division factor.
 *
 * Decodes the HPRE bitfield from RCC->CFGR and converts it to the
 * corresponding division factor applied to SYSCLK to generate HCLK.
 *
 * @param  hpre_bits  Raw HPRE prescaler bits (RCC_CFGR[7:4]).
 *
 * @return Division factor (1, 2, 4, 8, ..., 512).
 */
uint32_t ahb_presc_table(uint32_t hpre_bits);


/**
 * @brief  Returns the APB prescaler division factor.
 *
 * Decodes the PPRE bitfield from RCC->CFGR and converts it to the
 * corresponding division factor applied to HCLK to generate PCLK.
 *
 * @param  ppre_bits  Raw PPRE prescaler bits (RCC_CFGR).
 *
 * @return Division factor (1, 2, 4, 8, or 16).
 */
uint32_t apb_presc_table(uint32_t ppre_bits);


/**
 * @brief  Returns the current system clock frequency (SYSCLK).
 *
 * Determines which clock source is active (HSI, HSE, or PLL) and computes
 * the resulting core frequency by decoding RCC configuration registers.
 *
 * @return SYSCLK frequency in Hertz (Hz).
 */
uint32_t get_sysclk_hz(void);


/**
 * @brief  Returns the AHB bus clock frequency (HCLK).
 *
 * HCLK is derived from SYSCLK after applying the AHB prescaler (HPRE).
 * This clock feeds the CPU, memory, and AHB peripherals.
 *
 * @return HCLK frequency in Hertz (Hz).
 */
uint32_t get_hclk_hz(void);


/**
 * @brief  Returns the APB1 peripheral clock frequency (PCLK1).
 *
 * PCLK1 is derived from HCLK after applying the APB1 prescaler (PPRE1).
 * Used by peripherals such as TIM2–TIM7, USART2/3, I2C, etc.
 *
 * @return APB1 clock frequency in Hertz (Hz).
 */
uint32_t get_pclk1_hz(void);


/**
 * @brief  Returns the APB2 peripheral clock frequency (PCLK2).
 *
 * PCLK2 is derived from HCLK after applying the APB2 prescaler (PPRE2).
 * Used by peripherals such as TIM1/8/9/10/11, USART1/6, ADC, etc.
 *
 * @return APB2 clock frequency in Hertz (Hz).
 */
uint32_t get_pclk2_hz(void);


/**
 * @brief  Returns the effective timer clock frequency for a given APB clock.
 *
 * On STM32 devices, when the APB prescaler is greater than 1, the timer
 * clock automatically runs at 2× the corresponding PCLK. This helper
 * accounts for that hardware behavior and returns the actual timer input
 * frequency.
 *
 * Typical usage:
 *   uint32_t timclk = get_timxclk_from_pclk(get_pclk1_hz());
 *
 * @param  pclk  Corresponding APB clock frequency (Hz).
 *
 * @return Timer clock frequency in Hertz (Hz).
 */
uint32_t get_timxclk_from_pclk(uint32_t pclk);


#endif /* CLOCK_H_ */
