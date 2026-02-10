/*******************************************************************************
 * @file        clock.c
 * @brief       STM32F446 clock frequency helper functions
 * @author      Dominik
 *
 * @description
 * Provides runtime functions for determining the current system and bus
 * clock frequencies based on RCC register configuration.
 *
 * This module does NOT configure or modify the clock tree. It only reads
 * RCC registers and computes effective frequencies for:
 *
 *   - SYSCLK  (core clock)
 *   - HCLK    (AHB bus)
 *   - PCLK1   (APB1 bus)
 *   - PCLK2   (APB2 bus)
 *   - TIMxCLK (timer input clocks)
 *
 * These helpers are especially useful when configuring peripherals that
 * depend on precise timing, such as:
 *   - Timers (PSC/ARR calculations)
 *   - UART baud rate generation
 *   - Communication peripherals
 *
 * STM32 timer note:
 *   When an APB prescaler is greater than 1, timer clocks run at 2× PCLK.
 *   This module automatically accounts for this behavior.
 *
 ******************************************************************************/

#include "clock.h"
#include "stm32f446xx.h"
#include <stdint.h>

uint32_t ahb_presc_table(uint32_t hpre_bits) {
    // HPRE encoding: 0xxx => /1, 1000=>/2, 1001=>/4, ... 1111=>/512
    static const uint16_t div[16] = {
        1,1,1,1,1,1,1,1, 2,4,8,16,64,128,256,512
    };
    return div[hpre_bits & 0xF];
}

uint32_t apb_presc_table(uint32_t ppre_bits) {
    // PPRE encoding: 0xx => /1, 100=>/2, 101=>/4, 110=>/8, 111=>/16
    static const uint8_t div[8] = {1,1,1,1, 2,4,8,16};
    return div[ppre_bits & 0x7];
}

uint32_t get_sysclk_hz(void) {
    uint32_t sws = (RCC->CFGR >> RCC_CFGR_SWS_Pos) & 0x3;

    if (sws == 0x0) { // HSI
        return HSI_VALUE;
    } else if (sws == 0x1) { // HSE
        return HSE_VALUE;
    } else { // PLL
        // Decode PLL
        uint32_t pllsrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? HSE_VALUE : HSI_VALUE;
        uint32_t pllm   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
        uint32_t plln   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
        uint32_t pllp_bits = (RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos;
        uint32_t pllp = (pllp_bits + 1U) * 2U; // 00->2, 01->4, 10->6, 11->8

        uint32_t vco_in  = pllsrc / pllm;
        uint32_t vco_out = vco_in * plln;
        return vco_out / pllp;
    }
}

uint32_t get_hclk_hz(void) {
    uint32_t sys = get_sysclk_hz();
    uint32_t hpre = (RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0xF;
    return sys / ahb_presc_table(hpre);
}

uint32_t get_pclk1_hz(void) {
    uint32_t hclk = get_hclk_hz();
    uint32_t ppre1 = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7;
    return hclk / apb_presc_table(ppre1);
}

uint32_t get_pclk2_hz(void) {
    uint32_t hclk = get_hclk_hz();
    uint32_t ppre2 = (RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x7;
    return hclk / apb_presc_table(ppre2);
}

uint32_t get_timxclk_from_pclk(uint32_t pclk) {
	uint32_t ppre_bits = (RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x7;
	uint32_t presc = apb_presc_table(ppre_bits);
    return (presc == 1) ? pclk : (pclk * 2U);
}

