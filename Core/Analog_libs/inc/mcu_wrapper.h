/*
 * mcu_wrapper.h
 *
 * Descrição: Abstração de hardware (SPI, GPIO, Timer) para o ADBMS6830B.
 *
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_MCU_WRAPPER_H_
#define ANALOG_LIBS_INC_MCU_WRAPPER_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "common.h"

/* Timeouts ---------------------------------------------------------------- */
#define SPI_TIME_OUT    HAL_MAX_DELAY
#define UART_TIME_OUT   HAL_MAX_DELAY

/* Handles de periféricos (definidos pelo CubeMX) -------------------------- */
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef  htim2;

/* -----------------------------------------------------------------------
 * Chip Select
 * ----------------------------------------------------------------------- */
void bmsCsLow(void);
void bmsCsHigh(void);

/* -----------------------------------------------------------------------
 * Timer de microssegundos (TIM2 configurado como contador de 1 MHz)
 * ----------------------------------------------------------------------- */
void     startTimer(void);
void     stopTimer(void);
uint32_t getTimCount(void);  /* retorna microsegundos */

/* -----------------------------------------------------------------------
 * Inicialização / Wake-up da cadeia daisy chain
 * ----------------------------------------------------------------------- */
void bmsWakeupIc(uint8_t total_ic);

/* -----------------------------------------------------------------------
 * Delay
 * ----------------------------------------------------------------------- */
void Delay_ms(uint32_t delay);
void delayUs(uint32_t us);
void delayMsActive(uint32_t ms);   /* delay mantendo IsoSPI ativo           */

/* -----------------------------------------------------------------------
 * Funções SPI de baixo nível
 * ----------------------------------------------------------------------- */
void spiWriteBytes(uint16_t size, uint8_t *tx_data);
void spiWriteReadBytes(uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
void spiReadBytes(uint16_t size, uint8_t *rx_data);

#endif /* ANALOG_LIBS_INC_MCU_WRAPPER_H_ */
