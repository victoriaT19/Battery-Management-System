/*
 * mcu_wrapper.c
 *
 * Descrição: Implementação da camada de abstração de hardware.
 *
 * CORREÇÕES em relação ao arquivo anterior:
 *   1. '#define WAKEUP_DELAY 1;' → '#define WAKEUP_DELAY 1'  (semicolon no macro
 *      causava expansão inválida: 'HAL_Delay(1;)').
 *   2. 'adBmsWakeupIc' renomeado para 'bmsWakeupIc' (consistência com header).
 *   3. Adicionadas funções delayUs() e delayMsActive() (usadas por bms_safety.c).
 *   4. Macro GPIO_PORT/CS_PIN alinhados com pinout do main.h do projeto.
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#include "../inc/common.h"
#include "../inc/mcu_wrapper.h"
#include "main.h"

/* -----------------------------------------------------------------------
 * Handles HAL (definidos pelo CubeMX em spi.c / tim.c)
 * ----------------------------------------------------------------------- */
static SPI_HandleTypeDef  *hspi  = &hspi1;
static TIM_HandleTypeDef  *htim  = &htim2;

/* -----------------------------------------------------------------------
 * Pino de Chip Select – alinhe com as definições do seu main.h
 * No projeto Manchester: BMS_CS_GPIO_Port / BMS_CS_Pin (PB6)
 * ----------------------------------------------------------------------- */
#define GPIO_PORT   BMS_CS_GPIO_Port
#define CS_PIN      BMS_CS_Pin

#define WAKEUP_DELAY  1   /* ms – sem ponto-e-vírgula! */

/* -----------------------------------------------------------------------
 * Chip Select
 * ----------------------------------------------------------------------- */
void bmsCsLow(void) {
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
}

void bmsCsHigh(void) {
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
}

/* -----------------------------------------------------------------------
 * Timer de microsegundos (TIM2 deve ser configurado a 1 MHz no CubeMX)
 * ----------------------------------------------------------------------- */
void startTimer(void) {
    HAL_TIM_Base_Start(htim);
}

void stopTimer(void) {
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SetCounter(htim, 0);
}

uint32_t getTimCount(void) {
    return (uint32_t)__HAL_TIM_GetCounter(htim);
}

/* -----------------------------------------------------------------------
 * Wake-up da cadeia daisy chain
 * Cada IC acorda ao detectar uma borda CS dentro de 4 ms de inatividade.
 * ----------------------------------------------------------------------- */
void bmsWakeupIc(uint8_t total_ic) {
    for (uint8_t ic = 0; ic < total_ic; ic++) {
        bmsCsLow();
        HAL_Delay(WAKEUP_DELAY);
        bmsCsHigh();
        HAL_Delay(WAKEUP_DELAY);
    }
}

/* -----------------------------------------------------------------------
 * Delays
 * ----------------------------------------------------------------------- */
void Delay_ms(uint32_t delay) {
    HAL_Delay(delay);
}

void delayUs(uint32_t us) {
    startTimer();
    while (getTimCount() < us);
    stopTimer();
}

/**
 * @brief Delay em ms mantendo o IsoSPI ativo (pulsa CS a cada 500 µs).
 *        Necessário para evitar que o IC entre em modo de dormência durante
 *        esperas longas (ex.: aguardar conversão AUX).
 */
void delayMsActive(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        bmsCsLow();
        delayUs(500);
        bmsCsHigh();
        delayUs(500);
    }
}

/* -----------------------------------------------------------------------
 * Funções SPI de baixo nível
 * ----------------------------------------------------------------------- */
void spiWriteBytes(uint16_t size, uint8_t *tx_data) {
    HAL_SPI_Transmit(hspi, tx_data, size, SPI_TIME_OUT);
}

void spiWriteReadBytes(uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    HAL_SPI_Transmit(hspi, tx_data, 4, SPI_TIME_OUT);
    HAL_SPI_Receive(hspi, rx_data, size, SPI_TIME_OUT);
}

void spiReadBytes(uint16_t size, uint8_t *rx_data) {
    HAL_SPI_Receive(hspi, rx_data, size, SPI_TIME_OUT);
}
