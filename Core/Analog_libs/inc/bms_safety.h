/*
 * bms_safety.h
 *
 * Descrição: Módulo de segurança (PROTÓTIPO) para o ADBMS6830B.
 *            Implementa verificações de:
 *              - Sobretensão / Subtensão (SW check + HW flags)
 *              - Sobretemperatura (NTC via AUX ADC)
 *              - Fio aberto / Open Wire (método PUP/PDOWN)
 *
 * STATUS: PROTÓTIPO – funções marcadas com [TODO] requerem implementação
 *         completa da leitura SPI de registradores de status.
 *
 * Conformidade FSAE 2026:
 *   - EV.5.7.1 : OV/UV detectados dentro de 500 ms (verificação periódica).
 *   - T.9.1    : AMS deve abrir AIRs em caso de falha.
 *   - EV.5.5   : Temperatura máxima de operação das células.
 *
 * Referência: ADBMS6830B Datasheet Rev.0, seções Status Registers, Open Wire.
 *
 * Criado em: Mar 2026
 * Autor: vtaua
 */

#ifndef ANALOG_LIBS_INC_BMS_SAFETY_H_
#define ANALOG_LIBS_INC_BMS_SAFETY_H_

#include "common.h"
#include "bms_data.h"

/* -----------------------------------------------------------------------
 * Parâmetros NTC (ajustar conforme BOM do acumulador)
 * Equação Beta: T = 1 / (ln(R/R0)/B + 1/T0) − 273.15
 * ----------------------------------------------------------------------- */
#define NTC_BETA          3950.0f   /* Constante Beta do termistor NTC      */
#define NTC_R0            10000.0f  /* Resistência nominal a T0 (Ω)         */
#define NTC_T0_KELVIN     298.15f   /* T0 = 25 °C em Kelvin                 */
#define NTC_PULLUP_R      10000.0f  /* Resistor de pull-up (Ω)              */
#define NTC_VREF          3.0f      /* Tensão de referência do divisor (V)  */

/* -----------------------------------------------------------------------
 * Bitmask de falhas
 * ----------------------------------------------------------------------- */
typedef enum {
    BMS_FAULT_NONE       = 0x00,
    BMS_FAULT_OV         = 0x01,   /* Sobretensão                          */
    BMS_FAULT_UV         = 0x02,   /* Subtensão                            */
    BMS_FAULT_OVERTEMP   = 0x04,   /* Sobretemperatura                     */
    BMS_FAULT_OPEN_WIRE  = 0x08,   /* Fio aberto                           */
    BMS_FAULT_COMMS      = 0x10,   /* Falha de comunicação                 */
} bms_fault_t;

/* -----------------------------------------------------------------------
 * Estado de falha global
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t fault_mask;                          /* Bitmask de falhas ativas */
    bool     ov_cell[TOTAL_IC][TOTAL_CELL];        /* Célula em OV             */
    bool     uv_cell[TOTAL_IC][TOTAL_CELL];        /* Célula em UV             */
    bool     ot_cell[TOTAL_IC][TOTAL_CELL];        /* Célula em OT             */
    bool     ow_cell[TOTAL_IC][TOTAL_CELL];        /* Open wire detectado      */
    float    temp_cell[TOTAL_IC][TOTAL_CELL];      /* Temperatura calculada    */
} bms_safety_state_t;

/* -----------------------------------------------------------------------
 * API pública
 * ----------------------------------------------------------------------- */

/**
 * @brief Verifica OV e UV por software comparando com thresholds.
 *        Atualiza state->fault_mask com BMS_FAULT_OV / BMS_FAULT_UV.
 * @param volt   Matriz de tensões [TOTAL_IC][TOTAL_CELL]
 * @param state  Estado de segurança a atualizar
 * @return BMS_OK se nenhuma falha, BMS_ERR_VOLTAGE caso contrário.
 */
BMS_StatusTypeDef bms_safety_check_voltage(float volt[TOTAL_IC][TOTAL_CELL],
                                           bms_safety_state_t *state);

/**
 * @brief Converte tensão AUX (NTC) para temperatura em °C.
 *        Utiliza equação Beta do NTC.
 * @param v_aux  Tensão medida no pino AUX (V)
 * @return Temperatura em °C
 */
float bms_safety_aux_to_celsius(float v_aux);

/**
 * @brief Verifica sobretemperatura.
 *        Popula state->temp_cell a partir de v_aux e verifica limites.
 * @param v_aux  Matriz de tensões AUX [TOTAL_IC][TOTAL_CELL]
 * @param state  Estado de segurança
 * @return BMS_OK ou BMS_ERR_TEMP
 */
BMS_StatusTypeDef bms_safety_check_temperature(float v_aux[TOTAL_IC][TOTAL_CELL],
                                               bms_safety_state_t *state);

/**
 * @brief Detecção de fio aberto via método PUP/PDOWN.
 *        [PROTÓTIPO] – sequência MUTE → ADCV(OW=PUP) → ADCV(OW=PDOWN) → compare.
 *        Células com |V_pup − V_pdown| > BMS_OW_DELTA_V marcadas como OW.
 *
 * @param volt_pup   Tensões medidas com pull-up   [TOTAL_IC][TOTAL_CELL]
 * @param volt_pdown Tensões medidas com pull-down [TOTAL_IC][TOTAL_CELL]
 * @param state      Estado de segurança
 * @return BMS_OK ou BMS_ERR_OPEN_WIRE
 *
 * @note TODO: Implementar disparo ADCV com OW=PUP e OW=PDOWN via spiSendCmd().
 */
BMS_StatusTypeDef bms_safety_check_open_wire(float volt_pup[TOTAL_IC][TOTAL_CELL],
                                             float volt_pdown[TOTAL_IC][TOTAL_CELL],
                                             bms_safety_state_t *state);

/**
 * @brief Lê flags de hardware do RDSTATC/RDSTATD.
 *        [PROTÓTIPO] – parse dos registradores de status do IC.
 *
 * @param state  Estado de segurança
 * @return BMS_OK ou BMS_ERR_VOLTAGE
 *
 * @note TODO: Implementar leitura SPI de RDSTATC e RDSTATD.
 */
BMS_StatusTypeDef bms_safety_check_hw_flags(bms_safety_state_t *state);

/**
 * @brief Task de segurança completa: tensão + temperatura + OW (se habilitado).
 * @param volt       Tensões de célula
 * @param v_aux      Tensões AUX (NTC)
 * @param check_ow   Se true, executa verificação de open wire
 * @param state      Estado de segurança
 * @return BMS_OK ou código de erro
 */
BMS_StatusTypeDef bms_safety_task(float volt[TOTAL_IC][TOTAL_CELL],
                                  float v_aux[TOTAL_IC][TOTAL_CELL],
                                  bool  check_ow,
                                  bms_safety_state_t *state);

/**
 * @brief Aciona desligamento de segurança (abre AIRs, para balanceamento).
 *        [TODO]: Adicionar controle GPIO das pré-carga / AIRs.
 */
void bms_safety_trigger_shutdown(bms_fault_t fault);

/**
 * @brief Limpa todos os flags de falha.
 */
void bms_safety_clear_faults(bms_safety_state_t *state);

#endif /* ANALOG_LIBS_INC_BMS_SAFETY_H_ */
