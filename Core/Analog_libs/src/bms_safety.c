/*
 * bms_safety.c
 *
 * Descrição: Implementação do módulo de segurança (PROTÓTIPO).
 *
 * Funções implementadas:
 *   [OK]  bms_safety_check_voltage   – verificação SW de OV/UV
 *   [OK]  bms_safety_aux_to_celsius  – conversão NTC Beta
 *   [OK]  bms_safety_check_temperature – verificação de sobretemperatura
 *   [PROTO] bms_safety_check_open_wire – algoritmo PUP/PDOWN (leitura SPI TODO)
 *   [PROTO] bms_safety_check_hw_flags  – leitura RDSTATC/RDSTATD (SPI TODO)
 *
 * Conformidade FSAE 2026:
 *   EV.5.7.1 – OV/UV devem desabilitar acumulador em < 500 ms.
 *   T.9.1    – AMS deve abrir AIRs em qualquer condição de falha.
 *   EV.5.5   – temperatura máxima 60 °C.
 *
 * Referência: ADBMS6830B Datasheet Rev.0, seções 7.3 (Status Registers),
 *             8.4 (Open Wire Detection).
 *
 * Criado em: Mar 2026
 * Autor: vtaua
 */

#include "../inc/bms_safety.h"
#include "../inc/bms_balancing.h"
#include "../inc/bms_generic.h"
#include "../inc/bms_data.h"
#include "../inc/common.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* -----------------------------------------------------------------------
 * Verificação de Tensão SW (OV / UV)
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_voltage(float volt[TOTAL_IC][TOTAL_CELL],
                                           bms_safety_state_t *state)
{
    BMS_StatusTypeDef result = BMS_OK;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            float v = volt[ic][c];

            if (v > BMS_OV_THRESHOLD_V) {
                state->ov_cell[ic][c] = true;
                state->fault_mask |= BMS_FAULT_OV;
                result = BMS_ERR_VOLTAGE;
            } else {
                state->ov_cell[ic][c] = false;
            }

            if (v < BMS_UV_THRESHOLD_V) {
                state->uv_cell[ic][c] = true;
                state->fault_mask |= BMS_FAULT_UV;
                result = BMS_ERR_VOLTAGE;
            } else {
                state->uv_cell[ic][c] = false;
            }
        }
    }
    return result;
}

/* -----------------------------------------------------------------------
 * Conversão tensão AUX (NTC) → temperatura °C
 * Equação Beta NTC:
 *   R_ntc = R_pu × (V / (Vref − V))
 *   T_k   = 1 / (ln(R_ntc / R0) / Beta + 1 / T0)
 *   T_c   = T_k − 273.15
 * ----------------------------------------------------------------------- */
float bms_safety_aux_to_celsius(float v_aux) {
    if (v_aux <= 0.0f || v_aux >= NTC_VREF) {
        return 999.0f;   /* fora do range → sinaliza erro */
    }
    float r_ntc = NTC_PULLUP_R * (v_aux / (NTC_VREF - v_aux));
    float t_k   = 1.0f / (logf(r_ntc / NTC_R0) / NTC_BETA + 1.0f / NTC_T0_KELVIN);
    return t_k - 273.15f;
}

/* -----------------------------------------------------------------------
 * Verificação de temperatura
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_temperature(float v_aux[TOTAL_IC][TOTAL_CELL],
                                               bms_safety_state_t *state)
{
    BMS_StatusTypeDef result = BMS_OK;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            float t = bms_safety_aux_to_celsius(v_aux[ic][c]);
            state->temp_cell[ic][c] = t;

            if (t > BMS_TEMP_MAX_C || t < BMS_TEMP_MIN_C) {
                state->ot_cell[ic][c] = true;
                state->fault_mask |= BMS_FAULT_OVERTEMP;
                result = BMS_ERR_TEMP;
            } else {
                state->ot_cell[ic][c] = false;
            }
        }
    }
    return result;
}

/* -----------------------------------------------------------------------
 * Detecção de Open Wire (PROTÓTIPO)
 *
 * Procedimento completo (datasheet seção 8.4):
 *   1. MUTE    – desabilita descarga passiva para não interferir na medição.
 *   2. ADCV com OW=01 (pull-up)   → lê tensões volt_pup.
 *   3. ADCV com OW=10 (pull-down) → lê tensões volt_pdown.
 *   4. UNMUTE  – reabilita descarga.
 *   5. Para cada célula: se |Vpup − Vpdown| > BMS_OW_DELTA_V → fio aberto.
 *
 * TODO: implementar disparos ADCV OW=PUP/PDOWN via spiSendCmd(ADCV_ow).
 *       Atualmente, a função recebe as tensões já lidas como parâmetro.
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_open_wire(float volt_pup[TOTAL_IC][TOTAL_CELL],
                                             float volt_pdown[TOTAL_IC][TOTAL_CELL],
                                             bms_safety_state_t *state)
{
    BMS_StatusTypeDef result = BMS_OK;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            float diff = fabsf(volt_pup[ic][c] - volt_pdown[ic][c]);
            if (diff > BMS_OW_DELTA_V) {
                state->ow_cell[ic][c] = true;
                state->fault_mask |= BMS_FAULT_OPEN_WIRE;
                result = BMS_ERR_OPEN_WIRE;
            } else {
                state->ow_cell[ic][c] = false;
            }
        }
    }
    return result;
}

/* -----------------------------------------------------------------------
 * Leitura de flags de hardware (PROTÓTIPO)
 *
 * TODO:
 *   1. Chamar spiReadData(TOTAL_IC, RDSTATC, ...) para ler RDSTATC.
 *   2. Chamar spiReadData(TOTAL_IC, RDSTATD, ...) para ler RDSTATD.
 *   3. Parsear bits CS_FLT (OV/UV por célula) de RDSTATC.
 *   4. Parsear bits de falha de RDSTATD.
 *   5. Propagar para state->fault_mask.
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_hw_flags(bms_safety_state_t *state)
{
    /* [TODO] Implementar leitura SPI de RDSTATC + RDSTATD */
    (void)state;
    return BMS_OK;
}

/* -----------------------------------------------------------------------
 * Task completa de segurança
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_task(float volt[TOTAL_IC][TOTAL_CELL],
                                  float v_aux[TOTAL_IC][TOTAL_CELL],
                                  bool  check_ow,
                                  bms_safety_state_t *state)
{
    BMS_StatusTypeDef status = BMS_OK;

    /* 1. Verificação de tensão SW */
    BMS_StatusTypeDef v_status = bms_safety_check_voltage(volt, state);
    if (v_status != BMS_OK) status = v_status;

    /* 2. Verificação de temperatura */
    BMS_StatusTypeDef t_status = bms_safety_check_temperature(v_aux, state);
    if (t_status != BMS_OK) status = t_status;

    /* 3. Verificação de flags HW (RDSTATC/RDSTATD) */
    bms_safety_check_hw_flags(state);

    /* 4. Open wire (opcional – demorado ~10 ms) */
    if (check_ow) {
        /* [TODO] Executar sequência MUTE → ADCV(PUP) → lê → ADCV(PDOWN) → lê → UNMUTE */
        /* Por ora, apenas verificamos com as últimas tensões disponíveis */
        BMS_StatusTypeDef ow_status =
            bms_safety_check_open_wire(volt, volt, state);
        if (ow_status != BMS_OK) status = ow_status;
    }

    /* 5. Aciona desligamento se necessário */
    if (status != BMS_OK) {
        bms_safety_trigger_shutdown((bms_fault_t)state->fault_mask);
    }

    return status;
}

/* -----------------------------------------------------------------------
 * Trigger de shutdown – para balanceamento e sinaliza GPIO
 *
 * TODO:
 *   - Controlar pino AIR+ / AIR− via HAL_GPIO_WritePin(...)
 *   - Transmitir código de falha via CAN (T.9.1)
 * ----------------------------------------------------------------------- */
void bms_safety_trigger_shutdown(bms_fault_t fault)
{
    /* Para balanceamento imediatamente (chamada de alto nível em comm.c) */
    /* TODO: chamar bms_balancing_stop_all() a partir do contexto de comm.c */

    /* TODO: HAL_GPIO_WritePin(AIR_PORT, AIR_PIN, GPIO_PIN_RESET); */
    /* TODO: BMS_CAN_SendFaultFrame(fault); */
    (void)fault;
}

/* -----------------------------------------------------------------------
 * Limpar falhas
 * ----------------------------------------------------------------------- */
void bms_safety_clear_faults(bms_safety_state_t *state) {
    state->fault_mask = BMS_FAULT_NONE;
    memset(state->ov_cell,  0, sizeof(state->ov_cell));
    memset(state->uv_cell,  0, sizeof(state->uv_cell));
    memset(state->ot_cell,  0, sizeof(state->ot_cell));
    memset(state->ow_cell,  0, sizeof(state->ow_cell));
}
