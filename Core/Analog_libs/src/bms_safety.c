/*
 * bms_safety.c
 *
 * Descrição: Implementação do módulo de segurança.
 *
 * Funções implementadas:
 *   [OK] bms_safety_check_voltage        – verificação SW de OV/UV
 *   [OK] bms_safety_check_die_temperature – sobretemperatura do die do IC
 *   [OK] bms_safety_check_open_wire      – algoritmo canais pares/ímpares
 *   [OK] bms_safety_check_hw_flags       – leitura RDSTATC/RDSTATD
 *   [OK] bms_safety_trigger_shutdown     – abre shutdown circuit via GPIO
 *        (placeholder de pino -- ver aviso abaixo)
 *
 * Conformidade FSAE 2026: ver bms_safety.h para citações corretas
 * (EV.7.3.4 / EV.7.3.5). EV.5.7 e T.9.1 NÃO se aplicam aqui -- ver nota
 * em bms_safety.h.
 *
 * Referência: ADBMS6830B Datasheet Rev.0, seções 7.3 (Status Registers),
 *             8.4 (Open Wire Detection), Thermal Shutdown.
 *
 * Criado em: Mar 2026
 * Modificado em: jun 2026
 * Autor: vtaua
 */

#include "../inc/bms_safety.h"
#include "../inc/bms_balancing.h"
#include "../inc/bms_generic.h"
#include "../inc/bms_data.h"
#include "../inc/common.h"
#include "../inc/communication.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* -----------------------------------------------------------------------
 * @brief Verificação de Tensão SW (OV / UV)
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
 * @brief Verificação de sobretemperatura do DIE (auto-diagnóstico do IC).
 * Temperatura de célula é responsabilidade do TMS dedicado, fora deste
 * módulo.
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_die_temperature(float temp_die[TOTAL_IC],
                                                   bms_safety_state_t *state)
{
    BMS_StatusTypeDef result = BMS_OK;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        float t = temp_die[ic];
        state->temp_die[ic] = t;

        if (t > BMS_DIE_TEMP_MAX_C || t < BMS_DIE_TEMP_MIN_C) {
            state->die_ot[ic] = true;
            state->fault_mask |= BMS_FAULT_DIE_OVERTEMP;
            result = BMS_ERR_TEMP;
        } else {
            state->die_ot[ic] = false;
        }
    }
    return result;
}

/* -----------------------------------------------------------------------
 * @brief Detecção de Open Wire
 *
 * Procedimento (datasheet seção "Cell Open Wire Detection"):
 *   1. MUTE    – desabilita descarga passiva para não interferir na medição.
 *   2. ADCV com OW[1:0]=01 (canais pares)   → lê tensões volt_even.
 *   3. ADCV com OW[1:0]=10 (canais ímpares) → lê tensões volt_odd.
 *   4. UNMUTE  – reabilita descarga.
 *   5. Para cada célula: se |Vpares − Vímpares| > BMS_OW_DELTA_V → fio aberto.
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_open_wire(float volt_even[TOTAL_IC][TOTAL_CELL],
                                             float volt_odd[TOTAL_IC][TOTAL_CELL],
                                             bms_safety_state_t *state)
{
    BMS_StatusTypeDef result = BMS_OK;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            float diff = fabsf(volt_even[ic][c] - volt_odd[ic][c]);
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
 * @brief Leitura de flags de hardware
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_check_hw_flags(bms_safety_state_t *state)
{
    uint8_t rx_data[TOTAL_IC * RX_DATA];
    uint8_t pec_err[TOTAL_IC];
    uint8_t cmd_cntr[TOTAL_IC];

    spiReadData(TOTAL_IC, RDSTATC, rx_data, pec_err, cmd_cntr, RX_DATA);

    for(int ic = 0; ic < TOTAL_IC; ic++){
        uint16_t cs_flt = (rx_data[ic * RX_DATA + 1] << 8 | rx_data[ic * RX_DATA]);
        if(cs_flt != 0){
            /* CSxFLT: mismatch C-ADC vs S-ADC por canal (datasheet Table 108).
             * Mapeado no bucket de tensão pois indica problema de medição
             * de tensão de célula. */
            state->fault_mask |= BMS_FAULT_OV;
        }
    }

    spiReadData(TOTAL_IC, RDSTATD, rx_data, pec_err, cmd_cntr, RX_DATA);

    for(int ic = 0; ic < TOTAL_IC; ic++){
        if(rx_data[ic * RX_DATA] != 0 || rx_data[ic * RX_DATA + 1] != 0 ||
           rx_data[ic * RX_DATA + 2] != 0 || rx_data[ic * RX_DATA + 3] != 0){
            /* STDR0-3: flags individuais de OV/UV por célula (datasheet
             * Table 109). */
            state->fault_mask |= BMS_FAULT_OV;
        }
    }

    return BMS_OK;
}

/* -----------------------------------------------------------------------
 * @brief Task completa de segurança
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_safety_task(float volt[TOTAL_IC][TOTAL_CELL],
                                  float temp_die[TOTAL_IC],
                                  bool  check_ow,
                                  bms_safety_state_t *state)
{
    BMS_StatusTypeDef status = BMS_OK;

    /* 1. Verificação de tensão SW */
    BMS_StatusTypeDef v_status = bms_safety_check_voltage(volt, state);
    if (v_status != BMS_OK) status = v_status;

    /* 2. Verificação de temperatura do die */
    BMS_StatusTypeDef t_status = bms_safety_check_die_temperature(temp_die, state);
    if (t_status != BMS_OK) status = t_status;

    /* 3. Verificação de flags HW (RDSTATC/RDSTATD) */
    bms_safety_check_hw_flags(state);

    /* 4. Open wire (opcional – demorado ~10-20 ms) */
    if (check_ow) {
        spiSendCmd(MUTE);

        /* Leitura com canais pares habilitados (OW[1:0]=01) */
        ADCV.ow = 1;
        ADCV.cont = 0;
        spiSendCmd((uint8_t *)&ADCV);
        if (bmsPollAdc(PLCADC) == UINT32_MAX) { spiSendCmd(UNMUTE); return status; }
        readCellVoltage(VOLTAGE_C);

        float volt_ow_even[TOTAL_IC][TOTAL_CELL];
        memcpy(volt_ow_even, volt, sizeof(volt_ow_even));

        /* Leitura com canais ímpares habilitados (OW[1:0]=10) */
        ADCV.ow = 2;
        spiSendCmd((uint8_t *)&ADCV);
        if (bmsPollAdc(PLCADC) == UINT32_MAX) { spiSendCmd(UNMUTE); return status; }
        readCellVoltage(VOLTAGE_C);

        float volt_ow_odd[TOTAL_IC][TOTAL_CELL];
        memcpy(volt_ow_odd, volt, sizeof(volt_ow_odd));

        spiSendCmd(UNMUTE);

        BMS_StatusTypeDef ow_status =
            bms_safety_check_open_wire(volt_ow_even, volt_ow_odd, state);
        if (ow_status != BMS_OK) status = ow_status;

        ADCV.ow = 0;
        ADCV.cont = 1;
        spiSendCmd((uint8_t*)&ADCV);
    }

    /* 5. Aciona desligamento se necessário */
    if (status != BMS_OK) {
        bms_safety_trigger_shutdown((bms_fault_t)state->fault_mask);
    }

    return status;
}

/* -----------------------------------------------------------------------
 * @brief Trigger de shutdown – sinaliza GPIO do shutdown circuit.
 *
 * !!! ATENÇÃO – PINO PLACEHOLDER !!!
 * Usa BMS_CS2_Pin/BMS_CS2_GPIO_Port (PC7) como placeholder temporário
 * porque o pino real de controle do AIR/contator ainda não foi definido
 * no hardware. TROCAR pelo pino real assim que for reservado no main.h
 * e substituir os #defines abaixo.
 *
 * Lógica: nível ATIVO BAIXO assumido (GPIO_PIN_RESET = abre o shutdown
 * circuit). Confirme a polaridade real do driver/optoacoplador do AIR
 * antes de ligar em hardware -- se estiver invertido, a célula vai
 * permanecer fechada justamente quando deveria abrir.
 * ----------------------------------------------------------------------- */
#define AIR_CTRL_PLACEHOLDER_PORT  BMS_CS2_GPIO_Port   /* TODO: trocar pelo pino real */
#define AIR_CTRL_PLACEHOLDER_PIN   BMS_CS2_Pin          /* TODO: trocar pelo pino real */

void bms_safety_trigger_shutdown(bms_fault_t fault)
{
    /* Abre o shutdown circuit (placeholder de hardware, ver aviso acima) */
    HAL_GPIO_WritePin(AIR_CTRL_PLACEHOLDER_PORT, AIR_CTRL_PLACEHOLDER_PIN,
                       GPIO_PIN_RESET);

    /* TODO: Transmitir código de falha via CAN para telemetria/dashboard.
     *       BMS_GetCanData() já existe em comm.c; criar um frame dedicado
     *       de falha (não apenas dados de célula) e chamar aqui. */
    (void)fault;
}

/* -----------------------------------------------------------------------
 * @brief Limpar falhas
 * ----------------------------------------------------------------------- */
void bms_safety_clear_faults(bms_safety_state_t *state) {
    state->fault_mask = BMS_FAULT_NONE;
    memset(state->ov_cell,  0, sizeof(state->ov_cell));
    memset(state->uv_cell,  0, sizeof(state->uv_cell));
    memset(state->die_ot,   0, sizeof(state->die_ot));
    memset(state->ow_cell,  0, sizeof(state->ow_cell));
}
