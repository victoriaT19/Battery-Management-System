/*
 * comm.c
 *
 * Descrição: Camada de aplicação BMS – análogo ao bms_libWrapper.c do
 *            repositório ManchesterStingerMotorsports/g474-bms.
 *
 * IMPLEMENTAÇÕES NOVAS em relação ao arquivo anterior (que continha apenas
 * a struct adbms68 vazia):
 *   - Instância global adbms68_t (struct unificada de todos os dados BMS).
 *   - bms_init()          : configura CFGA/CFGB (VOV, VUV, DCTO, REFON).
 *   - readCellVoltage()   : lê RDCVA-F e converte para float.
 *   - auxMeasurement()    : lê RDAUXA/B e converte NTC → °C.
 *   - startDischarge()    : ativa balanceamento proporcional.
 *   - stopDischarge()     : para balanceamento.
 *   - programLoop()       : sequência principal (volt → temp → OW → balanceamento).
 *   - checkVoltage/Temp() : retornam status de falhas.
 *   - BMS_WriteFaultSignal: controla GPIO de falha.
 *
 * Referência: bms_libWrapper.c do repo Manchester Stinger (Nov 2024 – amrlxyz)
 *
 * Modificado em: Mar 2026
 * Autor: vtaua
 */

#include "../inc/common.h"
#include "../inc/bms_cmd.h"
#include "../inc/bms_data.h"
#include "../inc/bms_generic.h"
#include "../inc/communication.h"
#include "../inc/bms_balancing.h"
#include "../inc/bms_safety.h"
#include "../inc/mcu_wrapper.h"
#include "main.h"
#include <string.h>

/* -----------------------------------------------------------------------
 * Struct principal de dados do BMS
 * ----------------------------------------------------------------------- */
typedef struct {
    /* Registradores de configuração (Tx = o que enviamos, Rx = o que lemos) */
    ad68_cfa_t  cfa_Tx[TOTAL_IC];
    ad68_cfa_t  cfa_Rx[TOTAL_IC];
    ad68_cfb_t  cfb_Tx[TOTAL_IC];
    ad68_cfb_t  cfb_Rx[TOTAL_IC];

    /* Registradores PWM */
    ad68_pwma_t pwma[TOTAL_IC];
    ad68_pwmb_t pwmb[TOTAL_IC];

    /* Tensões de célula [tipo][IC][célula] */
    float volt_cell     [TOTAL_VOLTAGE_TYPES][TOTAL_IC][TOTAL_CELL];
    float volt_cell_dif [TOTAL_VOLTAGE_TYPES][TOTAL_IC][TOTAL_CELL];
    float volt_cell_sum [TOTAL_VOLTAGE_TYPES][TOTAL_IC];
    float volt_cell_avg [TOTAL_VOLTAGE_TYPES][TOTAL_IC];
    float volt_cell_min [TOTAL_VOLTAGE_TYPES][TOTAL_IC];
    float volt_cell_max [TOTAL_VOLTAGE_TYPES][TOTAL_IC];
    float volt_cell_delta[TOTAL_VOLTAGE_TYPES][TOTAL_IC];

    /* Temperatura */
    float volt_seg  [TOTAL_IC];
    float temp_cell [TOTAL_IC][TOTAL_CELL];   /* °C */
    float temp_ic   [TOTAL_IC];               /* Die temperature °C */
    float v_aux     [TOTAL_IC][TOTAL_CELL];   /* Tensão bruta AUX (NTC) */

    /* Status de descarga */
    uint16_t isDischarging      [TOTAL_IC];
    uint16_t isCellFaultDetect  [TOTAL_IC];

} adbms68_t;

/* Instância global */
static adbms68_t bms;

/* -----------------------------------------------------------------------
 * Buffers SPI temporários (reusados a cada operação)
 * ----------------------------------------------------------------------- */
static uint8_t  txData[TOTAL_IC][TX_DATA];
static uint8_t  rxData[TOTAL_IC][RX_DATA];
static uint8_t  rxPecErr[TOTAL_IC];
static uint8_t  rxCmdCnt[TOTAL_IC];

/* -----------------------------------------------------------------------
 * Estado de falha e controle
 * ----------------------------------------------------------------------- */
static BMS_StatusTypeDef  g_statusFlags   = BMS_ERR_COMMS;
static bool               g_balancingEnabled  = false;
static bms_bal_state_t    g_balState;
static bms_safety_state_t g_safetyState;

/* Tipo de tensão usado para balanceamento (S-ADC pois para PWM brevemente) */
static const VoltageTypes DISCHARGE_VOLT_TYPE = VOLTAGE_S;

/* Limiar de balanceamento configurável */
static float g_balDelta = BAL_DELTA_DEFAULT_V;

/* -----------------------------------------------------------------------
 * Helpers internos
 * ----------------------------------------------------------------------- */

/** Escreve registrador de configuração ou PWM para toda a cadeia */
static void writeRegister(RegisterTypes regType) {
    uint8_t *cmd;

    switch (regType) {
        case REG_CONFIG_A:
            for (int ic = 0; ic < TOTAL_IC; ic++)
                memcpy(txData[ic], &bms.cfa_Tx[ic], TX_DATA);
            cmd = WRCFGA;
            break;
        case REG_CONFIG_B:
            for (int ic = 0; ic < TOTAL_IC; ic++)
                memcpy(txData[ic], &bms.cfb_Tx[ic], TX_DATA);
            cmd = WRCFGB;
            break;
        case REG_PWM_A:
            for (int ic = 0; ic < TOTAL_IC; ic++)
                memcpy(txData[ic], &bms.pwma[ic], TX_DATA);
            cmd = WRPWMA;
            break;
        case REG_PWM_B:
            for (int ic = 0; ic < TOTAL_IC; ic++)
                memcpy(txData[ic], &bms.pwmb[ic], TX_DATA);
            cmd = WRPWMB;
            break;
        default:
            return;
    }
    spiWriteData(TOTAL_IC, cmd, (uint8_t *)txData);
}

/** Verifica PEC e retorna true se algum IC tem erro */
static bool checkPecFault(void) {
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        if (rxPecErr[ic]) return true;
    }
    return false;
}

/** Calcula estatísticas min/max/avg/delta por IC e globais */
static void calculateStats(VoltageTypes vtype) {
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        float sum =  0.0f;
        float mn  =  9999.0f;
        float mx  = -9999.0f;

        for (int c = 0; c < TOTAL_CELL; c++) {
            float v = bms.volt_cell[vtype][ic][c];
            sum += v;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }

        bms.volt_cell_sum  [vtype][ic] = sum;
        bms.volt_cell_avg  [vtype][ic] = sum / (float)TOTAL_CELL;
        bms.volt_cell_min  [vtype][ic] = mn;
        bms.volt_cell_max  [vtype][ic] = mx;
        bms.volt_cell_delta[vtype][ic] = mx - mn;

        for (int c = 0; c < TOTAL_CELL; c++) {
            bms.volt_cell_dif[vtype][ic][c] = bms.volt_cell[vtype][ic][c] - mn;
        }
    }
}

/* -----------------------------------------------------------------------
 * Valor padrão dos registradores após reset
 * Obtido de leitura RDCFG após SRST (conforme Manchester repo bms_resetConfig)
 * ----------------------------------------------------------------------- */
static void resetConfig(void) {
    /* CFGA padrão: REFON=0, fc=0 (será configurado no init) */
    const uint8_t cfgaDefault[TX_DATA] = { 0x00, 0x03, 0xFF, 0x00, 0x00, 0x01 };
    /* CFGB padrão (datasheet): VUV/VOV zerados, DCTO=0 */
    const uint8_t cfgbDefault[TX_DATA] = { 0x00, 0x00, 0x7F, 0xF8, 0x00, 0x00 };

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        memcpy(&bms.cfa_Tx[ic], cfgaDefault, TX_DATA);
        memcpy(&bms.cfb_Tx[ic], cfgbDefault, TX_DATA);
        memset(&bms.pwma[ic],   0, sizeof(ad68_pwma_t));
        memset(&bms.pwmb[ic],   0, sizeof(ad68_pwmb_t));
    }
}

/* -----------------------------------------------------------------------
 * Inicialização
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef bms_init(void) {
    memset(&bms,         0, sizeof(bms));
    memset(&g_balState,  0, sizeof(g_balState));
    memset(&g_safetyState, 0, sizeof(g_safetyState));

    resetConfig();

    /* Configurações CFGA: REFON=1, fc=001 (110 Hz corner) */
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        bms.cfa_Tx[ic].refon = 1;
        bms.cfa_Tx[ic].fc    = 0x01;
    }

    /* Configurações CFGB: VOV, VUV, DCTO */
    uint16_t vov_code = bms_encode_vov(BMS_OV_THRESHOLD_V);
    uint16_t vuv_code = bms_encode_vuv(BMS_UV_THRESHOLD_V);
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        bms.cfb_Tx[ic].vuv0_7  = (uint8_t)(vuv_code & 0xFF);
        bms.cfb_Tx[ic].vuv8_11 = (uint8_t)((vuv_code >> 8) & 0x0F);
        bms.cfb_Tx[ic].vov0_3  = (uint8_t)(vov_code & 0x0F);
        bms.cfb_Tx[ic].vov4_11 = (uint8_t)((vov_code >> 4) & 0xFF);
        bms.cfb_Tx[ic].dcto    = 0;
        bms.cfb_Tx[ic].dtmen   = 0;
    }

    /* Acorda cadeia e escreve configuração */
    bmsWakeupIc(TOTAL_IC);

    /* Verifica comunicação lendo SID */
    BMS_StatusTypeDef status = readRegister(REG_SID);
    if (status != BMS_OK) return BMS_ERR_COMMS;

    writeRegister(REG_CONFIG_A);
    writeRegister(REG_CONFIG_B);

    /* Inicia C-ADC contínuo */
    bms_startAdcvCont(false);
    delayMsActive(12);   /* aguarda filtro encher (~8 ms para avg register) */

    g_statusFlags = BMS_OK;
    return BMS_OK;
}

/* -----------------------------------------------------------------------
 * Configuração de GPO4/5 (mux de temperatura, igual ao Manchester repo)
 * ----------------------------------------------------------------------- */
void bms_setGpo45(uint8_t twoBitIndex) {
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        /* GPO pull-down: 1=sem pull-down (default), 0=pull-down */
        bms.cfa_Tx[ic].gpo1_8 = ((twoBitIndex) << 3) | (0xFF ^ (0x03 << 3));
    }
    writeRegister(REG_CONFIG_A);
}

/* -----------------------------------------------------------------------
 * Inicia C-ADC contínuo
 * ----------------------------------------------------------------------- */
void bms_startAdcvCont(bool enableRedundant) {
    ADCV.cont = 1;
    ADCV.dcp  = 0;
    ADCV.rstf = 1;
    ADCV.ow   = 0x00;
    ADCV.rd   = enableRedundant ? 1 : 0;
    spiSendCmd((uint8_t *)&ADCV);
}

/* -----------------------------------------------------------------------
 * Leitura de tensão de célula (parse 3 células por grupo, 6 grupos)
 * ----------------------------------------------------------------------- */
static void parseVoltage(uint8_t raw[TOTAL_IC][RX_DATA],
                         float   vArr[TOTAL_IC][TOTAL_CELL],
                         uint8_t reg_idx)
{
    uint8_t cell_base = reg_idx * 3;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int j = 0; j < 3; j++) {
            int c = cell_base + j;
            if (c >= TOTAL_CELL) break;
            int16_t raw16 = (int16_t)((raw[ic][j*2 + 1] << 8) | raw[ic][j*2]);
            vArr[ic][c] = CELL_VOLTAGE_V(raw16);
        }
    }
}

/* Tabela de comandos de leitura por tipo de tensão e grupo */
static uint8_t *rdCmdTable[TOTAL_VOLTAGE_TYPES][6] = {
    { (uint8_t*)NULL }, /* VOLTAGE_C     – inicializado em bms_init (abaixo) */
    { (uint8_t*)NULL }, /* VOLTAGE_C_AVG */
    { (uint8_t*)NULL }, /* VOLTAGE_C_FIL */
    { (uint8_t*)NULL }, /* VOLTAGE_S     */
    { (uint8_t*)NULL }, /* VOLTAGE_TEMP  */
};

static void initCmdTable(void) {
    uint8_t *c[6]   = { RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF };
    uint8_t *ca[6]  = { RDACA, RDACB, RDACC, RDACD, RDACE, RDACF };
    uint8_t *cf[6]  = { RDFCA, RDFCB, RDFCC, RDFCD, RDFCE, RDFCF };
    uint8_t *s[6]   = { RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF };
    uint8_t *ax[6]  = { RDAUXA, RDAUXB, RDAUXA, RDAUXB, RDAUXA, RDAUXB };

    for (int i = 0; i < 6; i++) {
        rdCmdTable[VOLTAGE_C]    [i] = c[i];
        rdCmdTable[VOLTAGE_C_AVG][i] = ca[i];
        rdCmdTable[VOLTAGE_C_FIL][i] = cf[i];
        rdCmdTable[VOLTAGE_S]    [i] = s[i];
        rdCmdTable[VOLTAGE_TEMP] [i] = ax[i];
    }
}

BMS_StatusTypeDef readCellVoltage(VoltageTypes voltTypes) {
    static bool tableInit = false;
    if (!tableInit) { initCmdTable(); tableInit = true; }

    for (int i = 0; i < 6; i++) {
        if (!rdCmdTable[voltTypes][i]) continue;
        spiReadData(TOTAL_IC, rdCmdTable[voltTypes][i],
                    (uint8_t *)rxData, rxPecErr, rxCmdCnt, RX_DATA);
        if (checkPecFault()) {
            g_statusFlags |= BMS_ERR_COMMS;
            return BMS_ERR_COMMS;
        }
        parseVoltage(rxData, bms.volt_cell[voltTypes], i);
    }

    calculateStats(voltTypes);
    return BMS_OK;
}

/* -----------------------------------------------------------------------
 * Leitura de registrador para depuração
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef readRegister(RegisterTypes regTypes) {
    uint8_t *cmd;
    switch (regTypes) {
        case REG_CONFIG_A: cmd = RDCFGA; break;
        case REG_CONFIG_B: cmd = RDCFGB; break;
        case REG_PWM_A:    cmd = RDPWMA; break;
        case REG_PWM_B:    cmd = RDPWMB; break;
        case REG_SID:      cmd = RDSID;  break;
        default: return BMS_ERR_COMMS;
    }
    spiReadData(TOTAL_IC, cmd, (uint8_t *)rxData, rxPecErr, rxCmdCnt, RX_DATA);
    if (checkPecFault()) return BMS_ERR_COMMS;
    return BMS_OK;
}

/* -----------------------------------------------------------------------
 * Medição auxiliar (NTC via GPIO)
 * Sequência Manchester: ADAX → PLAUX1 → lê mux1 → troca GPO → ADAX → lê mux0
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef auxMeasurement(void) {
    ADAX.ow  = 0;
    ADAX.ch  = 0x00;  /* todos os canais */
    ADAX.ch4 = 0;
    ADAX.pup = 0;

    /* Mux canal 1 */
    spiSendCmd((uint8_t *)&ADAX);
    bmsPollAdc(PLAUX1);

    spiReadData(TOTAL_IC, RDAUXA, (uint8_t *)rxData, rxPecErr, rxCmdCnt, RX_DATA);
    if (checkPecFault()) return BMS_ERR_COMMS;

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int j = 0; j < 3 && j < TOTAL_CELL/2; j++) {
            int16_t raw = (int16_t)((rxData[ic][j*2+1] << 8) | rxData[ic][j*2]);
            bms.v_aux[ic][j*2 + 1] = AUX_VOLTAGE_V(raw);
        }
    }

    /* Mux canal 0 */
    bms_setGpo45(0x00);
    delayMsActive(1);

    spiSendCmd((uint8_t *)&ADAX);
    bmsPollAdc(PLAUX1);

    spiReadData(TOTAL_IC, RDAUXA, (uint8_t *)rxData, rxPecErr, rxCmdCnt, RX_DATA);
    if (checkPecFault()) {
        bms_setGpo45(0x03);  /* restaura GPO */
        return BMS_ERR_COMMS;
    }

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int j = 0; j < 3 && j < TOTAL_CELL/2; j++) {
            int16_t raw = (int16_t)((rxData[ic][j*2+1] << 8) | rxData[ic][j*2]);
            bms.v_aux[ic][j*2] = AUX_VOLTAGE_V(raw);
        }
    }

    bms_setGpo45(0x03);  /* restaura GPO */

    /* Converte tensão AUX → temperatura °C */
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            bms.temp_cell[ic][c] = bms_safety_aux_to_celsius(bms.v_aux[ic][c]);
        }
    }

    return BMS_OK;
}

/* -----------------------------------------------------------------------
 * Balanceamento / Descarga
 * ----------------------------------------------------------------------- */

/** Cálculo e início de descarga proporcional */
void startDischarge(float threshold) {
    bms_balancing_task(bms.volt_cell[DISCHARGE_VOLT_TYPE],
                       bms.cfb_Tx, bms.pwma, bms.pwmb,
                       &g_balState, threshold);

    /* Marca células em descarga */
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        bms.isDischarging[ic] = 0;
        for (int c = 0; c < TOTAL_CELL; c++) {
            if (g_balState.cell_active[ic][c])
                BIT_SET(bms.isDischarging[ic], c);
        }
    }
}

void stopDischarge(void) {
    bms_balancing_stop_all(bms.cfb_Tx, bms.pwma, bms.pwmb);
    for (int ic = 0; ic < TOTAL_IC; ic++) {
        bms.isDischarging[ic] = 0;
    }
}

/** Dispara S-ADC (para PWM brevemente) para medição durante balanceamento */
BMS_StatusTypeDef balancingMeasurementVoltage(void) {
    ADSV.cont = 0;
    ADSV.dcp  = 0;
    ADSV.ow   = 0x00;
    spiSendCmd((uint8_t *)&ADSV);
    bmsPollAdc(PLSADC);
    return readCellVoltage(VOLTAGE_S);
}

void startBalancing(float delta_v) {
    float threshold = g_balState.v_min + delta_v;
    startDischarge(threshold);
}

/* -----------------------------------------------------------------------
 * Verificações de status
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef checkVoltage(void) {
    return bms_safety_check_voltage(
        bms.volt_cell[DISCHARGE_VOLT_TYPE], &g_safetyState);
}

BMS_StatusTypeDef checkTemp(void) {
    return bms_safety_check_temperature(bms.v_aux, &g_safetyState);
}

BMS_StatusTypeDef checkCur(void)       { return BMS_OK; /* TODO: shunt ADC */ }
BMS_StatusTypeDef checkCommsFault(void) { return (g_statusFlags & BMS_ERR_COMMS); }

/* -----------------------------------------------------------------------
 * Reset suave
 * ----------------------------------------------------------------------- */
void bms_reset(void) {
    stopDischarge();
    bmsWakeupIc(TOTAL_IC);
    spiSendCmd(SRST);
    HAL_Delay(10);
    bms_init();
}

/* -----------------------------------------------------------------------
 * Loop principal
 * ----------------------------------------------------------------------- */
BMS_StatusTypeDef programLoop(void) {
    static uint32_t owCheckTick = 0;
    const uint32_t  OW_CHECK_INTERVAL_MS = 5000;

    BMS_StatusTypeDef status;

    /* 1. Lê tensões de célula (S-ADC para balanceamento) */
    bmsWakeupIc(TOTAL_IC);
    status = balancingMeasurementVoltage();
    if (status != BMS_OK) return status;

    /* 2. Verifica tensão */
    status = checkVoltage();
    if (status != BMS_OK) {
        stopDischarge();
        bms_safety_trigger_shutdown((bms_fault_t)g_safetyState.fault_mask);
        g_statusFlags |= status;
        return status;
    }

    /* 3. Lê temperatura */
    bmsWakeupIc(TOTAL_IC);
    status = auxMeasurement();
    if (status != BMS_OK) return status;

    status = checkTemp();
    if (status != BMS_OK) {
        stopDischarge();
        bms_safety_trigger_shutdown((bms_fault_t)g_safetyState.fault_mask);
        g_statusFlags |= status;
        return status;
    }

    /* 4. Open wire periódico (a cada OW_CHECK_INTERVAL_MS) */
    uint32_t now = HAL_GetTick();
    if (now - owCheckTick > OW_CHECK_INTERVAL_MS) {
        owCheckTick = now;
        /* TODO: implementar sequência real PUP/PDOWN */
        bms_safety_check_open_wire(bms.volt_cell[DISCHARGE_VOLT_TYPE],
                                   bms.volt_cell[DISCHARGE_VOLT_TYPE],
                                   &g_safetyState);
    }

    /* 5. Balanceamento (se habilitado e sem falhas) */
    g_statusFlags = BMS_OK;
    if (g_balancingEnabled) {
        bmsWakeupIc(TOTAL_IC);
        startBalancing(g_balDelta);
    }

    return BMS_OK;
}

/* -----------------------------------------------------------------------
 * Controle de carga / balanceamento
 * ----------------------------------------------------------------------- */
void BMS_EnableBalancing(bool enabled) { g_balancingEnabled = enabled; }
void BMS_ToggleBalancing(void)         { g_balancingEnabled = !g_balancingEnabled; }
bool BMS_IsCharging(void)              { return false; /* TODO: charger logic */ }
void BMS_EnableCharging(bool enabled)  { (void)enabled; /* TODO */ }
void BMS_ChargingButtonLogic(void)     { /* TODO */ }

void BMS_SetCommsFault(bool state) {
    if (state) g_statusFlags |=  BMS_ERR_COMMS;
    else       g_statusFlags &= ~BMS_ERR_COMMS;
}

void BMS_WriteFaultSignal(bool state) {
    /* FAULT_CTRL_Pin / FAULT_CTRL_GPIO_Port definidos no main.h do Manchester */
#if defined(FAULT_CTRL_Pin) && defined(FAULT_CTRL_GPIO_Port)
    HAL_GPIO_WritePin(FAULT_CTRL_GPIO_Port, FAULT_CTRL_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    (void)state;
}

/* -----------------------------------------------------------------------
 * CAN – prepara buffer de mensagens (estrutura da Manchester)
 * ----------------------------------------------------------------------- */
#define BASE_CAN_ID  0xB000
#define CAN_BUF_LEN  (TOTAL_IC * TOTAL_CELL + TOTAL_IC + 4)

static CanTxMsg canTxBuf[CAN_BUF_LEN];

void BMS_GetCanData(CanTxMsg **buff, uint32_t *len) {
    uint32_t idx = 0;

    FDCAN_TxHeaderTypeDef hdr = {
        .IdType             = FDCAN_EXTENDED_ID,
        .TxFrameType        = FDCAN_DATA_FRAME,
        .DataLength         = 8,
        .ErrorStateIndicator= FDCAN_ESI_ACTIVE,
        .BitRateSwitch      = FDCAN_BRS_OFF,
        .FDFormat           = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker      = 0,
    };

    for (int ic = 0; ic < TOTAL_IC; ic++) {
        for (int c = 0; c < TOTAL_CELL; c++) {
            int16_t v    = (int16_t)(bms.volt_cell[DISCHARGE_VOLT_TYPE][ic][c] * 1000.0f);
            int16_t dif  = (int16_t)(bms.volt_cell_dif[DISCHARGE_VOLT_TYPE][ic][c] * 1000.0f);
            int16_t temp = (int16_t)(bms.temp_cell[ic][c] * 100.0f);
            uint8_t flags = (uint8_t)(
                ((BIT_CHECK(bms.isDischarging[ic], c) ? 1 : 0) << 0) |
                ((BIT_CHECK(bms.isCellFaultDetect[ic], c) ? 1 : 0) << 1));

            canTxBuf[idx].data[0] = (uint8_t)(v   & 0xFF);
            canTxBuf[idx].data[1] = (uint8_t)(v   >> 8);
            canTxBuf[idx].data[2] = (uint8_t)(dif & 0xFF);
            canTxBuf[idx].data[3] = (uint8_t)(dif >> 8);
            canTxBuf[idx].data[4] = (uint8_t)(temp & 0xFF);
            canTxBuf[idx].data[5] = (uint8_t)(temp >> 8);
            canTxBuf[idx].data[6] = flags;
            canTxBuf[idx].data[7] = 0;

            hdr.Identifier = BASE_CAN_ID + (uint32_t)(ic * TOTAL_CELL + c);
            canTxBuf[idx].header = hdr;
            idx++;
        }
    }

    *buff = canTxBuf;
    *len  = idx;
}

/* Alias para compatibilidade com o header */
void getCanData(CanTxMsg **buff, uint32_t *len) { BMS_GetCanData(buff, len); }

/* Stubs extras exigidos pelo header */
BMS_StatusTypeDef readV(void)       { return BMS_OK; }
BMS_StatusTypeDef readCurrent(void) { return BMS_OK; }
