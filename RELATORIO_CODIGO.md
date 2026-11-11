# Firmware BMS ADBMS6830B (STM32G474RE)

Referências: ADBMS6830B Rev. 0, FSAE Rules 2026 V1.0 (EV.7), repositório Manchester Stinger (`g474-bms`, branch `code-refactor`).

## 1. Visão geral
Firmware que lê 16 células por IC via SPI e verifica sobretensão, subtensão, temperatura do die, fio aberto e falhas de comunicação. Abre o Shutdown Circuit (SDC) via GPIO e faz balanceamento passivo.

Fluxo: `main()` → `bms_init()` → `while(1) programLoop()`.

## 2. Hardware / configuração (CubeMX)
| Item | Valor |
|---|---|
| Clock | HSI → PLL, 170 MHz |
| SPI2 | Master, modo 0, 8 bits, prescaler 128 → 1,33 MHz |
| CS | `BMS_CS` (PC0) por GPIO |
| TIM2 | prescaler 169 → 1 MHz (usado em `delayUs` e `bmsPollAdc`) |
| SDC | `BMS_CS2` (PC7), nível baixo = SDC aberto |
| FDCAN1/2 | gerados pelo CubeMX |
| UART | COM1 da BSP (115200) |

## 3. Módulos (`Core/Analog_libs`)
| Arquivo | Função |
|---|---|
| `bms_cmd.h` / `bms_generic.c` | Tabela de comandos, PEC15/PEC10, envio/leitura/escrita SPI, `bmsPollAdc` (timeout 100 ms), instâncias `ADCV/ADSV/ADAX` |
| `bms_data.h` | Structs de registradores (CFGA/B, PWM, status), conversões de tensão e ITMP, limites de segurança |
| `mcu_wrapper.c` | Camada HAL: CS, SPI, timer, wake-up, delays |
| `comm.c` / `communication.h` | Aplicação: `bms_init`, leitura de tensão, ITMP, balanceamento, `programLoop`, montagem de dados CAN |
| `bms_safety.c` | Verificações de OV/UV, temperatura do die, fio aberto, flags de hardware, `bms_safety_trigger_shutdown` |
| `bms_balancing.c` | Balanceamento (bottom balancing), codificação VOV/VUV, escrita de DCC/PWM |

## 4. Ciclo do `programLoop()`
1. Wake-up, ADSV single-shot, leitura S-ADC (`RDSVA..F`). Falha de comunicação abre o SDC.
2. `checkVoltage()`: OV 4,20 V / UV 2,50 V. Falha para a descarga e abre o SDC.
3. ADAX ITEMP + `RDSTATA` → `temp_ic`, limite −40…125 °C.
4. A cada 5 s: fio aberto (MUTE, ADCV com OW=0/1/2, UNMUTE, restaura ADCV contínuo).
5. Balanceamento (se habilitado e sem falhas): DCC e PWM; caso contrário `stopDischarge()`.

## 5. Proteções implementadas
- Sobretensão e subtensão por célula (software, S-ADC).
- Sobretemperatura do die (ITMP).
- Fio aberto por queda de tensão com chave de OW (C-ADC).
- Falha de comunicação (PEC ou timeout) abre o SDC.
- Falhas ficam travadas em `fault_mask` até reset do MCU.
- Balanceamento bloqueado com falha ativa.
- Timer de descarga do chip (DCTO = 1 min) e watchdog de 2 s como proteção contra travamento do MCU.

## 6. Documentação de referência
Datasheet ADBMS6830B Rev. 0, regulamento FSAE 2026 V1.0 e documentação técnica anterior (`BMS_ADBMS6830_Documentacao_Tecnica.docx`).
