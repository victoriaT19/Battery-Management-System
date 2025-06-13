#include "main.h"

#include <stdio.h>
#include <math.h>

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
ADC_HandleTypeDef hadc5;


FDCAN_HandleTypeDef hfdcan1;

uint16_t mux_raw[16];
float mux_voltage[16];
float mux_temp[16];
int minTempSeg[5];
int maxTempSeg[5];
int somaTempSeg[5];
int id_minSeg[5];
int id_maxSeg[5];
uint32_t last_print_time = 0;
float temp;
uint32_t val;
uint32_t val2 =0;
float vout = 0.0f;
float temperature;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);

void select_mux_channel(uint8_t channel) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  channel & 0x01);        // S0
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  (channel >> 1) & 0x01); // S1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, (channel >> 2) & 0x01); // S2
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (channel >> 3) & 0x01); // S3
}

uint16_t read_adc(uint16_t mux) {


  ADC_HandleTypeDef* adc_list[] = { &hadc1, &hadc2, &hadc3, &hadc4, &hadc5};

  if (mux >= sizeof(adc_list)/sizeof(adc_list[0])) {
      return 0;
  }

  HAL_ADC_Start(adc_list[mux]);
  HAL_ADC_PollForConversion(adc_list[mux], HAL_MAX_DELAY);
  return HAL_ADC_GetValue(adc_list[mux]);


    // ver ioc como representar o hadc dos outros valores
    /*HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);*/
}

// Conversão simples de tensão para temperatura (ajuste se necessário)
float voltage_to_temperature(float vout)
{
    const float voltage_table[] = {
        2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11,
        2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55,
        1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33,
        1.32, 1.31, 1.30
    };

    const float temp_table[] = {
        -40, -35, -30, -25, -20, -15, -10,  -5,   0,   5,
         10,  15,  20,  25,  30,  35,  40,  45,  50,  55,
         60,  65,  70,  75,  80,  85,  90,  95, 100, 105,
        110, 115, 120
    };

    const int table_size = sizeof(voltage_table) / sizeof(voltage_table[0]);

    // Fora da faixa
    if (vout > voltage_table[0] || vout < voltage_table[table_size - 1])
        printf("FORA DO INTERVALO!!!\n");

    // Procura o intervalo correto
    for (int i = 0; i < table_size - 1; i++)
    {
        if (vout <= voltage_table[i] && vout >= voltage_table[i + 1])
        {
            // Interpolação linear
            float v1 = voltage_table[i];
            float v2 = voltage_table[i + 1];
            float t1 = temp_table[i];
            float t2 = temp_table[i + 1];

            return t1 + ((vout - v1) / (v2 - v1)) * (t2 - t1);
        }
    }

    // Se não encontrou (por segurança)
    return -999.0f;
}

void send_address_claim() {
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t data[8] = {0xF3, 0x00, 0x80, 0x00, 0x40, 0x1E, 0x90, 0x00};

    txHeader.Identifier = 0x18EEFF80;
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

void send_thermistor_summary(int8_t minT, int8_t maxT, int8_t avgT, uint8_t count, uint8_t id_max, uint8_t id_min) {
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t data[8];

    data[0] = 0x00;
    data[1] = minT;
    data[2] = maxT;
    data[3] = avgT;
    data[4] = count;
    data[5] = id_max;
    data[6] = id_min;

    uint16_t checksum = 0x39 + 8;
    for (int i = 0; i < 7; i++) checksum += data[i];
    data[7] = (uint8_t)(checksum & 0xFF);

    txHeader.Identifier = 0x1839F380;
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

int main(void)
{


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  // Inicializa o vetor

  for (int i = 0; i < 16; i++) {
      mux_raw[i] = 0;
      mux_voltage[i] = 0.0f;
      mux_temp[i] = -999.0f;
  }
  last_print_time = HAL_GetTick();


  HAL_FDCAN_Start(&hfdcan1); // Inicializa a CAN


  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  while (1)
  {

	  if (HAL_GetTick() - last_print_time >= 500) {
	      float soma = 0;
	      int8_t minT = 127, maxT = -128;
	      uint8_t id_min = 0, id_max = 0;

        for(int i = 0; i < 5; i++){
          minTempSeg[i] = 127;
          maxTempSeg[i] = -128;
          somaTempSeg = 0, id_minSeg = 0, id_maxSeg = 0;
        }

	      for (uint8_t i = 0; i < 16; i++) {
          select_mux_channel(i);
          HAL_Delay(2);
          for(uint8_t j = 0; j < 5; j++){
            int cel_id = j * 16 + i; // caso queira printar o id da celula msm
            uint16_t raw = read_adc(j);
            float volt = (raw / 4095.0f) * 3.3f;
            float temp = voltage_to_temperature(volt);

            mux_raw[i] = raw;
            mux_voltage[i] = volt;
            mux_temp[i] = temp;

            if (temp > maxTempSeg[j]){
                maxTempSeg[j] = temp;
                id_maxSeg[j] = i;
            }
            if(temp > minTempSeg[j]){
                minTempSeg[j] = temp;
                id_minSeg[j] = i;
            }
              /*maior de tds os segmentos*/
            if (temp > maxT) {
                maxT = temp;
                id_max = i;
            }
            if (temp < minT) { /*menor de tds os segmentos*/
                minT = temp;
                id_min = i;
            }
            somaTempSeg[j] += temp;
            soma += temp;
          }
	      }

        send_address_claim();

        for(int i = 0; i < 5; i++){
          int8_t avgT = roundf(somaTempSeg[i]/ 16.0f);
	        send_thermistor_summary(minTempSeg[i], maxTempSeg[i], avgT, 16, id_maxSeg[i], id_minSeg[i]); 
        }

	      int8_t avgT = roundf(soma / 16.0f);//do jeito que está essa media total é de todos os segmentos

	      send_thermistor_summary(minT, maxT, avgT, 16, id_max, id_min); //de tds os segmentos

	      last_print_time = HAL_GetTick();
	  }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{


  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};


  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_FDCAN1_Init(void)
{

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 34;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 4;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
