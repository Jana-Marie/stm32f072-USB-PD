#include "main.h"
#include "usb_device.h"
#include "tcpm_driver.h"
#include "tcpm.h"
#include "usb_pd.h"
#include "FUSB302.h"
#include "usb_pd.h"
#include "usb_pd_tcpm.h"
#include "tcpm.h"
#include "usb_pd_driver.h"
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

void SystemClock_Config(void);
void dfu_otter_bootloader(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
void my_init(void);
static uint32_t pd_src_caps[CONFIG_USB_PD_PORT_COUNT][PDO_MAX_OBJECTS];

const struct tcpc_config_t tcpc_config[CONFIG_USB_PD_PORT_COUNT] = {
  {0, fusb302_I2C_SLAVE_ADDR, &fusb302_tcpm_drv}
};
uint32_t otterPDO[5];
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  if (HAL_GPIO_ReadPin(GPIOA, BUTTON_Pin) == 1) {
    dfu_otter_bootloader();
  }

  tcpm_init(0);
  delayUs(10);
  pd_init(0);
  delayUs(10);
  //my_init();

  HAL_GPIO_WritePin(GPIOA, LED_POWER_Pin, 1);
  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, 0);

  delayUs(50000);

  //pd_request_source_voltage(0, 9000);


  //MX_USB_DEVICE_Init();
//pd_request_power_swap(0);
  //pd_update_dual_role_config(0);
  //dual_role_on();
  while (1)
  {
    //pd_set_new_power_request(0);
    //if(pd[0].task_state == PD_STATE_HARD_RESET_SEND){
    //  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, 1);
    //}
    //HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, HAL_GPIO_ReadPin(GPIOA, INT_N_Pin)^HAL_GPIO_ReadPin(GPIOA, BUTTON_Pin));
    /*
    uint32_t otter1 = 0;
    uint16_t otter2 = 1;//pd_find_pdo_index(0, 9000, &otter1);

    memset(str, ' ', 40);
    sprintf(&str[0], "Otter! %ld  %d\n\r", otter1 , otter2);

    CDC_Transmit_FS((unsigned char*)str, sizeof(str));
    */
    //printf("Otter\n");
    if (HAL_GPIO_ReadPin(GPIOA, INT_N_Pin) == 0) {
      tcpc_alert(0);

    }
    pd_run_state_machine(0);
    pd_find_pdo_index(0, 9000, otterPDO);

    pd_process_source_cap(0, 2, pd_src_caps);

    HAL_Delay(4);
  }
}

void pd_process_source_cap_callback(int port, int cnt, uint32_t *src_caps)
{
  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, 1);
}

void my_init() {
  uint8_t cfg = 0;
  tcpc_write(cfg, TCPC_REG_RESET, TCPC_REG_RESET_SW_RESET);

  tcpc_write(cfg, TCPC_REG_POWER, 0x0F);

  tcpc_write(cfg, TCPC_REG_MASK, 0x00);
  tcpc_write(cfg, TCPC_REG_MASKA, 0x00);
  tcpc_write(cfg, TCPC_REG_MASKB, 0x00);
  tcpc_write(cfg, TCPC_REG_CONTROL0, 0x04);

  tcpc_write(cfg, TCPC_REG_CONTROL3, 0x07);

  tcpc_write(cfg, TCPC_REG_CONTROL1, TCPC_REG_CONTROL1_RX_FLUSH);

  tcpc_write(cfg, TCPC_REG_SWITCHES0, 0x07);

  delayUs(250);
  int cc1 = 0;
  tcpc_read(cfg, TCPC_REG_STATUS0, &cc1);
  cc1 = cc1 & TCPC_REG_STATUS0_BC;

  tcpc_write(cfg, TCPC_REG_SWITCHES0, 0x0B);

  delayUs(250);
  int cc2 = 0;
  tcpc_read(cfg, TCPC_REG_STATUS0, &cc2);
  cc2 = cc2 & TCPC_REG_STATUS0_BC;

  if (cc1 > cc2) {
    tcpc_write(cfg, TCPC_REG_SWITCHES1, 0x25);
    tcpc_write(cfg, TCPC_REG_SWITCHES0, 0x07);
  } else {
    tcpc_write(cfg, TCPC_REG_SWITCHES1, 0x26);
    tcpc_write(cfg, TCPC_REG_SWITCHES0, 0x0B);
  }

  tcpc_write(cfg, TCPC_REG_RESET, TCPC_REG_RESET_PD_RESET);
}

void dfu_otter_bootloader(void)
{
  *((unsigned long *)0x20003FF0) = 0xDEADBEEF;
  NVIC_SystemReset();
}

uint32_t getUs(void) {
  uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;

  register uint32_t ms, cycle_cnt;
  do {
    ms = HAL_GetTick();
    cycle_cnt = SysTick->VAL;
  } while (ms != HAL_GetTick());

  return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t micros) {
  uint32_t start = getUs();
  while (getUs() - start < (uint32_t) micros) {
    asm("NOP");
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14
                                     | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20000209; //0x20000209
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);

  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);

  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);

  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C2);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, MOSFET_Pin | OLED_PULLUP_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = INT_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_N_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOSFET_Pin | OLED_PULLUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_STATUS_Pin | LED_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
