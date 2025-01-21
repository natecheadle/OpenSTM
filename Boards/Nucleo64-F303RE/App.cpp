#include "App.h"

#include <stm32f3xx_ll_bus.h>
#include <stm32f3xx_ll_exti.h>
#include <stm32f3xx_ll_gpio.h>
#include <stm32f3xx_ll_rcc.h>
#include <stm32f3xx_ll_system.h>
#include <stm32f3xx_ll_usart.h>
#include <stm32f3xx_ll_utils.h>

#include "IO/Digital/DigitalOut.hpp"
#include "Vendor/STMicro/F3/IO/DigitalOut.h"

namespace {
#define NVIC_PRIORITYGROUP_0                                   \
  ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority, \
                              4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1                                   \
  ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority, \
                              3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2                                   \
  ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority, \
                              2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3                                   \
  ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority, \
                              1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4                                   \
  ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority, \
                              0 bit  for subpriority */

#define B1_Pin LL_GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin LL_GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin LL_GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
}  // namespace

namespace openstm::hal::boards::nucleo64f303re {

DigitalOut<stmicro::f3::DigitalOut> App::LED2 =
    DigitalOut(stmicro::f3::DigitalOut(PinID::Five, GPIOA));

App::App(std::function<void()> update) : m_UpdateFunc(update) {}

void App::Initialize() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  ConfigSysClock();
  InitGPIO();
  InitUART();
}

void App::Update() { m_UpdateFunc(); }

void App::ConfigSysClock() {
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
  }
  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1) {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_9,
                              LL_RCC_PREDIV_DIV_1);
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1) {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

void App::InitGPIO() {
  LED2.Initialize();
}

void App::InitUART() {
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0, 0, 0, 0, 0, 0, 0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0, 0, 0, 0, 0, 0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 38400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART2);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}
}  // namespace openstm::hal::boards::nucleo64f303re
