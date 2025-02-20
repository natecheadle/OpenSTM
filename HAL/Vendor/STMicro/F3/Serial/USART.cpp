#include "USART.h"

#include <stm32f3xx_ll_bus.h>
#include <stm32f3xx_ll_gpio.h>
#include <stm32f3xx_ll_usart.h>

#include <cassert>
#include <cmath>

namespace {
openstm::hal::stmicro::f3::USART* pUSART1 = nullptr;
openstm::hal::stmicro::f3::USART* pUSART2 = nullptr;
openstm::hal::stmicro::f3::USART* pUSART3 = nullptr;
}  // namespace

namespace openstm::hal::stmicro::f3 {
USART::USART(PinID txPin, PinID rxPin, GPIO_TypeDef* gpiox,
             USART_TypeDef* usart, std::uint32_t baudRate)
    : USART_Base(txPin, rxPin, baudRate), m_GPIOx(gpiox), m_USART(usart) {
  if (m_USART == USART1) {
    pUSART1 = this;
  } else if (m_USART == USART2) {
    pUSART2 = this;
  } else if (m_USART == USART3) {
    pUSART3 = this;
  }
}

USART::USART(USART&& other)
    : USART_Base(std::forward<USART>(other)),
      m_GPIOx(other.m_GPIOx),
      m_USART(other.m_USART) {
  if (m_USART == USART1) {
    pUSART1 = this;
  } else if (m_USART == USART2) {
    pUSART2 = this;
  } else if (m_USART == USART3) {
    pUSART3 = this;
  }
}

bool USART::IsRxIdle() const { return m_pActiveReceive == nullptr; }

bool USART::IsTxIdle() const { return m_pActiveSend == nullptr; }

void USART::Initialize() {
  LL_USART_InitTypeDef USART_InitStruct = {};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  GPIO_InitStruct.Pin =
      static_cast<std::uint32_t>(TXPin()) | static_cast<std::uint32_t>(RXPin());
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(m_GPIOx, &GPIO_InitStruct);

  if (m_USART == USART1) {
    NVIC_SetPriority(USART1_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART1_IRQn);
  }
  if (m_USART == USART2) {
    NVIC_SetPriority(USART2_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART2_IRQn);
  }
  if (m_USART == USART3) {
    NVIC_SetPriority(USART3_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);
  }

  USART_InitStruct.BaudRate = BaudRate();
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(m_USART, &USART_InitStruct);
  LL_USART_DisableIT_CTS(m_USART);
  LL_USART_ConfigAsyncMode(m_USART);
  LL_USART_Enable(m_USART);

  LL_USART_SetRxTimeout(m_USART, BaudRate());
  LL_USART_EnableRxTimeout(m_USART);
  LL_USART_ClearFlag_RTO(m_USART);
  LL_USART_EnableIT_RTO(m_USART);
  LL_USART_EnableIT_RXNE(m_USART);
}

IUSART::Result USART::SendBytes(const std::uint8_t* pData, std::size_t size) {
  Result rslt{};
  bool isComplete{false};
  SendBytesAsync(pData, size, [&](Result sendRslt) {
    rslt = sendRslt;
    isComplete = true;
  });
  while (!isComplete) {
  }
  return rslt;
}

void USART::SendBytesAsync(
    const std::uint8_t* pData, std::size_t size,
    std::function<void(const Result&)> completionCallback) {
  if (m_pActiveSend) {
    if (completionCallback) {
      completionCallback({ErrorCode::NotIdle});
      return;
    }
  }
  size_t txSize{0};
  for (size_t i = 0; i < size; ++i) {
    if (!TxBuffer().Push(pData[i])) {
      break;  // for()
    }
    ++txSize;
  }

  if (txSize == 0) {
    if (completionCallback) {
      completionCallback({});
    }
  }

  m_ActiveSend = SendMessage{
      .Size = txSize, .Sent = 0, .Callback = std::move(completionCallback)};
  m_pActiveSend = &m_ActiveSend;

  LL_USART_EnableIT_TXE(m_USART);
}

IUSART::Result USART::ReceiveBytes(std::uint8_t* pData, std::size_t maxSize,
                                   std::uint32_t timeout) {
  Result rslt;
  bool isComplete{false};
  ReceiveBytesAsync(pData, maxSize, timeout, [&](Result receiveRslt) {
    rslt = receiveRslt;
    isComplete = true;
  });
  while (!isComplete) {
  }
  return rslt;
}

void USART::ReceiveBytesAsync(
    std::uint8_t* pData, std::size_t maxSize, std::uint32_t timeout,
    std::function<void(const Result&)> completionCallback) {
  Result rslt;

  if (m_pActiveReceive) {
    if (completionCallback) {
      completionCallback({ErrorCode::NotIdle});
      return;
    }
  }

  if (RxBuffer().BufferedCount() >= maxSize) {
    for (size_t i = 0; i < maxSize; ++i) {
      std::uint8_t next{0};
      RxBuffer().Pop(next);
      pData[i] = next;
    }
    if (completionCallback) {
      completionCallback({maxSize});
    }
  }

  LL_USART_SetRxTimeout(m_USART, timeout);
  LL_USART_EnableRxTimeout(m_USART);

  m_ActiveReceive = ReceiveMessage{.Buffer = pData,
                                   .BufferSize = maxSize,
                                   .ReceivedSize = 0,
                                   .Callback = std::move(completionCallback)};
  m_pActiveReceive = &m_ActiveReceive;
}

void USART::HandleInterrupt() {
  if (LL_USART_IsActiveFlag_RXNE(m_USART)) {
    ReceiveByte(LL_USART_ReceiveData8(m_USART));
  }

  if (LL_USART_IsActiveFlag_RTO(m_USART)) {
    LL_USART_ClearFlag_RTO(m_USART);
    if (IsErrorEnabled(ErrorCode::ReceiveTimeout)) {
      HandleReceiveError(ErrorCode::ReceiveTimeout);
    }
  }

  if (LL_USART_IsActiveFlag_TXE(m_USART)) {
    TransmitDataEmpty();
  }

  if (LL_USART_IsActiveFlag_TC(m_USART)) {
    LL_USART_ClearFlag_TC(m_USART);
    TransmissionComplete();
  }

  if (LL_USART_IsActiveFlag_PE(m_USART)) {
    LL_USART_ClearFlag_PE(m_USART);
    if (IsErrorEnabled(ErrorCode::Parity)) {
      HandleReceiveError(ErrorCode::Parity);
    }
  }

  if (LL_USART_IsActiveFlag_FE(m_USART)) {
    LL_USART_ClearFlag_FE(m_USART);
    if (IsErrorEnabled(ErrorCode::Framing)) {
      HandleReceiveError(ErrorCode::Framing);
    }
  }

  if (LL_USART_IsActiveFlag_NE(m_USART)) {
    LL_USART_ClearFlag_NE(m_USART);
    if (IsErrorEnabled(ErrorCode::NoiseDetected)) {
      HandleReceiveError(ErrorCode::NoiseDetected);
    }
  }

  if (LL_USART_IsActiveFlag_ORE(m_USART)) {
    LL_USART_ClearFlag_ORE(m_USART);
    if (IsErrorEnabled(ErrorCode::Overrun)) {
      HandleReceiveError(ErrorCode::Overrun);
    }
  }
}

void USART::HandleReceiveError(ErrorCode code) {
  if (m_pActiveReceive) {
    if (m_pActiveReceive->Callback) {
      m_pActiveReceive->Callback({code, m_pActiveReceive->ReceivedSize});
    }
    m_pActiveReceive = nullptr;
  }
}

void USART::TransmitDataEmpty() {
  std::uint8_t nextByte{0};
  if (TxBuffer().Pop(nextByte)) {
    LL_USART_TransmitData8(m_USART, nextByte);
    if (m_pActiveSend) {
      m_pActiveSend->Sent++;
      if (m_pActiveSend->Sent == m_pActiveSend->Size) {
        if (m_pActiveSend->Callback) {
          m_pActiveSend->Callback({m_pActiveSend->Sent});
        }
        m_pActiveSend = nullptr;
      }
    }
  }
  if (TxBuffer().IsEmpty()) {
    LL_USART_DisableIT_TXE(m_USART);
  }
}

void USART::TransmissionComplete() {
  LL_USART_DisableIT_TXE(m_USART);
  LL_USART_DisableIT_TC(m_USART);
}

void USART::ReceiveByte(std::uint8_t nextByte) {
  if (m_pActiveReceive) {
    while (!RxBuffer().IsEmpty()) {
      std::uint8_t next{0};
      if (RxBuffer().Pop(next)) {
        m_pActiveReceive->Buffer[m_pActiveReceive->ReceivedSize] = next;
        m_pActiveReceive->ReceivedSize++;
        if (m_pActiveReceive->ReceivedSize == m_pActiveReceive->BufferSize) {
          RxBuffer().Push(nextByte);
          if (m_pActiveReceive->Callback) {
            m_pActiveReceive->Callback({m_pActiveReceive->BufferSize});
          }
          m_pActiveReceive = nullptr;
          return;
        }
      }
    }
    m_pActiveReceive->Buffer[m_pActiveReceive->ReceivedSize] = nextByte;
    m_pActiveReceive->ReceivedSize++;
    if (m_pActiveReceive->ReceivedSize == m_pActiveReceive->BufferSize) {
      if (m_pActiveReceive->Callback) {
        m_pActiveReceive->Callback({m_pActiveReceive->BufferSize});
      }
      m_pActiveReceive = nullptr;
      return;
    }
  } else {
    RxBuffer().Push(nextByte);
  }
}

}  // namespace openstm::hal::stmicro::f3

extern "C" {
void USART1_IRQHandler(void) {
  if (pUSART1) {
    pUSART1->HandleInterrupt();
  }
}

void USART2_IRQHandler(void) {
  if (pUSART2) {
    pUSART2->HandleInterrupt();
  }
}

void USART3_IRQHandler(void) {
  if (pUSART3) {
    pUSART3->HandleInterrupt();
  }
}
}
