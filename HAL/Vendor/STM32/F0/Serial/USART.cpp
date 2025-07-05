#include "USART.h"

#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_dma.h>
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_usart.h>

#include <cassert>
#include <cmath>

namespace {
openstm::hal::stmicro::f0::USART* pUSART1 = nullptr;
openstm::hal::stmicro::f0::USART* pUSART2 = nullptr;
openstm::hal::stmicro::f0::USART* pUSART3 = nullptr;
openstm::hal::stmicro::f0::USART* pUSART4 = nullptr;
}  // namespace

namespace openstm::hal::stmicro::f0 {

USART::DMAMasks::DMAMasks(USART_TypeDef* usart) {
  if (usart == USART1) {
    TXChannel = LL_DMA_CHANNEL_2;
    TXGI = DMA_ISR_GIF2;
    TXHT = DMA_ISR_HTIF2;
    TXTC = DMA_ISR_TCIF2;
    TXTE = DMA_ISR_TEIF2;

    TXClearGI = DMA_IFCR_CGIF2;
    TXClearHT = DMA_IFCR_CHTIF2;
    TXClearTC = DMA_IFCR_CTCIF2;
    TXClearTE = DMA_IFCR_CTEIF2;

    RXChannel = LL_DMA_CHANNEL_3;
    RXGI = DMA_ISR_GIF3;
    RXHT = DMA_ISR_HTIF3;
    RXTC = DMA_ISR_TCIF3;
    RXTE = DMA_ISR_TEIF3;

    RXClearGI = DMA_IFCR_CGIF3;
    RXClearHT = DMA_IFCR_CHTIF3;
    RXClearTC = DMA_IFCR_CTCIF3;
    RXClearTE = DMA_IFCR_CTEIF3;
  } else if (usart == USART2) {
    TXChannel = LL_DMA_CHANNEL_4;
    TXGI = DMA_ISR_GIF4;
    TXHT = DMA_ISR_HTIF4;
    TXTC = DMA_ISR_TCIF4;
    TXTE = DMA_ISR_TEIF4;

    TXClearGI = DMA_IFCR_CGIF4;
    TXClearHT = DMA_IFCR_CHTIF4;
    TXClearTC = DMA_IFCR_CTCIF4;
    TXClearTE = DMA_IFCR_CTEIF4;

    RXChannel = LL_DMA_CHANNEL_5;
    RXGI = DMA_ISR_GIF5;
    RXHT = DMA_ISR_HTIF5;
    RXTC = DMA_ISR_TCIF5;
    RXTE = DMA_ISR_TEIF5;

    RXClearGI = DMA_IFCR_CGIF5;
    RXClearHT = DMA_IFCR_CHTIF5;
    RXClearTC = DMA_IFCR_CTCIF5;
    RXClearTE = DMA_IFCR_CTEIF5;

  } else if (usart == USART3) {
    TXChannel = LL_DMA_CHANNEL_7;
    TXGI = DMA_ISR_GIF7;
    TXHT = DMA_ISR_HTIF7;
    TXTC = DMA_ISR_TCIF7;
    TXTE = DMA_ISR_TEIF7;

    TXClearGI = DMA_IFCR_CGIF7;
    TXClearHT = DMA_IFCR_CHTIF7;
    TXClearTC = DMA_IFCR_CTCIF7;
    TXClearTE = DMA_IFCR_CTEIF7;

    RXChannel = LL_DMA_CHANNEL_6;
    RXGI = DMA_ISR_GIF6;
    RXHT = DMA_ISR_HTIF6;
    RXTC = DMA_ISR_TCIF6;
    RXTE = DMA_ISR_TEIF6;

    RXClearGI = DMA_IFCR_CGIF6;
    RXClearHT = DMA_IFCR_CHTIF6;
    RXClearTC = DMA_IFCR_CTCIF6;
    RXClearTE = DMA_IFCR_CTEIF6;
  } else if (usart == USART4) {
    TXChannel = LL_DMA_CHANNEL_7;
    TXGI = DMA_ISR_GIF7;
    TXHT = DMA_ISR_HTIF7;
    TXTC = DMA_ISR_TCIF7;
    TXTE = DMA_ISR_TEIF7;

    TXClearGI = DMA_IFCR_CGIF7;
    TXClearHT = DMA_IFCR_CHTIF7;
    TXClearTC = DMA_IFCR_CTCIF7;
    TXClearTE = DMA_IFCR_CTEIF7;

    RXChannel = LL_DMA_CHANNEL_6;
    RXGI = DMA_ISR_GIF6;
    RXHT = DMA_ISR_HTIF6;
    RXTC = DMA_ISR_TCIF6;
    RXTE = DMA_ISR_TEIF6;

    RXClearGI = DMA_IFCR_CGIF6;
    RXClearHT = DMA_IFCR_CHTIF6;
    RXClearTC = DMA_IFCR_CTCIF6;
    RXClearTE = DMA_IFCR_CTEIF6;
  }
}

USART::USART(PinID txPin, PinID rxPin, GPIO_TypeDef* gpiox,
             USART_TypeDef* usart, std::uint32_t baudRate, size_t txBufferSize,
             size_t rxBufferSize, bool txDMAEnabled, bool rxDMAEnabled)
    : USART_Base(txPin, rxPin, baudRate, txBufferSize, rxBufferSize),
      m_GPIOx(gpiox),
      m_USART(usart),
      m_TxDMAEnabled(txDMAEnabled),
      m_RxDMAEnabled(rxDMAEnabled),
      m_DMAMask(usart) {
  if (m_USART == USART1) {
    pUSART1 = this;
  } else if (m_USART == USART2) {
    pUSART2 = this;
  } else if (m_USART == USART3) {
    pUSART3 = this;
  } else if (m_USART == USART4) {
    pUSART4 = this;
  }
}

USART::USART(USART&& other)
    : USART_Base(std::forward<USART>(other)),
      m_GPIOx(other.m_GPIOx),
      m_USART(other.m_USART),
      m_TxDMAEnabled(other.m_TxDMAEnabled),
      m_RxDMAEnabled(other.m_RxDMAEnabled),
      m_DMAMask(other.m_DMAMask) {
  if (m_USART == USART1) {
    pUSART1 = this;
  } else if (m_USART == USART2) {
    pUSART2 = this;
  } else if (m_USART == USART3) {
    pUSART3 = this;
  } else if (m_USART == USART4) {
    pUSART4 = this;
  }
}

bool USART::IsRxIdle() const { return m_pActiveReceive == nullptr; }

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

  IRQn_Type txDMAIrqn{};
  IRQn_Type rxDMAIrqn{};

  if (m_USART == USART1) {
    NVIC_SetPriority(USART1_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART1_IRQn);
    txDMAIrqn = DMA1_Channel2_3_IRQn;
    rxDMAIrqn = DMA1_Channel2_3_IRQn;
  }
  if (m_USART == USART2) {
    NVIC_SetPriority(USART2_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART2_IRQn);
    txDMAIrqn = DMA1_Channel4_5_6_7_IRQn;
    rxDMAIrqn = DMA1_Channel4_5_6_7_IRQn;
  }
  if (m_USART == USART3) {
    NVIC_SetPriority(USART3_4_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_4_IRQn);
    txDMAIrqn = DMA1_Channel4_5_6_7_IRQn;
    rxDMAIrqn = DMA1_Channel4_5_6_7_IRQn;
  }
  if (m_USART == USART4) {
    NVIC_SetPriority(USART3_4_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_4_IRQn);
    txDMAIrqn = DMA1_Channel4_5_6_7_IRQn;
    rxDMAIrqn = DMA1_Channel4_5_6_7_IRQn;
  }

  if (m_TxDMAEnabled || m_RxDMAEnabled) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    if (m_TxDMAEnabled) {
      NVIC_SetPriority(txDMAIrqn,
                       NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
      NVIC_EnableIRQ(txDMAIrqn);

      LL_DMA_SetDataTransferDirection(DMA1, m_DMAMask.TXChannel,
                                      LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

      LL_DMA_SetChannelPriorityLevel(DMA1, m_DMAMask.TXChannel,
                                     LL_DMA_PRIORITY_LOW);

      LL_DMA_SetMode(DMA1, m_DMAMask.TXChannel, LL_DMA_MODE_NORMAL);

      LL_DMA_SetPeriphIncMode(DMA1, m_DMAMask.TXChannel,
                              LL_DMA_PERIPH_NOINCREMENT);

      LL_DMA_SetMemoryIncMode(DMA1, m_DMAMask.TXChannel,
                              LL_DMA_MEMORY_INCREMENT);

      LL_DMA_SetPeriphSize(DMA1, m_DMAMask.TXChannel, LL_DMA_PDATAALIGN_BYTE);

      LL_DMA_SetMemorySize(DMA1, m_DMAMask.TXChannel, LL_DMA_MDATAALIGN_BYTE);

      LL_DMA_EnableIT_TC(DMA1, m_DMAMask.TXChannel);
    }
    if (m_RxDMAEnabled) {
      NVIC_SetPriority(rxDMAIrqn,
                       NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
      NVIC_EnableIRQ(rxDMAIrqn);

      LL_DMA_SetDataTransferDirection(DMA1, m_DMAMask.RXChannel,
                                      LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

      LL_DMA_SetChannelPriorityLevel(DMA1, m_DMAMask.RXChannel,
                                     LL_DMA_PRIORITY_LOW);

      LL_DMA_SetMode(DMA1, m_DMAMask.RXChannel, LL_DMA_MODE_NORMAL);

      LL_DMA_SetPeriphIncMode(DMA1, m_DMAMask.RXChannel,
                              LL_DMA_PERIPH_NOINCREMENT);

      LL_DMA_SetMemoryIncMode(DMA1, m_DMAMask.RXChannel,
                              LL_DMA_MEMORY_INCREMENT);

      LL_DMA_SetPeriphSize(DMA1, m_DMAMask.RXChannel, LL_DMA_PDATAALIGN_BYTE);

      LL_DMA_SetMemorySize(DMA1, m_DMAMask.RXChannel, LL_DMA_MDATAALIGN_BYTE);
    }
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

IUSART::Result USART::SendBytes(std::span<const std::uint8_t> data) {
  Result rslt{};
  bool isComplete{false};
  SendBytesAsync(data, [&](Result sendRslt) {
    rslt = sendRslt;
    isComplete = true;
  });
  while (!isComplete) {
  }
  return rslt;
}

void USART::SendBytesAsync(
    std::span<const std::uint8_t> data,
    std::function<void(const Result&)> completionCallback) {
  SendMessage msg{data, std::move(completionCallback)};
  if (!PushTxBuffer(msg)) {
    completionCallback({ErrorCode::BufferFull});
  }

  if (m_TxDMAEnabled) {
    if (BufferedTxMessages() <= 1) {
      DisableDMAChannel(m_DMAMask.TXChannel);
      SetTXBuffer(data);
      LL_DMA_EnableChannel(DMA1, m_DMAMask.TXChannel);
      LL_DMA_EnableIT_TC(DMA1, m_DMAMask.TXChannel);
      LL_USART_EnableDMAReq_TX(m_USART);
    }
  } else {
    LL_USART_EnableIT_TXE(m_USART);
  }
}

IUSART::Result USART::ReceiveBytes(std::span<std::uint8_t> buffer,
                                   std::uint32_t timeout) {
  Result rslt;
  bool isComplete{false};
  ReceiveBytesAsync(buffer, timeout, [&](Result receiveRslt) {
    rslt = receiveRslt;
    isComplete = true;
  });
  while (!isComplete) {
  }
  return rslt;
}

void USART::ReceiveBytesAsync(
    std::span<std::uint8_t> buffer, std::uint32_t timeout,
    std::function<void(const Result&)> completionCallback) {
  Result rslt;

  if (m_pActiveReceive) {
    if (completionCallback) {
      completionCallback({ErrorCode::NotIdle});
      return;
    }
  }

  LL_USART_SetRxTimeout(m_USART, timeout);
  LL_USART_EnableRxTimeout(m_USART);

  if (m_RxDMAEnabled) {
    LL_USART_DisableIT_RXNE(m_USART);
    std::uint8_t nextByte{0};
    size_t sizePopped{0};
    while (PopRxBuffer(nextByte)) {
      buffer[sizePopped] = nextByte;
      sizePopped++;
    }

    ResetRxDMABuffer({buffer.begin() + sizePopped, buffer.end()});

    m_ActiveReceive = ReceiveMessage{.Buffer = buffer,
                                     .ReceivedSize = sizePopped,
                                     .Callback = std::move(completionCallback)};
  } else {
    LL_USART_EnableIT_RXNE(m_USART);
    if (RxBufferCount() >= buffer.size()) {
      for (size_t i = 0; i < buffer.size(); ++i) {
        std::uint8_t next{0};
        if (PopRxBuffer(next)) {
          buffer[i] = next;
        }
      }
      if (completionCallback) {
        completionCallback({buffer.size()});
      }
    }
    m_ActiveReceive = ReceiveMessage{.Buffer = buffer,
                                     .ReceivedSize = 0,
                                     .Callback = std::move(completionCallback)};
  }

  m_pActiveReceive = &m_ActiveReceive;
}

void USART::HandleUSARTInterrupt() {
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

void USART::HandleDMATxInterrupt() {
  if (READ_BIT(DMA1->ISR, m_DMAMask.TXTC) == m_DMAMask.TXTC) {
    SendMessage nextMsg{};
    if (PopTxBuffer(nextMsg)) {
      nextMsg.SentData = nextMsg.Data.size();
      if (nextMsg.CompletionCallback) {
        nextMsg.CompletionCallback({nextMsg.SentData});
      }
      if (PopTxBuffer(nextMsg)) {
        DisableDMAChannel(m_DMAMask.TXChannel);
        SetTXBuffer(nextMsg.Data);
        LL_DMA_EnableChannel(DMA1, m_DMAMask.TXChannel);
        LL_USART_EnableDMAReq_TX(m_USART);
      } else {
        LL_DMA_DisableIT_TC(DMA1, m_DMAMask.TXChannel);
        LL_USART_DisableDMAReq_TX(m_USART);
      }
    } else {
      LL_DMA_DisableIT_TC(DMA1, m_DMAMask.TXChannel);
      LL_USART_DisableDMAReq_TX(m_USART);
    }
  }
  WRITE_REG(DMA1->IFCR, m_DMAMask.TXClearGI);
}

void USART::HandleDMARxInterrupt() {
  if (m_pActiveReceive) {
    size_t dataLength = m_pActiveReceive->Buffer.size() -
                        m_pActiveReceive->ReceivedSize -
                        LL_DMA_GetDataLength(DMA1, m_DMAMask.RXChannel);
    m_pActiveReceive->ReceivedSize += dataLength;
    if (m_pActiveReceive->Callback) {
      if (READ_BIT(DMA1->ISR, m_DMAMask.RXTE) == m_DMAMask.RXTE) {
        m_pActiveReceive->Callback(
            {ErrorCode::DMAError, m_pActiveReceive->ReceivedSize});
      } else {
        m_pActiveReceive->Callback({m_pActiveReceive->ReceivedSize});
      }
    }
    m_pActiveReceive = nullptr;
  }
  WRITE_REG(DMA1->IFCR, m_DMAMask.RXClearGI);
  LL_USART_EnableIT_RXNE(m_USART);
}

void USART::DisableDMAChannel(std::uint32_t channel) {
  LL_DMA_DisableChannel(DMA1, channel);
  while (LL_DMA_IsEnabledChannel(DMA1, channel)) {
  }
}

void USART::SetTXBuffer(std::span<const std::uint8_t> buffer) {
  LL_DMA_ConfigAddresses(
      DMA1, m_DMAMask.TXChannel,
      reinterpret_cast<std::uintptr_t>(buffer.data()),
      LL_USART_DMA_GetRegAddr(m_USART, LL_USART_DMA_REG_DATA_TRANSMIT),
      LL_DMA_GetDataTransferDirection(DMA1, m_DMAMask.TXChannel));
  LL_DMA_SetDataLength(DMA1, m_DMAMask.TXChannel, buffer.size());
}

void USART::ResetRxDMABuffer(std::span<std::uint8_t> buffer) {
  DisableDMAChannel(m_DMAMask.RXChannel);
  LL_DMA_ConfigAddresses(
      DMA1, m_DMAMask.RXChannel,
      LL_USART_DMA_GetRegAddr(m_USART, LL_USART_DMA_REG_DATA_RECEIVE),
      reinterpret_cast<std::uintptr_t>(buffer.data()),
      LL_DMA_GetDataTransferDirection(DMA1, m_DMAMask.RXChannel));

  LL_DMA_SetDataLength(DMA1, m_DMAMask.RXChannel, buffer.size());

  LL_DMA_EnableIT_TE(DMA1, m_DMAMask.RXChannel);
  LL_DMA_EnableIT_TC(DMA1, m_DMAMask.RXChannel);

  LL_DMA_EnableChannel(DMA1, m_DMAMask.RXChannel);
  LL_USART_EnableDMAReq_RX(m_USART);
}

void USART::HandleReceiveError(ErrorCode code) {
  if (m_pActiveReceive) {
    if (m_RxDMAEnabled) {
      size_t dataLength = m_pActiveReceive->Buffer.size() -
                          m_pActiveReceive->ReceivedSize -
                          LL_DMA_GetDataLength(DMA1, m_DMAMask.RXChannel);
      m_pActiveReceive->ReceivedSize += dataLength;
      LL_USART_EnableIT_RXNE(m_USART);
    }
    if (m_pActiveReceive->Callback) {
      m_pActiveReceive->Callback({code, m_pActiveReceive->ReceivedSize});
    }
    m_pActiveReceive = nullptr;
  }
}

void USART::TransmitDataEmpty() {
  std::uint8_t nextByte{0};
  if (PopTxBuffer(nextByte)) {
    LL_USART_TransmitData8(m_USART, nextByte);
  } else {
    SendMessage nextMsg;
    if (PopTxBuffer(nextMsg)) {
      if (nextMsg.CompletionCallback) {
        nextMsg.CompletionCallback(nextMsg.SentData);
      }
    }
    LL_USART_DisableIT_TXE(m_USART);
  }
}

void USART::TransmissionComplete() {
  LL_USART_DisableIT_TXE(m_USART);
  LL_USART_DisableIT_TC(m_USART);
}

void USART::ReceiveByte(std::uint8_t nextByte) {
  if (m_pActiveReceive) {
    std::uint8_t nextBufferByte{0};
    while (PopRxBuffer(nextBufferByte)) {
      m_pActiveReceive->Buffer[m_pActiveReceive->ReceivedSize] = nextBufferByte;
      m_pActiveReceive->ReceivedSize++;
      if (m_pActiveReceive->ReceivedSize == m_pActiveReceive->Buffer.size()) {
        PushRxBuffer(nextBufferByte);
        if (m_pActiveReceive->Callback) {
          m_pActiveReceive->Callback({m_pActiveReceive->Buffer.size()});
        }
        m_pActiveReceive = nullptr;
        return;
      }
    }
    m_pActiveReceive->Buffer[m_pActiveReceive->ReceivedSize] = nextByte;
    m_pActiveReceive->ReceivedSize++;
    if (m_pActiveReceive->ReceivedSize == m_pActiveReceive->Buffer.size()) {
      PushRxBuffer(nextByte);
      if (m_pActiveReceive->Callback) {
        m_pActiveReceive->Callback({m_pActiveReceive->Buffer.size()});
      }
      m_pActiveReceive = nullptr;
      return;
    }
  } else {
    PushRxBuffer(nextByte);
  }
}

}  // namespace openstm::hal::stmicro::f0

extern "C" {
void USART1_IRQHandler(void) {
  if (pUSART1) {
    pUSART1->HandleUSARTInterrupt();
  }
}

void USART2_IRQHandler(void) {
  if (pUSART2) {
    pUSART2->HandleUSARTInterrupt();
  }
}

void USART3_IRQHandler(void) {
  if (pUSART3) {
    pUSART3->HandleUSARTInterrupt();
  }
}

void DMA1_Channel2_IRQHandler(void) {
  if (pUSART3) {
    pUSART3->HandleDMATxInterrupt();
  }
}

void DMA1_Channel3_IRQHandler(void) {
  if (pUSART3) {
    pUSART3->HandleDMARxInterrupt();
  }
}

void DMA1_Channel4_IRQHandler(void) {
  if (pUSART1) {
    pUSART1->HandleDMATxInterrupt();
  }
}

void DMA1_Channel5_IRQHandler(void) {
  if (pUSART1) {
    pUSART1->HandleDMARxInterrupt();
  }
}

void DMA1_Channel6_IRQHandler(void) {
  if (pUSART2) {
    pUSART2->HandleDMARxInterrupt();
  }
}

void DMA1_Channel7_IRQHandler(void) {
  if (pUSART2) {
    pUSART2->HandleDMATxInterrupt();
  }
}
}
