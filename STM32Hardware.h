#ifndef _STM32_HARDWARE_H_
#define _STM32_HARDWARE_H_


UART_HandleTypeDef UartHandle;
uint8_t Buffer_1;
uint8_t *RxBuffer = &Buffer_1;

//extern __IO ITStatus TxReady;
extern __IO ITStatus RxReady;


class STM32Hardware
{
  public:
    STM32Hardware() {}

    // Initialize the ATM32
    void init()
    {
      /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
      /* UART configured as follows:
          - Word Length = 8 Bits
          - Stop Bit = One Stop bit
          - Parity = None
          - BaudRate = 57600 baud
          - Hardware flow control disabled (RTS and CTS signals) */
      UartHandle.Instance        = USART6;  //uart6 > virtual, uart3 > gpio

      UartHandle.Init.BaudRate     = 57600;
      UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
      UartHandle.Init.StopBits     = UART_STOPBITS_1;
      UartHandle.Init.Parity       = UART_PARITY_NONE;
      UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
      UartHandle.Init.Mode         = UART_MODE_TX_RX;
      UartHandle.Init.OverSampling = UART_OVERSAMPLING_16; // compensate baud error

//      if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
//      {
//        Error_Handler();
//      }
      if(HAL_UART_Init(&UartHandle) != HAL_OK)
      {
        Error_Handler();
      }

      HAL_UART_Receive_IT(&UartHandle, RxBuffer, 1);
    }

    // Read a byte of data from ROS connection.
    // If no data , hal_uart-timeout, returns -1
    int read()
    {
//      return (HAL_UART_Receive(&UartHandle, RxBuffer, 1, 15) == HAL_OK) ? *RxBuffer : -1;
      if (RxReady == SET)
      {
        RxReady = RESET;
        HAL_UART_Receive_IT(&UartHandle, RxBuffer, 1);
        return *RxBuffer;
      }
      else
        return -1;
     }

    // Send a byte of data to ROS connection
    void write(uint8_t* data, int length)
    {
      HAL_UART_Transmit(&UartHandle, (uint8_t*)data, (uint16_t)length, HAL_MAX_DELAY);
//      HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)data, (uint16_t)length);
//      while (TxReady != SET);
//      TxReady = RESET;
    }

    // Returns milliseconds since start of program
    unsigned long time(void)
    {
      return HAL_GetTick();
    }

};

#endif

