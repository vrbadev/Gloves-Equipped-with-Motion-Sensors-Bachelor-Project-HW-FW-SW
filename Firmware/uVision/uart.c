#include "inc/uart.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE { uart_send((char) ch); return ch; }

void uart_init()
{
  UART_InitType UART_InitStructure;
  
  /* GPIO Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART, ENABLE);
  
  /* 
  ------------ USART configuration -------------------
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  UART_InitStructure.UART_BaudRate = 115200;
  UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
  UART_InitStructure.UART_WordLengthReceive = UART_WordLength_8b;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_Odd;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
  UART_InitStructure.UART_FifoEnable = DISABLE;
  UART_Init(&UART_InitStructure);
  
  /* FIFO levels */
  //UART_RxFifoIrqLevelConfig(FIFO_LEV_1_64);
	//UART_TxFifoIrqLevelConfig(FIFO_LEV_1_8);
	
  /* Interrupt as soon as data is received. */
	UART_ITConfig(UART_IT_RX, ENABLE);

  /* Enable the Interrupt */
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = UART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MED_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  /* Enable UART */
  UART_Cmd(ENABLE);
}

void uart_send(uint8_t tx_data)
{
  /* Wait if TX fifo is full. */
  while (UART_GetFlagStatus(UART_FLAG_TXFF) == SET);
  /* send the data */
  UART_SendData(tx_data);
}

void uart_sendstr(char* str)
{
	while(*str) uart_send(*(str++));
}

void UART_Handler(void)
{
	if (UART_GetITStatus(UART_IT_RX)) {
		uart_send(UART_ReceiveData()); // Echo
		
		UART_ClearITPendingBit(UART_IT_RX);
	}
}
