#include "inc/i2c.h"
#include "inc/uart.h"


volatile FlagStatus i2c_eot = SET;
static uint8_t i2c_bus = 0;
static uint8_t i2c_addr = 0;

void i2c_init(void)
{
  /* Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_I2C2, ENABLE);
	
	I2C_InitType i2c_settings;
	I2C_StructInit(&i2c_settings);
	i2c_settings.I2C_ClockSpeed = 400000;
  i2c_settings.I2C_OperatingMode = I2C_OperatingMode_Master;
	
	I2C_Init(I2C2, &i2c_settings);
	
	/*
  // Configure the interrupt source
  I2C_ITConfig(I2C2, I2C_IT_MTDWS , ENABLE);
	
  // Enable the Interrupt
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MED_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/
	
  I2C_ClearITPendingBit(I2C2, I2C_IT_MSK);
	
	I2C_Cmd(I2C2, ENABLE);
}

void i2c_change_bus(uint8_t bus)
{
  if (i2c_bus == bus) {
    return;
  }
  for (uint8_t pin = 1; pin <= 4; pin <<= 1) {
    GPIO_WriteBit(pin, (BitAction) ((bus & pin) > 0));
  }
  i2c_bus = bus;
}

void i2c_change_addr(uint8_t addr)
{
  i2c_addr = addr >> 1;
}

int i2c_write(uint8_t reg, uint8_t* data, uint8_t len)
{	
	// I2C write without IRQ
	I2C_TransactionType t;
  
  // Write the slave address
  t.Operation = I2C_Operation_Write;
  t.Address = i2c_addr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = 1+len;
  
  // Flush the slave address
  I2C_FlushTx(I2C2);
  while (I2C_WaitFlushTx(I2C2) == I2C_OP_ONGOING);
  
  // Begin transaction
  I2C_BeginTransaction(I2C2, &t);

  // Fill TX FIFO with data to write
  I2C_FillTxFIFO(I2C2, reg);

  for(uint8_t i=0; i<len;i++) {
    I2C_FillTxFIFO(I2C2, data[i]);
  }
  
  // Wait loop
  do {
    if(I2C_GetStatus(I2C2) == I2C_OP_ABORTED) {
      return ERROR;
		}
  } while (I2C_GetITStatus(I2C2, I2C_IT_MTD) == RESET);
    
  // Clear pending bits
  I2C_ClearITPendingBit(I2C2, I2C_IT_MTD | I2C_IT_MTDWS);

  return SUCCESS;
	
	/*// I2C write with IRQ
	I2C_TransactionType t;
  
  // Write the slave address
  t.Operation = I2C_Operation_Write;
  t.Address = i2c_addr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = 1+len;
  
  // Flush the slave address
  I2C_FlushTx(I2C2);
  while (I2C_WaitFlushTx(I2C2) == I2C_OP_ONGOING);
  
  // Begin transaction
  i2c_eot = RESET;
  I2C_BeginTransaction(I2C2, &t);

  // Fill TX FIFO with data to write
  I2C_FillTxFIFO(I2C2, reg);

  for(uint8_t i=0; i<len;i++) {
    I2C_FillTxFIFO(I2C2, data[i]);
  }
  
  // Wait loop
  while(i2c_eot == RESET);
  
  return SUCCESS;
	*/
}

int i2c_read(uint8_t reg, uint8_t* data, uint8_t len)
{
	// I2C read without IRQ
	I2C_TransactionType t;
  
  // Write the slave address
  t.Operation = I2C_Operation_Write;
  t.Address = i2c_addr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Disable;
  t.Length = 1;  
  
  // Flush the slave address
  I2C_FlushTx(I2C2);
  while (I2C_WaitFlushTx(I2C2) == I2C_OP_ONGOING);
    
  // Begin transaction
  I2C_BeginTransaction(I2C2, &t);
	
  // Fill TX FIFO with data to write
  I2C_FillTxFIFO(I2C2, reg);

  // Wait loop
  do {
    if(I2C_GetStatus(I2C2) == I2C_OP_ABORTED) {
			printf("I2C_OP_ABORTED write\r\n");
      return ERROR;
		}
  } while (I2C_GetITStatus(I2C2, I2C_IT_MTDWS) == RESET);
  
  // Clear pending bits
  I2C_ClearITPendingBit(I2C2, I2C_IT_MTDWS);
  
  // read data
  t.Operation = I2C_Operation_Read;
  t.Address = i2c_addr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = len;  
  I2C_BeginTransaction(I2C2, &t);
  
  // Wait loop
  do {
    if(I2C_OP_ABORTED == I2C_GetStatus(I2C2)){
			printf("I2C_OP_ABORTED read\r\n");
      return ERROR;
		}
    
  } while (RESET == I2C_GetITStatus(I2C2, I2C_IT_MTD));
  
  // Clear pending bits
  I2C_ClearITPendingBit(I2C2, I2C_IT_MTD | I2C_IT_MTDWS);
  
  // Get data from RX FIFO
  while(len--) {
    *data = I2C_ReceiveData(I2C2);
    data ++;
  }
  
  return SUCCESS;
	
	/* // I2C read with IRQ
	I2C_TransactionType t;
	
    // Write the slave address
  t.Operation = I2C_Operation_Write;
  t.Address = i2c_addr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Disable;
  t.Length = 1;  
	
  // Flush the slave address
  I2C_FlushTx(I2C2);
  while (I2C_WaitFlushTx(I2C2) == I2C_OP_ONGOING);
	
  // Begin transaction
  i2c_eot = RESET;  
  I2C_BeginTransaction(I2C2, &t);
	
  // Fill TX FIFO with data to write
  I2C_FillTxFIFO(I2C2, reg);
	
  // Wait loop
  while(i2c_eot == RESET); // HERE IT STUCKS
	
  // read data
  t.Operation = I2C_Operation_Read;
  t.Address = i2c_addr;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = len; 
	
  // Begin transaction
  i2c_eot = RESET;  
  I2C_BeginTransaction(I2C2, &t);
	
  // Wait loop
  while(i2c_eot == RESET);
	
  // Get data from RX FIFO
  while(len--) {
    *data = I2C_ReceiveData(I2C2);
    data ++;
  }
	
  return SUCCESS;*/
}

void I2C2_Handler(void)
{
  // Check DMA_CH_I2C_RX_IT_TC
  if(I2C_GetITStatus(I2C2, I2C_IT_MTDWS)) {
    I2C_ClearITPendingBit(I2C2, I2C_IT_MTD | I2C_IT_MTDWS);
    
    // Set the i2c_eot flag
    i2c_eot = SET;    
  }
}

uint8_t i2c_read_reg(uint8_t reg)
{
	uint8_t read = 0;
	i2c_read(reg, &read, 1);
	return read;
}
