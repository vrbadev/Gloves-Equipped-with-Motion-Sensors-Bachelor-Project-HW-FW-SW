/**************************************************************
File:         main.c
Application:  Smart gloves for gesture recognition
Date:         April 2020
Author:       Vojtech Vrba
              FEE CTU in Prague
              
Known bugs:   1) LED5 can't be connected to the pin DIO12 
                 (input-only) -> doesn't work
              2) BSS138 transistors for switching LEDs lack
                 pull-down resistors -> capacitance of traces
                 forces LEDs to turn on and off randomly
***************************************************************/

#include "BlueNRG_x_device.h"
#include "sleep.h"

#include "inc/delay.h"
#include "inc/uart.h"
#include "inc/adc.h"
#include "inc/i2c.h"
#include "inc/ble_app.h"
#include "inc/sensors.h"

/* Indication LEDs pins definition */
#define PIN_LED3 GPIO_Pin_3 // yellow
#define PIN_LED4 GPIO_Pin_6 // red
#define PIN_LED5 GPIO_Pin_12 // blue

/* MUX select pins definition */
#define PIN_S0 GPIO_Pin_0
#define PIN_S1 GPIO_Pin_1
#define PIN_S2 GPIO_Pin_2


/* 
  BLE application advertising packet (max 31 B)
  Each entry is in format:
    [len of next 2 fields in bytes], [AD type (1 B)], [entry value (len-1) B]
*/
const uint8_t device_adv[] = {
  2,  AD_TYPE_TX_POWER_LEVEL, 
      0x00 /* 0 dBm */,
  
  12, AD_TYPE_COMPLETE_LOCAL_NAME, 
      'S','m','a','r','t',' ','G','l','o','v','e',
  
  13, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 
      0x01, /*SKD version */
      0x04, /* 0x04: BlueNRG1-1/2 eval kits */
      0x00,
      0xD4, /* ACC+Gyro 0xC0 | 0x04 Temp | 0x10 Pressure*/
      0x00, /*  */
      0x00, /*  */
      0x00, /* BLE MAC start */
      0x00,
      0x00,
      0x00,
      0x00,
      0x00 /* BLE MAC stop */
};


/*
  BLE application initialization structure
  Contains all information needed for GAP and GATT setup
*/
static ble_app_t app =
{
  .device_adv = (uint8_t*) device_adv,
  .device_adv_len = sizeof(device_adv),
  .device_addr = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02},
  .hal_config_data = {
    .Offset = CONFIG_DATA_PUBADDR_OFFSET,
    .Length = CONFIG_DATA_PUBADDR_LEN
  },
  .hal_tx_power_level = {
    .En_High_Power = 1,
    .PA_Power = 4
  },
  .gap = {
    .Role = GAP_PERIPHERAL_ROLE,
    .privacy_enabled = GAP_PRIVACY_DISABLED,
    .Bonding_Mode = BONDING,
    .MITM_Mode = MITM_PROTECTION_REQUIRED,
    .SC_Support = SC_IS_NOT_SUPPORTED,
    .KeyPress_Notification_Support = KEYPRESS_IS_NOT_SUPPORTED,
    .Min_Encryption_Key_Size = 7,
    .Max_Encryption_Key_Size = 16,
    .Use_Fixed_Pin = USE_FIXED_PIN_FOR_PAIRING,
    .Fixed_Pin = 123456,
    .Identity_Address_Type = GAP_ID_PUBLIC_ADDRESS
  },
  .gatt_services = {
    [0] = {
      .uuid.Service_UUID_128 = {0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b},
      .Service_UUID_Type = UUID_TYPE_128,
      .Service_Type = PRIMARY_SERVICE,
      .Max_Attribute_Records = 1+3*4,
      .gatt_characteristics = {
        [0] = {
          .Char_UUID_Type = UUID_TYPE_128,
          .uuid.Char_UUID_128 = {0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b},
          .Char_Value_Length = MEASURED_DATA_MAX_SIZE,
          .Char_Properties = CHAR_PROP_NOTIFY,// | CHAR_PROP_READ,
          .Security_Permissions = ATTR_PERMISSION_NONE,
          .GATT_Evt_Mask = GATT_DONT_NOTIFY_EVENTS,
          .Enc_Key_Size = 16,
          .Is_Variable = CHAR_VALUE_SIZE_FIXED
        },
        [1] = {
          .Char_UUID_Type = UUID_TYPE_128,
          .uuid.Char_UUID_128 = {0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xbc,0x56,0x00,0x02,0xa5,0xd5,0xc5,0x1b},
          .Char_Value_Length = CONF_DATA_MAX_SIZE,
          .Char_Properties = CHAR_PROP_NOTIFY | CHAR_PROP_READ,
          .Security_Permissions = ATTR_PERMISSION_NONE,
          .GATT_Evt_Mask = GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
          .Enc_Key_Size = 16,
          .Is_Variable = CHAR_VALUE_SIZE_FIXED
        }
      },
      .gatt_characteristics_num = 2
    }
  },
  .gatt_services_num = 1
};


/* List of all connected sensors */
extern fn_pointers_t fn_pointers_lsm6dsl, fn_pointers_lis2mdl, fn_pointers_lps22hb;

static sensor_t ALL_SENSORS[SENSORS_NUM] = {
  // Sensor #00, SDA1, LSM6DSL, address LOW,  thumb,          proximal phalanx
  { SDA1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #01, SDA1, LSM6DSL, address HIGH, thumb,          distal phalanx
  { SDA1, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #02, SDA2, LSM6DSL, address LOW,  index finger,   intermediate phalanx
  { SDA2, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #03, SDA2, LSM6DSL, address HIGH, index finger,   distal phalanx
  { SDA2, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #04, SDA3, LSM6DSL, address HIGH, index finger,   proximal phalanx
  { SDA3, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #05, SDA3, LSM6DSL, address LOW,  middle finger,  proximal phalanx
  { SDA3, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #06, SDA4, LSM6DSL, address LOW,  middle finger,  intermediate phalanx
  { SDA4, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #07, SDA4, LSM6DSL, address HIGH, middle finger,  distal phalanx
  { SDA4, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #08, SDA5, LSM6DSL, address LOW,  ring finger,    intermediate phalanx
  { SDA5, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #09, SDA5, LSM6DSL, address HIGH, ring finger,    distal phalanx
  { SDA5, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #10, SDA6, LSM6DSL, address HIGH, ring finger,    proximal phalanx
  { SDA6, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #11, SDA6, LSM6DSL, address LOW,  little finger,  proximal phalanx
  { SDA6, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #12, SDA7, LSM6DSL, address LOW,  little finger,  intermediate phalanx
  { SDA7, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #13, SDA7, LSM6DSL, address HIGH, little finger,  distal phalanx
  { SDA7, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #14, SDA8, LSM6DSL, address HIGH, hand
  { SDA8, LSM6DSL_ACC_GYRO_I2C_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lsm6dsl },
  
  // Sensor #15, SDA8, LPS22HB, address HIGH, hand
  { SDA8, LPS22HB_ADDRESS_HIGH, (fn_pointers_t*) &fn_pointers_lps22hb },
  
  // Sensor #16, SDA8, LIS2MDL, address def., hand
  { SDA8, LIS2MDLSensor_I2C_ADD, (fn_pointers_t*) &fn_pointers_lis2mdl },
};

/* Arrays containing data to send */
uint8_t measured_data[MEASURED_DATA_MAX_SIZE] = {0};
uint8_t conf_data[CONF_DATA_MAX_SIZE] = {0};

adc_measurement_t adc_measurement = { .f = 0.0f };

volatile uint8_t sensors_initialized = 0;

/* Init all sensors */
uint8_t sensors_all_init(void)
{
  for (uint8_t i = 0; i < SENSORS_NUM; i++) {
    if(sensor_init(&ALL_SENSORS[i])) {
      printf("Error when initialising sensor #%02d!\r\n", i);
      return 1;
    }
  }
  return 0;
}

/* Gather data from all sensors + time + ADC */
void sensors_gather_data(void)
{
  uint8_t data_pos = 0;
  
  /* Add time to data packet */
  *((uint32_t*) &measured_data[data_pos]) = get_time_ms();
  data_pos += 4;
  
  /* Add battery voltage to data packet */
  measured_data[data_pos++] = adc_measurement.u8[0];
  measured_data[data_pos++] = adc_measurement.u8[1];
  measured_data[data_pos++] = adc_measurement.u8[2];
  measured_data[data_pos++] = adc_measurement.u8[3];
  
  /* Add sensors measurements to data packet */
  int8_t bytesRead = 0;
  for(uint8_t i = 0; i < SENSORS_NUM; i++) {
    //printf("Get sensor #%02d data to data pos %d / %d ... ", i, data_pos, MEASURED_DATA_MAX_SIZE);
    if((bytesRead = sensor_read_data(&ALL_SENSORS[i], &measured_data[data_pos])) <= 0) {
      printf("Failed to read sensor #%02d data to data pos %d / %d ... ", i, data_pos, MEASURED_DATA_MAX_SIZE);
    } else {
      //printf("success (read %dB)!\r\n", bytesRead);
      data_pos += bytesRead;
    }
  }
  
  /*int16_t i1 = *((int16_t*)&measured_data[8]);
  int16_t i2 = *((int16_t*)&measured_data[10]);
  int16_t i3 = *((int16_t*)&measured_data[12]);
  printf("Sensor #00 measurement: time: %u ms; A: [%d %d %d]\r\n", *((uint32_t*) &measured_data[0]), i1, i2, i3);*/
  
  /* Check if all bytes of packet were written */
  //if (data_pos != MEASURED_DATA_MAX_SIZE) {
    //printf("Weird, the data packet is not complete!\r\n");
  //}
  
  //printf("Gathered %dB\r\n", data_pos);
}

/* Update data packet containing necessary configurations of sensors. */
void update_ble_data_conf(void)
{
  uint8_t data_pos = 0;
  int8_t bytesRead = 0;
  for(uint8_t i = 0; i < SENSORS_NUM; i++) {
    conf_data[data_pos++] = i;
    if((bytesRead = sensor_read_conf(&ALL_SENSORS[i], &conf_data[data_pos])) <= 0) {
      data_pos--;
    } else {
      data_pos += bytesRead;
    }
  }
  
  ble_app_update_char_value(app.gatt_services[0].handle, app.gatt_services[0].gatt_characteristics[1].handle, (uint8_t*) &conf_data, data_pos);
}

void update_ble_data(void)
{
  uint8_t len = app.gatt_services[0].gatt_characteristics[0].Char_Value_Length;
  ble_app_update_char_value(app.gatt_services[0].handle, app.gatt_services[0].gatt_characteristics[0].handle, (uint8_t*) &measured_data, len);
}

void gpio_setup(void)
{
  /* Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
	
	// UART pins
  GPIO_InitUartTxPin8(); // TXD at DIO8
  GPIO_InitUartRxPin11(); // RXD at DIO11
	
	// I2C2 pins
	GPIO_InitI2c2ClkPin4(); // SCL at DIO4
	GPIO_InitI2c2DataPin5(); // SDA at DIO5
  
  // LED pins
  GPIO_InitOutputPinx(PIN_LED3); // LED3 at DIO3
  GPIO_InitOutputPinx(PIN_LED4); // LED4 at DIO6
  GPIO_InitOutputPinx(PIN_LED5); // LED5 at DIO12
  GPIO_ResetBits(PIN_LED3 | PIN_LED4 | PIN_LED5);
  
  // Select pins for MUX
  GPIO_InitOutputPinx(PIN_S0); // S0 at DIO0
  GPIO_InitOutputPinx(PIN_S1); // S1 at DIO1
  GPIO_InitOutputPinx(PIN_S2); // S2 at DIO2
}

void systick_ms_callback(uint32_t ms)
{
  if (ms % 1000 == 0) {
    adc_read(&adc_measurement);
    adc_measurement.f *= (43.0f/33.0f);
  }
}

int main(void)
{  
  uint32_t last_update_ms = 0, time = 0;
  
  SystemInit();
	gpio_setup();
  
  GPIO_SetBits(PIN_LED3);
  
	uart_init();
	systick_init();
	mft_init();
	adc_init();
  i2c_init();
  
  printf("\r\nBlueNRG-2 BLE Application (version: %s)\r\n", BLE_APP_VERSION_STRING);
  uint8_t ret;
  
  /* Initialize all sensors */
  if(sensors_all_init()) {
    printf("Failed to initialize all sensors!\r\n");
    goto fault_state;
  } else {
    printf("All sensors initialized successfully.\r\n");
  }
  
	/* BLE Device Init */
  ret = ble_app_init(&app);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in ble_app_init() 0x%02x\r\n", ret);
    goto fault_state;
  } else {    
    printf("BLE app initialized successfully.\r\n");
  }
  
  /* Put device in general discoverable mode */
  ble_app_set_connectable(&app);
  
  update_ble_data_conf();
	
  GPIO_ResetBits(PIN_LED3);
  
  sensors_initialized = 1;
  
  while(1) {	
    /* Update the data */
    time = get_time_ms();
    if (time - last_update_ms > (DATA_REFRESH_PERIOD_MS - 1)) {
      sensors_gather_data();
      update_ble_data();
      last_update_ms = time;
    }
    
    /* BLE stack tick */
    ble_stack_tick();
    
    /* Application Tick */
    ble_app_tick();
    
    /* BLE sleep for power saving */
    ble_sleep();
  }
  
  /* Fault state - red LED blinking quickly */
fault_state:
  while(1) {
    GPIO_ToggleBits(PIN_LED4);
    delay_ms(50UL);
  }
}
