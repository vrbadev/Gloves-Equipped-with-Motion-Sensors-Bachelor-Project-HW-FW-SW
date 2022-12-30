#ifndef BLE_APP_H__
#define BLE_APP_H__

/* This file contains all the information needed to init the BlueNRG-1 stack. 
 * These constants and variables are used from the BlueNRG-1 stack to reserve RAM and FLASH 
 * according the application requests
 */

#include "ble_const.h"

/* Firmware version */
#define BLE_APP_VERSION_STRING "1.0.0"

#define ADV_INTERVAL_MIN_MS  1000
#define ADV_INTERVAL_MAX_MS  1200

/* Select HAL VTimer ID */
#define SELECTED_TIMER 0

/* BLE characteristic update period in millis */
#define DATA_REFRESH_PERIOD_MS 30


#define MAXIMUM_CHARACTERISTICS 2
#define MAXIMUM_SERVICES 1
#define MAXIMUM_ADV_DATA_VAL 16

/* Specify if privacy is enabled or not and which one */
#define GAP_PRIVACY_DISABLED 0
#define GAP_PRIVACY_HOST_ENABLED 1
#define GAP_PRIVACY_CONTROLLER_ENABLED 2


/* Connection handle to notify. */
#define GATT_NOTIFY_ALL 0x0

/* Allow notification or Indication generation, if enabled in the client characteristic configuration descriptor. */
#define GATT_UPDATE_DO_NOT_NOTIFY 0
#define GATT_UPDATE_NOTIFICATION 1
#define GATT_UPDATE_INDICATION 2

#define GAP_ID_PUBLIC_ADDRESS 0
#define GAP_ID_RANDOM_STATIC_ADDRESS 1

#define CHAR_VALUE_SIZE_FIXED 0
#define CHAR_VALUE_SIZE_VARIABLE 1

typedef struct {
  uint8_t Offset;
  uint8_t Length;
} ble_hal_config_data_t;

typedef struct {
  uint8_t En_High_Power;
  uint8_t PA_Power;
} ble_hal_tx_power_level_t;

typedef struct {
  uint8_t Role;
  uint8_t privacy_enabled;
  uint8_t Bonding_Mode;
  uint8_t MITM_Mode;
  uint8_t SC_Support;
  uint8_t KeyPress_Notification_Support;
  uint8_t Min_Encryption_Key_Size;
  uint8_t Max_Encryption_Key_Size;
  uint8_t Use_Fixed_Pin;
  uint32_t Fixed_Pin;
  uint8_t Identity_Address_Type;
} ble_gap_t;

typedef struct {
  uint16_t handle;
  uint8_t Char_UUID_Type;
  Char_UUID_t uuid;
  uint16_t Char_Value_Length;
  uint8_t Char_Properties;
  uint8_t Security_Permissions;
  uint8_t GATT_Evt_Mask;
  uint8_t Enc_Key_Size;
  uint8_t Is_Variable;
} ble_gatt_characteristic_t;

typedef struct {
  uint16_t handle;
  Service_UUID_t uuid;
  uint8_t Service_UUID_Type;
  uint8_t Service_Type;
  uint8_t Max_Attribute_Records;
  ble_gatt_characteristic_t gatt_characteristics[MAXIMUM_CHARACTERISTICS];
  uint8_t gatt_characteristics_num;
} ble_gatt_service_t;

typedef struct {
  uint8_t* device_adv;
  uint8_t device_adv_len;
  uint8_t device_addr[6];
  ble_hal_config_data_t hal_config_data;
  ble_hal_tx_power_level_t hal_tx_power_level;
  ble_gap_t gap;
  ble_gatt_service_t gatt_services[MAXIMUM_SERVICES];
  uint8_t gatt_services_num;
} ble_app_t;

/* Public functions definitions */
tBleStatus ble_app_init(ble_app_t* app);
void ble_app_tick(void);
void ble_app_set_connectable(ble_app_t* app);
tBleStatus ble_app_update_char_value(uint16_t gatt_serv_handle, uint16_t gatt_char_handle, uint8_t* buff, uint8_t numbytes);
uint32_t get_time_ms(void);
void ble_stack_tick(void);
void ble_sleep(void);

#endif
