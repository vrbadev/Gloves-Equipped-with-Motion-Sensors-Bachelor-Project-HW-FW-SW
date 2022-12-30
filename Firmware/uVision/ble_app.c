#include "inc/ble_app.h"
#include "inc/ble_stack.h"
#include "inc/uart.h"

#include "sleep.h"
#include "osal.h"

/* Local variables */
static uint8_t vtimer_expired = FALSE;
static uint32_t app_start_time = 0;

/* Imported function for updating data characteristics */
extern void update_ble_data(void);
extern void update_ble_data_conf(void);

/*
  Function: ble_app_init
  -----------------------------
  BLE application initialisation steps:
  1. Public device address is set
  2. Trasmitter power is set
  3. GATT server is initialised
  4. GAP server is initialised, device name is defined
  5. Authentication is defined
  6. All GATT service and its characteristic are added
  7. Virtual timer is started
*/

tBleStatus ble_app_init(ble_app_t* app)
{
  uint8_t ret;
  uint16_t handle_gap_serv, handle_gap_char_name, handle_gap_char_appearance;

  /* Initialize BLE stack */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("APP init: Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    return ret;
  }
  
  /* Set the device public address */
  ret = aci_hal_write_config_data(app->hal_config_data.Offset, app->hal_config_data.Length, app->device_addr);  
  if(ret != BLE_STATUS_SUCCESS) {
    printf("APP init: aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  /* Set the TX power -2 dBm */
  aci_hal_set_tx_power_level(app->hal_tx_power_level.En_High_Power, app->hal_tx_power_level.PA_Power);
  
  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("APP init: aci_gatt_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  /* GAP Init */
  ret = aci_gap_init(app->gap.Role, app->gap.privacy_enabled, app->device_adv[3]-1, &handle_gap_serv, &handle_gap_char_name, &handle_gap_char_appearance);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("APP init: aci_gap_init() failed: 0x%02x\r\n", ret);
    return ret;
  }
 
  /* Update device name */
  ret = aci_gatt_update_char_value_ext(GATT_NOTIFY_ALL, handle_gap_serv, handle_gap_char_name, GATT_UPDATE_DO_NOT_NOTIFY, app->device_adv[3]-1, 0x0, app->device_adv[3]-1, &app->device_adv[5]);
  if(ret != BLE_STATUS_SUCCESS) {
    printf("APP init: aci_gatt_update_char_value_ext() failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  /* Setup client authentication settings */
  ret = aci_gap_set_authentication_requirement(app->gap.Bonding_Mode, 
                                               app->gap.MITM_Mode, 
                                               app->gap.SC_Support, 
                                               app->gap.KeyPress_Notification_Support, 
                                               app->gap.Min_Encryption_Key_Size, 
                                               app->gap.Max_Encryption_Key_Size, 
                                               app->gap.Use_Fixed_Pin, 
                                               app->gap.Fixed_Pin, 
                                               app->gap.Identity_Address_Type);
  if(ret != BLE_STATUS_SUCCESS) {
    printf("APP init: aci_gap_set_authentication_requirement() failed: 0x%02x\r\n", ret);
    return ret;
  } 

  for(uint8_t i = 0; i < app->gatt_services_num; i++) {
    /* Each GATT service is added and a new handle is assigned */
    ret = aci_gatt_add_service(app->gatt_services[i].Service_UUID_Type,
                               &app->gatt_services[i].uuid, 
                               app->gatt_services[i].Service_Type, 
                               app->gatt_services[i].Max_Attribute_Records, 
                               &app->gatt_services[i].handle); 
    if (ret != BLE_STATUS_SUCCESS) {
      printf("APP init: #%02d: aci_gatt_add_serv() failed: 0x%02x\r\n", i, ret);
      return ret;
    }
    
    for(uint8_t j = 0; j < app->gatt_services[i].gatt_characteristics_num; j++) {
      /* Each characteristic is added and a new handle is assigned */
      ret = aci_gatt_add_char(app->gatt_services[i].handle, 
                              app->gatt_services[i].gatt_characteristics[j].Char_UUID_Type, 
                              &app->gatt_services[i].gatt_characteristics[j].uuid, 
                              app->gatt_services[i].gatt_characteristics[j].Char_Value_Length, 
                              app->gatt_services[i].gatt_characteristics[j].Char_Properties, 
                              app->gatt_services[i].gatt_characteristics[j].Security_Permissions, 
                              app->gatt_services[i].gatt_characteristics[j].GATT_Evt_Mask, 
                              app->gatt_services[i].gatt_characteristics[j].Enc_Key_Size, 
                              app->gatt_services[i].gatt_characteristics[j].Is_Variable, 
                              &app->gatt_services[i].gatt_characteristics[j].handle);
      if (ret != BLE_STATUS_SUCCESS) {
        printf("APP init: #%02d-#%02d: aci_gatt_add_char() failed: 0x%02x\r\n", i, j, ret);
        return ret;
      }
    }
  }
  

  /* Start the Timer */
  ret = HAL_VTimerStart_ms(SELECTED_TIMER, DATA_REFRESH_PERIOD_MS);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("APP init: HAL_VTimerStart_ms() failed; 0x%02x\r\n", ret);
    return ret;
  } else {
    vtimer_expired = FALSE;
  }
  
  app_start_time = HAL_VTimerGetCurrentTime_sysT32();

  return BLE_STATUS_SUCCESS;
}


/*
  Function: ble_app_tick
  -----------------------
  Checks for expiracy of virtual HAL timer
*/
void ble_app_tick()
{    
  /* Update sensor value */
  if (vtimer_expired) {
    vtimer_expired = FALSE;
    if (HAL_VTimerStart_ms(SELECTED_TIMER, DATA_REFRESH_PERIOD_MS) != BLE_STATUS_SUCCESS) {
      vtimer_expired = TRUE;
    }
  }
}

void ble_stack_tick()
{
  BTLE_StackTick();
}

void ble_sleep()
{
  /* Power Save management */
  BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
}


/*
  Function: ble_app_update_char_value
  ---------------------------------
  Updates value of selected characteristic.
*/
tBleStatus ble_app_update_char_value(uint16_t gatt_serv_handle, uint16_t gatt_char_handle, uint8_t* buff, uint8_t numbytes)
{  
  tBleStatus ret;
  ret = aci_gatt_update_char_value_ext(GATT_NOTIFY_ALL, gatt_serv_handle, gatt_char_handle, GATT_UPDATE_NOTIFICATION, numbytes, 0x0, numbytes, buff);
  if (ret != BLE_STATUS_SUCCESS){
    printf("aci_gatt_update_char_value_ext() failed: 0x%02X\r\n",ret) ;
    return BLE_STATUS_ERROR;
  }
  
  return BLE_STATUS_SUCCESS;	
}


/*
  Function: ble_app_set_connectable
  ---------------------------------
  Makes BLE device discoverable for new clients. Updates advertising packet.
*/
void ble_app_set_connectable(ble_app_t* app)
{  
  uint8_t ret;  
  
  /* Provide data used in Scanning Packets that have a data field */
  hci_le_set_scan_response_data(0, NULL);
  
  printf("Set General Discoverable Mode.\r\n");
  
  /* Put the device in general discoverable mode */
  ret = aci_gap_set_discoverable(ADV_IND,
                                 (ADV_INTERVAL_MIN_MS*1000)/625,
                                 (ADV_INTERVAL_MAX_MS*1000)/625,
                                 STATIC_RANDOM_ADDR, 
                                 NO_WHITE_LIST_USE,
                                 app->device_adv[3], 
                                 &app->device_adv[4], 
                                 0, 
                                 NULL, 
                                 0, 
                                 0); 
  
  /* Update the advertising data */
  aci_gap_update_adv_data(app->device_adv_len, app->device_adv);
  
  if(ret != BLE_STATUS_SUCCESS) {
    printf("aci_gap_set_discoverable() failed: 0x%02x\r\n",ret);
  } else {
    printf("aci_gap_set_discoverable() --> SUCCESS\r\n");
  }
}


/*
  Function: hci_le_connection_complete_event
  ------------------------------------------
  This event indicates that a new connection has been created.
*/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  // Do nothing
  printf("Client Connected!\r\n");
}


/*
  Function: hci_disconnection_complete_event
  ------------------------------------------
  This event occurs when a connection is terminated.
*/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  // Do nothing
  printf("Client Disconnected!\r\n");
}

/*
  Function: aci_gatt_tx_pool_available_event
  ------------------------------------------
  This event is generated each time BLE FW stack raises 
  the error code BLE_STATUS_INSUFFICIENT_RESOURCES (0x64).
*/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle, uint16_t Available_Buffers)
{
  //update_ble_data();
}

/*
  Function: aci_gatt_read_permit_req_event
  ----------------------------------------
  This event is given when a read request is received by the server from the client.
*/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  if(Connection_Handle != 0) { 
    tBleStatus ret = aci_gatt_allow_read(Connection_Handle);
    if(ret != BLE_STATUS_SUCCESS) {
			printf("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
    }
  }   
}

/*
  Function: HAL_VTimerTimeoutCallback
  -----------------------------------
  This function will be called on the expiry of a one-shot virtual timer.
*/
void HAL_VTimerTimeoutCallback(uint8_t timerNum)
{
  if (timerNum == SELECTED_TIMER) {
    vtimer_expired = TRUE;
  }
}

uint32_t get_time_ms()
{
  return HAL_VTimerDiff_ms_sysT32(HAL_VTimerGetCurrentTime_sysT32(), app_start_time);
}
