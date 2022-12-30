#pragma once

#ifndef BLE_STACK_H__
#define BLE_STACK_H__

#include "stack_user_cfg.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"

/* Default number of link */
#define MIN_NUM_LINK            1
/* Default number of GAP and GATT services */
#define DEFAULT_NUM_GATT_SERVICES   2
/* Default number of GAP and GATT attributes */
#define DEFAULT_NUM_GATT_ATTRIBUTES 9


/* Number of services requests */
#define NUM_APP_GATT_SERVICES 1

/* Number of attributes requests */
#define NUM_APP_GATT_ATTRIBUTES 15

#define SENSOR_MAX_ATT_SIZE (247) 

/* 
 * Number of links needed for the BLE app.
 * Only 1 by default (MIN_NUM_LINK).
 * --------------------------
 * Maximum number of simultaneous connections that the device will support. 
 * Valid values are from 1 to 8 (NUM_LINKS used in the calculation of TOTAL_BUFFER_SIZE).
 */
#define NUM_LINKS               (MIN_NUM_LINK)

/* 
 * Number of GATT attributes needed for the BLE app. 
 * ----------------------------
 * Maximum number of Attributes (i.e. the number of characteristic + the number 
 * of characteristic values + the number of descriptors, excluding the services) 
 * that can be stored in the GATT database. Note that certain characteristics and 
 * relative descriptors are added automatically during device initialization so 
 * this parameters should be 9 plus the number of user Attributes 
 * (NUM_GATT_ATTRIBUTES used in the calculation of TOTAL_BUFFER_SIZE)
 */
#define NUM_GATT_ATTRIBUTES     (DEFAULT_NUM_GATT_ATTRIBUTES + NUM_APP_GATT_ATTRIBUTES)

/* 
 * Number of GATT services needed for the BLE app. 
 * -----------------------------
 * Maximum number of Services that can be stored in the GATT database. Note that 
 * the GAP and GATT services are automatically added so this parameter should be 
 * 2 plus the number of user services (NUM_GATT_SERVICES used in the calculation 
 * of TOTAL_BUFFER_SIZE)
 */
#define NUM_GATT_SERVICES       (DEFAULT_NUM_GATT_SERVICES + NUM_APP_GATT_SERVICES)

/* 
 * Array size for the attribute value 
 * ----------------------------
 * Size of the storage area for Attribute values (ATT_VALUE_ARRAY_SIZE used 
 * in the calculation of TOTAL_BUFFER_SIZE).
 * This value depends on the number of attributes used by application. 
 * In particular the sum of the following quantities (in octets) 
 * should be made for each attribute:
 *   - attribute value length
 *   - 5, if UUID is 16 bit; 19, if UUID is 128 bit
 *   - 2, if server configuration descriptor is used
 *   - 2*numOfLinks, if client configuration descriptor is used
 *   - 2, if extended properties is used
 *
 * The total amount of memory needed is the sum of the above quantities for each attribute.
 */
#define ATT_VALUE_ARRAY_SIZE    (1600) 

/* 
 * Flash security database size
 * ----------------------------
 * Size of the database used to store security information for bonded devices 
 * (FLASH_SEC_DB_SIZE used in the calculation of TOTAL_FLASH_BUFFER_SIZE). 
 * Current SUPPORTED VALUE is 1024
*/
#define FLASH_SEC_DB_SIZE       (0x400)

/* 
 * Flash server database size
 * --------------------------
 * Size of the database used for service change notification for bonded devices 
 * (FLASH_SERVER_DB_SIZE used in the calculation of TOTAL_FLASH_BUFFER_SIZE). 
 * Current SUPPORTED VALUE is 1024
 */
#define FLASH_SERVER_DB_SIZE    (0x400)

/* Set supported max value for ATT_MTU enabled by the application. Allowed values in range: [23:158] [New parameter added on BLE stack v2.x] */
#define MAX_ATT_MTU             247

/* Set supported max value for attribute size: it is the biggest attribute size enabled by the application */
#define MAX_ATT_SIZE            (SENSOR_MAX_ATT_SIZE)  

/* 
 * Prepare Write List size in terms of number of packet with ATT_MTU bytes
 * ---------------------------------
 * Set the minumum number of prepare write requests needed for a long write 
 * procedure for a characteristic with len > 20bytes: 
 * 
 * It returns 0 for characteristics with len <= 20bytes
 * 
 * NOTE: If prepare write requests are used for a characteristic (reliable write on multiple characteristics), then 
 * this value should be set to the number of prepare write needed by the application.
*/
#define PREPARE_WRITE_LIST_SIZE PREP_WRITE_X_ATT(MAX_ATT_SIZE) 

/* 
 * Additional number of memory blocks  to be added to the minimum 
 * -------------------------------------
 * 6 for reaching the max throughput: ~220kbps (same as BLE stack 1.x)
 */
#define OPT_MBLOCKS		(6) /*  */

/* 
 * Set the number of memory block for packet allocation 
*/
#define MBLOCKS_COUNT           (MBLOCKS_CALC(PREPARE_WRITE_LIST_SIZE, MAX_ATT_MTU, NUM_LINKS) + OPT_MBLOCKS)

/* 
 * RAM reserved to manage all the data stack according the number of links,
 * number of services, number of attributes and attribute value length
 * -----------------------------------
 * Start address of the RAM buffer for GATT database allocated according to 
 * TOTAL_BUFFER_SIZE (32bit aligned RAM area) - its return value is used to 
 * check the MACRO correctness.
*/
NO_INIT(uint32_t dyn_alloc_a[TOTAL_BUFFER_SIZE(NUM_LINKS,NUM_GATT_ATTRIBUTES,NUM_GATT_SERVICES,ATT_VALUE_ARRAY_SIZE,MBLOCKS_COUNT,CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED)>>2]);

/* 
 * FLASH reserved to store all the security database information and
 * and the server database information.
 * ------------------------------------
 * Start address for the non-volatile memory area allocated according 
 * to TOTAL_FLASH_BUFFER_SIZE (Aligned to 2048 bytes flash sector boundary).
 */
NO_INIT_SECTION(uint32_t stacklib_flash_data[TOTAL_FLASH_BUFFER_SIZE(FLASH_SEC_DB_SIZE, FLASH_SERVER_DB_SIZE)>>2], ".noinit.stacklib_flash_data");

/* 
 * FLASH reserved to store: security root keys, static random address, public address 
 * ------------------------------------
 * stacklib_stored_device_id_data is address of the const device id data vector 
 * (56 bytes, 32bit aligned FLASH area, all elements must be initialized to 0xFF).
 */
NO_INIT_SECTION(uint8_t stacklib_stored_device_id_data[56], ".noinit.stacklib_stored_device_id_data");

// ****************************************
// HW STRUCTURE members
// ****************************************

/* Radio Config Hot Table */
extern uint8_t hot_table_radio_config[];

/* Maximum duration of the connection event */
#define MAX_CONN_EVENT_LENGTH 0xFFFFFFFF

/* Sleep clock accuracy */
#if (LS_SOURCE == LS_SOURCE_INTERNAL_RO)

  /* Sleep clock accuracy in Slave mode */
  #define SLAVE_SLEEP_CLOCK_ACCURACY 500

  /* Sleep clock accuracy in Master mode */
  #define MASTER_SLEEP_CLOCK_ACCURACY MASTER_SCA_500ppm

#else

  /* Sleep clock accuracy in Slave mode */
  #define SLAVE_SLEEP_CLOCK_ACCURACY 100

  /* Sleep clock accuracy in Master mode */
  #define MASTER_SLEEP_CLOCK_ACCURACY MASTER_SCA_100ppm

#endif

/* Low Speed Oscillator source */
#if (LS_SOURCE == LS_SOURCE_INTERNAL_RO)
  #define LOW_SPEED_SOURCE  1 // Internal RO
#else
  #define LOW_SPEED_SOURCE  0 // External 32 KHz
#endif

/* High Speed start up time */
#define HS_STARTUP_TIME 328 // 800 us

/* 
 * CREATE HW CONFIG STRUCTURE
 * Structure containing low level hardware configuration data for the device
*/
#define CONFIG_TABLE            \
{                               \
  (uint32_t*)hot_table_radio_config,          \
  MAX_CONN_EVENT_LENGTH,        \
  SLAVE_SLEEP_CLOCK_ACCURACY,   \
  MASTER_SLEEP_CLOCK_ACCURACY,  \
  LOW_SPEED_SOURCE,             \
  HS_STARTUP_TIME               \
}

// CREATE INIT STRUCUTRE
// This structure contains memory and low level hardware configuration data for the device

const BlueNRG_Stack_Initialization_t BlueNRG_Stack_Init_params = {
    (uint8_t*)stacklib_flash_data,
    FLASH_SEC_DB_SIZE,
    FLASH_SERVER_DB_SIZE,
    (uint8_t*)stacklib_stored_device_id_data,
    (uint8_t*)dyn_alloc_a,
    TOTAL_BUFFER_SIZE(NUM_LINKS,NUM_GATT_ATTRIBUTES,NUM_GATT_SERVICES,ATT_VALUE_ARRAY_SIZE,MBLOCKS_COUNT,CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED),
    NUM_GATT_ATTRIBUTES,
    NUM_GATT_SERVICES,
    ATT_VALUE_ARRAY_SIZE,
    NUM_LINKS,
    0, /* extended_packet_length_enable - reserved for future use */
    PREPARE_WRITE_LIST_SIZE,
    MBLOCKS_COUNT,
    MAX_ATT_MTU,
    CONFIG_TABLE,
};

/** 
 * @brief Number of application services
 */
#define NUMBER_OF_APPLICATION_SERVICES (1)

/* Maximum number of attribute records that can be added to the first application service: acceleration service */
#define MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1 (7) 

/* Set the Application Service Max number of attributes records with init parameters coming from application *.config.h file */
uint8_t Services_Max_Attribute_Records[NUMBER_OF_APPLICATION_SERVICES] = {MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1};

#endif
