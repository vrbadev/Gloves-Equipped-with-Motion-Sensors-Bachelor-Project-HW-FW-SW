#include "inc/sensors.h"
#include "inc/i2c.h"
#include "inc/uart.h"

/* Constant pointers to init and data reading functions of sensors */
const fn_init_t fn_init_lsm6dsl = (uint8_t(*)(void)) &LSM6DSLSensor_Init;
const fn_get_t fn_get_raw_lsm6dsl[FN_RAW_NUM] = {(uint8_t (*)(uint8_t*)) &LSM6DSLSensor_Get_X_AxesRaw, (uint8_t (*)(uint8_t*)) &LSM6DSLSensor_Get_G_AxesRaw};
const fn_get_t fn_get_conf_lsm6dsl[FN_SENS_NUM] = {(uint8_t (*)(uint8_t*)) &LSM6DSLSensor_Get_X_Sensitivity, (uint8_t (*)(uint8_t*)) &LSM6DSLSensor_Get_G_Sensitivity};
const fn_pointers_t fn_pointers_lsm6dsl = { (fn_init_t*) &fn_init_lsm6dsl, (fn_get_t(*)[FN_RAW_NUM]) &fn_get_raw_lsm6dsl, (fn_get_t(*)[FN_SENS_NUM]) &fn_get_conf_lsm6dsl };

const fn_init_t fn_init_lis2mdl = (uint8_t(*)(void)) &LIS2MDLSensor_Init;
const fn_get_t fn_get_raw_lis2mdl[FN_RAW_NUM] = {(uint8_t (*)(uint8_t*)) &LIS2MDLSensor_GetAxesRaw, NULL};
const fn_get_t fn_get_conf_lis2mdl[FN_SENS_NUM] = {(uint8_t (*)(uint8_t*)) &LIS2MDLSensor_GetSensitivity, NULL};
const fn_pointers_t fn_pointers_lis2mdl = { (fn_init_t*) &fn_init_lis2mdl, (fn_get_t(*)[FN_RAW_NUM]) &fn_get_raw_lis2mdl, (fn_get_t(*)[FN_SENS_NUM]) &fn_get_conf_lis2mdl };

const fn_init_t fn_init_lps22hb = (uint8_t(*)(void)) &LPS22HBSensor_Init;
const fn_get_t fn_get_raw_lps22hb[FN_RAW_NUM] = {(uint8_t (*)(uint8_t*)) &LPS22HBSensor_GetTemperature, (uint8_t (*)(uint8_t*)) &LPS22HBSensor_GetPressure};
const fn_get_t fn_get_conf_lps22hb[FN_SENS_NUM] = {NULL, NULL};
const fn_pointers_t fn_pointers_lps22hb = { (fn_init_t*) &fn_init_lps22hb, (fn_get_t(*)[FN_RAW_NUM]) &fn_get_raw_lps22hb, (fn_get_t(*)[FN_SENS_NUM]) &fn_get_conf_lps22hb };

/* Variable pointer for sensors to access their private data */
uint32_t* selected_sensor_private_data = 0;

/*
  Selects specified sensor:
  1. Changes pointer to private data for each sensor
  2. Changes I2C bus (SDA line) using MUX
  3. Changes I2C slave address
*/
void sensor_select(sensor_t* sensor)
{
  selected_sensor_private_data = sensor->private_data;
  i2c_change_bus(sensor->i2c_bus);
  i2c_change_addr(sensor->sensor_addr);
}


/*
  Initializes specified sensor using pre-defined function pointer.
*/
uint8_t sensor_init(sensor_t* sensor)
{
  sensor_select(sensor);
  fn_init_t* init = sensor->fn_pointers->fn_init;
  return (*init)();
  
  //return (*(*((uint8_t (***)(void)) (sensor->fn_pointers+0))))();
}

uint8_t fn_result_meta(void* ptr)
{
  if (ptr == &LSM6DSLSensor_Get_X_AxesRaw || ptr == &LSM6DSLSensor_Get_G_AxesRaw || ptr == &LIS2MDLSensor_GetAxesRaw) {
    return 3 * sizeof(int16_t);
  } else if (ptr == &LPS22HBSensor_GetTemperature || ptr == LPS22HBSensor_GetPressure) {
    return sizeof(float);
  } else if (ptr == &LSM6DSLSensor_Get_X_Sensitivity) {
    return 'A';
  } else if (ptr == &LSM6DSLSensor_Get_G_Sensitivity) {
    return 'G';
  } else if (ptr == &LIS2MDLSensor_GetSensitivity) {
    return 'M';
  }
  return 0;
}

/*
  Reads all data from sensor using pre-defined function pointers.
  Returns number of bytes read or -1 if error occurs.
*/
int8_t sensor_read_data(sensor_t* sensor, uint8_t* buffer)
{
  sensor_select(sensor);
  
  fn_get_t ((*fn_gets)[FN_RAW_NUM]) = (fn_get_t(*)[FN_RAW_NUM])*((fn_get_t**) &(sensor->fn_pointers->fn_get_raw));
  
  uint8_t i = 0;
  int8_t data_pos = 0;
  while(i < FN_RAW_NUM && (*fn_gets)[i]) {
    if((*fn_gets)[i](&(buffer[data_pos]))) {
      return -1;
    }
    data_pos += fn_result_meta((void*) ((*fn_gets)[i]));
    i++;
  }
  
  return data_pos;
}


/*
  Reads all necessary configuration (now just sensitivity) from the sensor.
  Returns number of bytes read or -1 if error occurs.
*/
int8_t sensor_read_conf(sensor_t* sensor, uint8_t* buffer)
{
  sensor_select(sensor);
  
  fn_get_t ((*fn_gets)[FN_SENS_NUM]) = (fn_get_t(*)[FN_SENS_NUM])*((fn_get_t**) &(sensor->fn_pointers->fn_get_sens));
  
  uint8_t i = 0;
  int8_t data_pos = 0;
  union { float f; uint8_t u8[4]; } un;
  while(i < FN_SENS_NUM && (*fn_gets)[i]) {
    if((*fn_gets)[i]((uint8_t*) &un.f)) {
      return -1;
    }
    buffer[data_pos++] = fn_result_meta((void*) ((*fn_gets)[i]));
    buffer[data_pos++] = un.u8[0];
    buffer[data_pos++] = un.u8[1];
    buffer[data_pos++] = un.u8[2];
    buffer[data_pos++] = un.u8[3];
    i++;
  }
  
  return data_pos;
}
 
