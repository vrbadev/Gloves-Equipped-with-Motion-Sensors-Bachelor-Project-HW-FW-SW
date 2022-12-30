#ifndef SENSORS_H__
#define SENSORS_H__

#include "../lib_sensors/inc/LSM6DSLSensor.h"
#include "../lib_sensors/inc/LPS22HBSensor.h"
#include "../lib_sensors/inc/LIS2MDLSensor.h"

#define SDA1 0
#define SDA2 1
#define SDA3 2
#define SDA4 3
#define SDA5 4
#define SDA6 5
#define SDA7 6
#define SDA8 7

#define SENSORS_NUM 17
#define FN_RAW_NUM 2
#define FN_SENS_NUM 2
#define PRIVATE_DATA_4BS 4
#define SENSOR_DATA_NUMBYTES_MAX 12

#define MEASUREMENT_LSM6DSL_SIZE 12
#define MEASUREMENT_LIS2MDL_SIZE 6
#define MEASUREMENT_LPS22HB_SIZE 8
#define MEASUREMENT_TIME_SIZE 4
#define MEASUREMENT_ADC_SIZE 4

#define MEASURED_DATA_MAX_SIZE (15*MEASUREMENT_LSM6DSL_SIZE + 1*MEASUREMENT_LIS2MDL_SIZE + 1*MEASUREMENT_LPS22HB_SIZE + MEASUREMENT_TIME_SIZE + MEASUREMENT_ADC_SIZE)

#define NONZERO(a) (a > 0 ? 1 : 0)
#define META_T_SIZE 5
#define FN_SENS_LSM6DSL_NUM 2
#define FN_SENS_LIS2MDL_NUM 1
#define FN_SENS_LPS22HB_NUM 0

#define CONF_DATA_MAX_SIZE (15*(FN_SENS_LSM6DSL_NUM*META_T_SIZE+NONZERO(FN_SENS_LSM6DSL_NUM)) + 1*(FN_SENS_LIS2MDL_NUM*META_T_SIZE+NONZERO(FN_SENS_LIS2MDL_NUM)) + 1*(FN_SENS_LPS22HB_NUM*META_T_SIZE+NONZERO(FN_SENS_LPS22HB_NUM)))

/* Type definitions */
typedef uint8_t (*fn_get_t)(uint8_t*);
typedef uint8_t (*fn_init_t)(void);

typedef struct {
  fn_init_t* fn_init;
  fn_get_t (*fn_get_raw)[FN_RAW_NUM];
  fn_get_t (*fn_get_sens)[FN_SENS_NUM];
} fn_pointers_t;

typedef struct {
  uint8_t i2c_bus;
  uint8_t sensor_addr;
  fn_pointers_t* fn_pointers;
  uint32_t private_data[PRIVATE_DATA_4BS];
} sensor_t;

typedef union {
  uint8_t u8[SENSOR_DATA_NUMBYTES_MAX];
  int16_t i16[SENSOR_DATA_NUMBYTES_MAX / sizeof(int16_t)];
  uint32_t u32[SENSOR_DATA_NUMBYTES_MAX / sizeof(uint32_t)];
  float f[SENSOR_DATA_NUMBYTES_MAX / sizeof(float)];
} sensor_data_t;

/* Public functions */
uint8_t sensor_init(sensor_t* sensor);
int8_t sensor_read_data(sensor_t* sensor, uint8_t* buffer);
int8_t sensor_read_conf(sensor_t* sensor, uint8_t* buffer);

#endif
