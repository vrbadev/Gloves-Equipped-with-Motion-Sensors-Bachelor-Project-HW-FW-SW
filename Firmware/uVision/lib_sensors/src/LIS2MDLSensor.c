/**
 ******************************************************************************
 * @file    LIS2MDLSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Implementation of an LIS2MDL 3 axes gyroscope sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "../inc/LIS2MDLSensor.h"
#include "../../inc/delay.h"

/* Private sensor data 
[0] -> mag_is_enabled
*/
extern uint32_t* selected_sensor_private_data;

/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_Init(void)
{
  /* Enable BDU */
  if (LIS2MDLSensor_block_data_update_set( PROPERTY_ENABLE) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  /* Operating mode selection - power down */
  if (LIS2MDLSensor_operating_mode_set( LIS2MDLSensor_POWER_DOWN) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  /* Output data rate selection */
  if (LIS2MDLSensor_data_rate_set( LIS2MDLSensor_ODR_100Hz) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  /* Self Test disabled. */
  if (LIS2MDLSensor_self_test_set( PROPERTY_DISABLE) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  selected_sensor_private_data[0] = 0;
  
  /* Enable magnetometer */
  return LIS2MDLSensor_Enable();
}

/**
 * @brief  Read component ID
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_ReadID(uint8_t *Id)
{
  if (LIS2MDLSensor_device_id_get(Id) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  return LIS2MDLSensor_OK;
}


/**
 * @brief Enable the LIS2MDL magnetometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_Enable()
{
  /* Check if the component is already enabled */
  if (selected_sensor_private_data[0] == 1U)
  {
    return LIS2MDLSensor_OK;
  }

  /* Output data rate selection. */
  if (LIS2MDLSensor_operating_mode_set(LIS2MDLSensor_CONTINUOUS_MODE) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  selected_sensor_private_data[0] = 1;

  return LIS2MDLSensor_OK;
}

/**
 * @brief Disable the LIS2MDL magnetometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_Disable()
{
  /* Check if the component is already disabled */
  if (selected_sensor_private_data[0] == 0U)
  {
    return LIS2MDLSensor_OK;
  }

  /* Output data rate selection - power down. */
  if (LIS2MDLSensor_operating_mode_set(LIS2MDLSensor_POWER_DOWN) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  selected_sensor_private_data[0] = 0;

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_GetSensitivity(float *Sensitivity)
{
  *Sensitivity = LIS2MDLSensor_MAG_SENSITIVITY_FS_50GAUSS;

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_GetOutputDataRate(float *Odr)
{
  LIS2MDLStatusTypeDef ret = LIS2MDLSensor_OK;
  LIS2MDLSensor_odr_t odr_low_level;

  /* Get current output data rate. */
  if (LIS2MDLSensor_data_rate_get(&odr_low_level) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  switch (odr_low_level)
  {
    case LIS2MDLSensor_ODR_10Hz:
      *Odr = 10.0f;
      break;

    case LIS2MDLSensor_ODR_20Hz:
      *Odr = 20.0f;
      break;

    case LIS2MDLSensor_ODR_50Hz:
      *Odr = 50.0f;
      break;

    case LIS2MDLSensor_ODR_100Hz:
      *Odr = 100.0f;
      break;

    default:
      ret = LIS2MDLSensor_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LIS2MDL magnetometer sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_SetOutputDataRate(float Odr)
{
  LIS2MDLSensor_odr_t new_odr;

  new_odr = (Odr <= 10.000f) ? LIS2MDLSensor_ODR_10Hz
            : (Odr <= 20.000f) ? LIS2MDLSensor_ODR_20Hz
            : (Odr <= 50.000f) ? LIS2MDLSensor_ODR_50Hz
            :                    LIS2MDLSensor_ODR_100Hz;

  if (LIS2MDLSensor_data_rate_set(new_odr) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_GetFullScale(int32_t *FullScale)
{
  *FullScale = 50;

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Set the LIS2MDL magnetometer sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_SetFullScale(int32_t FullScale)
{
  (void)FullScale;
  return LIS2MDLSensor_OK;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor axes
 * @param  MagneticField pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_GetAxes(int32_t *MagneticField)
{
  axis3bit16_t data_raw;
  float sensitivity;

  /* Read raw data values. */
  if (LIS2MDLSensor_magnetic_raw_get(data_raw.u8bit) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  /* Get LIS2MDL actual sensitivity. */
  LIS2MDLSensor_GetSensitivity(&sensitivity);

  /* Calculate the data. */
  MagneticField[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  MagneticField[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  MagneticField[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Get the LIS2MDL magnetometer sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_GetAxesRaw(int16_t *Value)
{
  axis3bit16_t data_raw;

  /* Read raw data values. */
  if (LIS2MDLSensor_magnetic_raw_get(data_raw.u8bit) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Get the LIS2MDL register value for magnetic sensor
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_ReadReg(uint8_t Reg, uint8_t *Data)
{
  if (LIS2MDLSensor_read_reg(Reg, Data, 1) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Set the LIS2MDL register value for magnetic sensor
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_WriteReg(uint8_t Reg, uint8_t Data)
{
  if (LIS2MDLSensor_write_reg(Reg, &Data, 1) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Set self test
 * @param  val the value of self_test in reg CFG_REG_C
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_SetSelfTest(uint8_t val)
{
  if (LIS2MDLSensor_self_test_set(val) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  return LIS2MDLSensor_OK;
}

/**
 * @brief  Get the LIS2MDL MAG data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_GetDRDYStatus(uint8_t *Status)
{
  if (LIS2MDLSensor_mag_data_ready_get(Status) != LIS2MDLSensor_OK)
  {
    return LIS2MDLSensor_ERROR;
  }

  return LIS2MDLSensor_OK;
}



LIS2MDLStatusTypeDef LIS2MDLSensor_Calibration(int16_t mag_bias[3], int16_t mag_scale[3]) 
{
  uint16_t ii = 0, sample_count = 0, delay = 0;
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  float odr;
  if (LIS2MDLSensor_GetOutputDataRate(&odr) != LIS2MDLSensor_OK) {
    return LIS2MDLSensor_ERROR;
  }
    
  delay_ms(4000);
    
  // shoot for ~fifteen seconds of mag data
  sample_count = (int16_t) (odr * 15);
  delay = (int16_t) ((1000.0f / odr) * 1.2f);
    
  for(ii = 0; ii < sample_count; ii++) {
    LIS2MDLSensor_GetAxesRaw(mag_temp);  // Read the mag data   
    for (int axis = 0; axis < 3; axis++) {
      if(mag_temp[axis] > mag_max[axis]) mag_max[axis] = mag_temp[axis];
      if(mag_temp[axis] < mag_min[axis]) mag_min[axis] = mag_temp[axis];
    }
    delay_ms(delay);
  }

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
   
  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
  
  return LIS2MDLSensor_OK;
}
