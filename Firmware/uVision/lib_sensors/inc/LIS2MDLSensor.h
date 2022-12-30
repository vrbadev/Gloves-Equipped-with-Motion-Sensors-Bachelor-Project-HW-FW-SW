/**
 ******************************************************************************
 * @file    LIS2MDLSensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Abstract Class of an LIS2MDL 3 axes magnetometer sensor.
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LIS2MDLSensor_H__
#define __LIS2MDLSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "../inc/lis2mdl_reg.h"

/* Defines -------------------------------------------------------------------*/


#define LIS2MDLSensor_MAG_SENSITIVITY_FS_50GAUSS  1.500f  /**< Sensitivity value for 50 gauss full scale [mgauss/LSB] */

/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  LIS2MDLSensor_OK = 0,
  LIS2MDLSensor_ERROR =-1
} LIS2MDLStatusTypeDef;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LIS2MDLSensor_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LIS2MDLSensor_Axes_t;


/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of an LIS2MDL Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
LIS2MDLStatusTypeDef LIS2MDLSensor_Init               (void);
LIS2MDLStatusTypeDef LIS2MDLSensor_ReadID             (uint8_t *Id);
LIS2MDLStatusTypeDef LIS2MDLSensor_Enable             (void);
LIS2MDLStatusTypeDef LIS2MDLSensor_Disable            (void);
LIS2MDLStatusTypeDef LIS2MDLSensor_GetSensitivity     (float *sensitivity);
LIS2MDLStatusTypeDef LIS2MDLSensor_GetOutputDataRate  (float *odr);
LIS2MDLStatusTypeDef LIS2MDLSensor_SetOutputDataRate  (float odr);
LIS2MDLStatusTypeDef LIS2MDLSensor_GetFullScale       (int32_t *fullscale);
LIS2MDLStatusTypeDef LIS2MDLSensor_SetFullScale       (int32_t fullscale);
LIS2MDLStatusTypeDef LIS2MDLSensor_GetAxes            (int32_t *magnetic_field);
LIS2MDLStatusTypeDef LIS2MDLSensor_GetAxesRaw         (int16_t *value);
LIS2MDLStatusTypeDef LIS2MDLSensor_ReadReg            (uint8_t reg, uint8_t *data);
LIS2MDLStatusTypeDef LIS2MDLSensor_WriteReg           (uint8_t reg, uint8_t data);
LIS2MDLStatusTypeDef LIS2MDLSensor_SetSelfTest        (uint8_t val);
LIS2MDLStatusTypeDef LIS2MDLSensor_GetDRDYStatus      (uint8_t *status);
LIS2MDLStatusTypeDef LIS2MDLSensor_Calibration        (int16_t mag_bias[3], int16_t mag_scale[3]);

#endif
