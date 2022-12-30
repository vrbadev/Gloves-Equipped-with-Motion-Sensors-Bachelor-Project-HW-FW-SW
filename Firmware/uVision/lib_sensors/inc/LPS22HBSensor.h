/**
 ******************************************************************************
 * @file    LPS22HBSensor.h
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Abstract Class of a LPS22HB Pressure sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

#ifndef __LPS22HBSensor_H__
#define __LPS22HBSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "LPS22HB_Driver.h"

/* Typedefs ------------------------------------------------------------------*/
typedef enum
{
  LPS22HB_STATUS_OK = 0,
  LPS22HB_STATUS_ERROR,
  LPS22HB_STATUS_TIMEOUT,
  LPS22HB_STATUS_NOT_IMPLEMENTED
} LPS22HBStatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LPS22HB Pressure sensor.
 */
LPS22HBStatusTypeDef LPS22HBSensor_Init           (void);
LPS22HBStatusTypeDef LPS22HBSensor_Enable         (void);
LPS22HBStatusTypeDef LPS22HBSensor_Disable        (void);
LPS22HBStatusTypeDef LPS22HBSensor_ReadID         (uint8_t *ht_id);
LPS22HBStatusTypeDef LPS22HBSensor_Reset          (void);
LPS22HBStatusTypeDef LPS22HBSensor_GetPressure    (float *pfData);
LPS22HBStatusTypeDef LPS22HBSensor_GetTemperature (float *pfData);
LPS22HBStatusTypeDef LPS22HBSensor_GetODR         (float *odr);
LPS22HBStatusTypeDef LPS22HBSensor_SetODR         (float odr);
LPS22HBStatusTypeDef LPS22HBSensor_ReadReg        (uint8_t reg, uint8_t *data);
LPS22HBStatusTypeDef LPS22HBSensor_WriteReg       (uint8_t reg, uint8_t data);


#endif
