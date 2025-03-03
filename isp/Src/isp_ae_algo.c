/**
 ******************************************************************************
 * @file    isp_ae_algo.c
 * @author  AIS Application Team
 * @brief   ISP AWB algorithm
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "isp_core.h"
#include "isp_services.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#define AE_FINE_TOLERANCE                 5
#define AE_FINE_TOLERANCE_LOW_LUX(target) ((target) < 15 ? 8 : 10)
#define AE_COARSE_TOLERANCE                 10
#define AE_COARSE_TOLERANCE_LOW_LUX(target) ((target) < 15 ? 10 : 15)
#define AE_TOLERANCE                        0.10  /* % */
#define AE_TOLERANCE_LOW_LUX                0.15  /* % */

#define AE_LOW_LUX_LIMIT                    50    /* lux */

#define AE_EXPOSURE_COARSE_INCREMENT        500   /* ms */
#define AE_EXPOSURE_COARSE_DECREMENT        300   /* ms */
#define AE_EXPOSURE_FINE_INCREMENT          150   /* ms */
#define AE_EXPOSURE_FINE_DECREMENT          100   /* ms */

#define AE_GAIN_COARSE_INCREMENT            2000  /* mdB */
#define AE_GAIN_COARSE_DECREMENT            1500  /* mdB */
#define AE_GAIN_FINE_INCREMENT              500   /* mdB */
#define AE_GAIN_FINE_DECREMENT              300   /* mdB */

#define AE_MAX_GAIN_INCREMENT               10000 /* mdB */
/* Private macro -------------------------------------------------------------*/
#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static ISP_IQParamTypeDef *IQParamConfig;
static ISP_SensorInfoTypeDef *pSensorInfo;
static uint32_t analog_gain_max;
static uint32_t previous_lux = 0;

/* Global variables ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void isp_ae_init(ISP_HandleTypeDef *hIsp)
{
  IQParamConfig = ISP_SVC_IQParam_Get(hIsp);
  pSensorInfo = &hIsp->sensorInfo;

  /* TODO: change sensor info to get max analog gain from sensor driver */
  /* Set max analog gain */
  if (strcmp(hIsp->sensorInfo.name, "IMX335") == 0)
  {
    analog_gain_max = 36000;
  }
  else if (strcmp(hIsp->sensorInfo.name, "VD66GY") == 0)
  {
    analog_gain_max = 18000;
  }
  else if (strcmp(hIsp->sensorInfo.name, "VD5941") == 0)
  {
    analog_gain_max = 12000;
  }
  else
  {
    analog_gain_max = 0;
  }
}

void get_new_exposure(uint32_t lux, uint32_t averageL, uint32_t *pExposure, uint32_t *pGain, uint32_t curExposure, uint32_t curGain)
{
  double a, b, c, d;
  double cur_global_exposure = curExposure * pow(10, (double)curGain / 20000);
  double new_global_exposure;
  uint32_t custom_low_lux_limit = ((uint32_t)(((double)IQParamConfig->AECAlgo.exposureTarget * IQParamConfig->luxRef.calibFactor * (IQParamConfig->luxRef.LL_LuxRef *
          ((double)IQParamConfig->luxRef.LL_Expo1 / IQParamConfig->luxRef.LL_Lum1 -
           (double)IQParamConfig->luxRef.LL_Expo2 / IQParamConfig->luxRef.LL_Lum2) /
          ((double)IQParamConfig->luxRef.LL_Expo1 - IQParamConfig->luxRef.LL_Expo2))) / AE_LOW_LUX_LIMIT) + 1) * AE_LOW_LUX_LIMIT;

  /* Handle start conditions */
  if ((averageL <= 5) && curExposure == pSensorInfo->exposure_min && curGain == pSensorInfo->gain_min)
  {
    new_global_exposure = curGain ? curExposure * pow(10, ((double)curGain + 3000) / 20000) : curExposure + 2000;
    if (new_global_exposure <= pSensorInfo->exposure_max)
    {
      *pGain = 0;
      *pExposure = (new_global_exposure < pSensorInfo->exposure_min) ? pSensorInfo->exposure_min : new_global_exposure;
    }
    else
    {
      *pExposure = pSensorInfo->exposure_max;
      *pGain = (uint32_t)(20 * 1000 * log10((float)new_global_exposure / (float)(*pExposure)));
      *pGain = (*pGain < pSensorInfo->gain_min) ? pSensorInfo->gain_min : (*pGain > analog_gain_max) ? analog_gain_max : *pGain;
    }
    return;
  }

  /* Check if coarse convergence is reached */
  if (((lux > AE_LOW_LUX_LIMIT) && (abs(averageL - IQParamConfig->AECAlgo.exposureTarget) > MAX((float)IQParamConfig->AECAlgo.exposureTarget * AE_TOLERANCE, AE_COARSE_TOLERANCE))) ||
      ((lux <= AE_LOW_LUX_LIMIT) && (abs(averageL - IQParamConfig->AECAlgo.exposureTarget) > MAX((float)IQParamConfig->AECAlgo.exposureTarget * AE_TOLERANCE_LOW_LUX, AE_COARSE_TOLERANCE_LOW_LUX(IQParamConfig->AECAlgo.exposureTarget)))))
  {
    if (lux <= IQParamConfig->luxRef.HL_LuxRef)
    {
      /* Calculate a and b with the low lux references to improve precision when lux is under HL_LuxRef */
      a = (IQParamConfig->luxRef.LL_LuxRef *
          ((double)IQParamConfig->luxRef.LL_Expo1 / IQParamConfig->luxRef.LL_Lum1 -
           (double)IQParamConfig->luxRef.LL_Expo2 / IQParamConfig->luxRef.LL_Lum2)) /
          ((double)IQParamConfig->luxRef.LL_Expo1 - IQParamConfig->luxRef.LL_Expo2);

      b = (IQParamConfig->luxRef.LL_LuxRef * (double)IQParamConfig->luxRef.LL_Expo1 / IQParamConfig->luxRef.LL_Lum1) -
          (a * IQParamConfig->luxRef.LL_Expo1);
    }
    else
    {
      /* Calculate a and b with the high lux references for higher lux conditions*/
      a = (IQParamConfig->luxRef.HL_LuxRef *
          ((double)IQParamConfig->luxRef.HL_Expo1 / IQParamConfig->luxRef.HL_Lum1 -
           (double)IQParamConfig->luxRef.HL_Expo2 / IQParamConfig->luxRef.HL_Lum2)) /
          ((double)IQParamConfig->luxRef.HL_Expo1 - IQParamConfig->luxRef.HL_Expo2);

      b = (IQParamConfig->luxRef.HL_LuxRef * (double)IQParamConfig->luxRef.HL_Expo1 / IQParamConfig->luxRef.HL_Lum1) -
          (a * IQParamConfig->luxRef.HL_Expo1);
    }

    if (lux <= custom_low_lux_limit)
    {
      /* Calculate coefficient for very low lux model as we reach the limit of the previous one */
      d = pSensorInfo->exposure_max * pow(10, (double)analog_gain_max / 20000);
      c = ((b / (((double)custom_low_lux_limit / (IQParamConfig->AECAlgo.exposureTarget * IQParamConfig->luxRef.calibFactor)) - (double)a)) - d) / custom_low_lux_limit;

      new_global_exposure = (c * (double)lux) + d;
    }
    else
    {
      /* Else apply previous estimation model with a and b coefficients */
      new_global_exposure = b / (((double)lux / (IQParamConfig->AECAlgo.exposureTarget * IQParamConfig->luxRef.calibFactor)) - (double)a);
    }

    /* Check validity of new global exposure */
    if (averageL < IQParamConfig->AECAlgo.exposureTarget)
    {
      /* Exposure should be increased */
      if (new_global_exposure <= cur_global_exposure)
      {
        /* Wrong estimation, to be corrected */
        new_global_exposure = -1;

        if ((abs(lux - previous_lux) <= (float)lux * 0.05) && (lux != 0))
        {
          /* Same lux estimation but previous setting did not allow convergence */
          new_global_exposure = b * (((double)IQParamConfig->AECAlgo.exposureTarget - averageL) / lux + cur_global_exposure / (a * cur_global_exposure + b)) / (1 - a * (((double)IQParamConfig->AECAlgo.exposureTarget - averageL) / lux + cur_global_exposure / (a * cur_global_exposure + b)));
        }

        if (new_global_exposure <= 0)
        {
          /* Increase exposure with small increment to get closer to target */
          if (cur_global_exposure <= pSensorInfo->exposure_max)
          {
            new_global_exposure = averageL ? cur_global_exposure * (double)IQParamConfig->AECAlgo.exposureTarget / averageL : cur_global_exposure + AE_EXPOSURE_COARSE_INCREMENT;
          }
          else
          {
            new_global_exposure = pSensorInfo->exposure_max * pow(10, ((double)curGain + AE_GAIN_COARSE_INCREMENT) / 20000);
          }
        }
      }

      if (cur_global_exposure != 0 && averageL != 0)
      {
        /* Compare exposure ratio and luminance ratio, to check the consistency of the results  */
        if ((((new_global_exposure / cur_global_exposure) < 1.10) && (((double)IQParamConfig->AECAlgo.exposureTarget / averageL) > 1.40)) || /* new global exposure is very close to previous exposure while luminance is at least 40% higher */
            (((new_global_exposure / cur_global_exposure) < 1.50) && (((double)IQParamConfig->AECAlgo.exposureTarget / averageL) > 1.80)) || /* new global exposure is less than 50% higher while luminance is more than 80% higher */
            (((new_global_exposure / cur_global_exposure) > 1.65) && (((double)IQParamConfig->AECAlgo.exposureTarget / averageL) < 1.30)) || /* new global exposure is very high while luminance is less than 30% higher */
            (((new_global_exposure / cur_global_exposure) > 1.35) && (((double)IQParamConfig->AECAlgo.exposureTarget / averageL) < 1.10)))   /* new global exposure is too high when luminance is very close to the target */
        {
          /* Apply the same ratio */
          new_global_exposure = cur_global_exposure * ((double)IQParamConfig->AECAlgo.exposureTarget / averageL);
        }
      }
    }
    else
    {
      /* Exposure should be decreased */
      if (new_global_exposure >= cur_global_exposure)
      {
        /* Wrong estimation, to be corrected */
        new_global_exposure = -1;

        if ((abs(lux - previous_lux) <= (float)lux * 0.05) && lux != 0)
        {
          /* Same lux estimation but previous setting did not allow convergence */
          new_global_exposure = b * (((double)averageL - IQParamConfig->AECAlgo.exposureTarget) / lux + cur_global_exposure / (a * cur_global_exposure + b)) / (-1 - a * (((double)averageL - IQParamConfig->AECAlgo.exposureTarget) / lux + cur_global_exposure / (a * cur_global_exposure + b)));
        }

        if (new_global_exposure <= 0) //small step to get closer to target
        {
          /* Decrease exposure with small decrement to get closer to target */
          if (cur_global_exposure <= pSensorInfo->exposure_max)
          {
            new_global_exposure = averageL ? cur_global_exposure * (double)IQParamConfig->AECAlgo.exposureTarget / averageL : cur_global_exposure - AE_EXPOSURE_COARSE_DECREMENT;
          }
          else
          {
            new_global_exposure = pSensorInfo->exposure_max * pow(10, ((double)curGain - AE_GAIN_COARSE_DECREMENT) / 20000);
          }
        }
      }

      if (cur_global_exposure != 0 && averageL != 0)
      {
          /* Compare exposure ratio and luminance ratio, to check the consistency of the results  */
        if ((((new_global_exposure / cur_global_exposure) > 0.60) && (((double)IQParamConfig->AECAlgo.exposureTarget / averageL) < 0.40)) || /* new global exposure is at least 60% of the current exposure while luminance is less than 40% of the target */
            (((new_global_exposure / cur_global_exposure) < 0.45) && (((double)IQParamConfig->AECAlgo.exposureTarget / averageL) > 0.65)) || /* new global exposure is less than 45% of the current exposure while luminance is more than 65% of the target */
            (((new_global_exposure / cur_global_exposure) < 0.15) && (((double)IQParamConfig->AECAlgo.exposureTarget / averageL) > 0.50)) || /* new global exposure is very low while luminance is more than 50% of the target */
            (((new_global_exposure / cur_global_exposure) < 0.60) && (((double)IQParamConfig->AECAlgo.exposureTarget / averageL) > 0.85)))   /* new global exposure is less than 60% of the current exposure while luminance is very close to the target */
        {
          /* Apply the same ratio */
          new_global_exposure = cur_global_exposure * ((double)IQParamConfig->AECAlgo.exposureTarget / averageL);
        }
      }
    }

    /* Clamp and split exposure into exposure value and gain */
    if (new_global_exposure <= pSensorInfo->exposure_max)
    {
      *pGain = 0;
      *pExposure = (new_global_exposure < pSensorInfo->exposure_min) ? pSensorInfo->exposure_min : new_global_exposure;
    }
    else
    {
      *pExposure = pSensorInfo->exposure_max;
      *pGain = (uint32_t)(20 * 1000 * log10(new_global_exposure / (float)(*pExposure)));

      /* Limit digital gain (lux value is very low and the gain is already very high and we need to avoid oscillations) */
      if ((*pGain > analog_gain_max) && (abs(*pGain - curGain) > AE_MAX_GAIN_INCREMENT))
      {
        *pGain = *pGain < curGain ? (curGain < AE_MAX_GAIN_INCREMENT ? 0 : curGain - AE_MAX_GAIN_INCREMENT) : curGain + AE_MAX_GAIN_INCREMENT;
      }
      *pGain = (*pGain < pSensorInfo->gain_min) ? pSensorInfo->gain_min : (*pGain > pSensorInfo->gain_max) ? pSensorInfo->gain_max : *pGain;
    }
  }
  else
  {
    /* Convergence is reached */

    /* Coarse convergence reached, refine convergence */
    if (((lux > AE_LOW_LUX_LIMIT) && (abs(averageL - IQParamConfig->AECAlgo.exposureTarget) > AE_FINE_TOLERANCE)) ||
        ((lux <= AE_LOW_LUX_LIMIT) && (abs(averageL - IQParamConfig->AECAlgo.exposureTarget) > AE_FINE_TOLERANCE_LOW_LUX(IQParamConfig->AECAlgo.exposureTarget))))
    {
      if (averageL < IQParamConfig->AECAlgo.exposureTarget)
      {
        /* Sligthly increase exposure */
        if (cur_global_exposure <= pSensorInfo->exposure_max)
        {
          new_global_exposure = averageL ? cur_global_exposure * (double)IQParamConfig->AECAlgo.exposureTarget / averageL : cur_global_exposure + AE_EXPOSURE_FINE_INCREMENT;
        }
        else
        {
          new_global_exposure = pSensorInfo->exposure_max * pow(10, ((double)curGain + AE_GAIN_FINE_INCREMENT) / 20000);
        }
      }
      else
      {
        /* Slightly decrease exposure */
        if (cur_global_exposure <= pSensorInfo->exposure_max)
        {
          new_global_exposure = averageL ? cur_global_exposure * (double)IQParamConfig->AECAlgo.exposureTarget / averageL : cur_global_exposure - AE_EXPOSURE_FINE_DECREMENT;
        }
        else
        {
          new_global_exposure = pSensorInfo->exposure_max * pow(10, ((double)curGain - AE_GAIN_FINE_DECREMENT) / 20000);
        }
      }

      /* Clamp and split exposure into exposure value and gain */
      if (new_global_exposure <= pSensorInfo->exposure_max)
      {
        *pGain = 0;
        *pExposure = (new_global_exposure < pSensorInfo->exposure_min) ? pSensorInfo->exposure_min : new_global_exposure;
      }
      else
      {
        *pExposure = pSensorInfo->exposure_max;
        *pGain = (uint32_t)(20 * 1000 * log10((float)new_global_exposure / (float)(*pExposure)));

        *pGain = (*pGain < pSensorInfo->gain_min) ? pSensorInfo->gain_min : (*pGain > pSensorInfo->gain_max) ? pSensorInfo->gain_max : *pGain;
      }
    }
    else
    {
      /* Converged */
      *pExposure = curExposure;
      *pGain = curGain;
    }
  }

  previous_lux = lux;
}
