/* Includes-----------------------------------------------------------------------------------*/
#include "IIS2DLPCSensor.h"


/* Class Implementation-----------------------------------------------------------------------*/

/** Constructor I2C
 *  @param i2c object
 *  @param address the address of the component's instance
 */
//IIS2DLPCSensor::IIS2DLPCSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
IIS2DLPCSensor::IIS2DLPCSensor(cyhal_i2c_t * pi2c_dev, uint8_t address) : m_pi2c_dev(pi2c_dev),m_address(address)

{
  reg_ctx.write_reg = IIS2DLPC_io_write;
  reg_ctx.read_reg = IIS2DLPC_io_read;
  reg_ctx.handle = (void *)this;
  acc_is_enabled = 0U;

}


/**
 *  @brief Initialize the sensor
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Init()
{
  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface */
  if (iis2dlpc_auto_increment_set(&(reg_ctx), PROPERTY_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable Block Data Update */
  if (iis2dlpc_block_data_update_set(&(reg_ctx), PROPERTY_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* FIFO mode selection */
  if (iis2dlpc_fifo_mode_set(&(reg_ctx), IIS2DLPC_BYPASS_MODE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }
  /* Power mode selection */
  if (iis2dlpc_power_mode_set(&(reg_ctx), IIS2DLPC_HIGH_PERFORMANCE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Select default output data rate. */
  acc_odr = 100.0f;
  /* Select default operating mode. */
  acc_operating_mode = IIS2DLPC_HIGH_PERFORMANCE_MODE;
  /* Select default low noise (disabled). */
  acc_low_noise = IIS2DLPC_LOW_NOISE_DISABLE;

  /* Output data rate selection - power down */
  if (iis2dlpc_data_rate_set(&(reg_ctx), IIS2DLPC_XL_ODR_OFF) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Full scale selection */
  if (iis2dlpc_full_scale_set(&(reg_ctx), IIS2DLPC_2g) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Self Test disabled  */
  if (iis2dlpc_self_test_set(&(reg_ctx), IIS2DLPC_XL_ST_DISABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  acc_is_enabled = 0U;

  return IIS2DLPC_OK;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::begin()
{

  if (Init() != IIS2DLPC_OK)
  {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::end()
{
  /* Disable acc */
  if (Disable() != IIS2DLPC_OK)
  {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/**
 * @brief Read component ID
 * @param Id pointer to store the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::ReadID(uint8_t *Id)
{
  if (iis2dlpc_device_id_get(&(reg_ctx), Id) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/**
 * @brief Enabled the IIS2DLPC accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Enable()
{
  /*Check if the component is already enabled */
  if (acc_is_enabled == 1U) {
    return IIS2DLPC_OK;
  }

  /*Function that set Ouput Data Rate and Power Mode*/
  if (SetOutputDataRate_When_Enable(acc_odr, acc_operating_mode, acc_low_noise) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  acc_is_enabled = 1U;

  return IIS2DLPC_OK;
}

/**
 * @brief Disable the IIS2DLPC accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Disable()
{
  /*Check if the component is already disabled*/
  if (acc_is_enabled == 0U) {
    return IIS2DLPC_OK;
  }

  /* Output data rate selection - power down*/
  if (iis2dlpc_data_rate_set(&(reg_ctx), IIS2DLPC_XL_ODR_OFF) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  acc_is_enabled = 0U;

  return IIS2DLPC_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensitivity
 * @param Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::GetSensitivity(float *Sensitivity)
{
  iis2dlpc_fs_t full_scale;
  iis2dlpc_mode_t mode;

  /*Read full scale*/
  if (iis2dlpc_full_scale_get(&(reg_ctx), &full_scale) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Read power mode selection*/
  if (iis2dlpc_power_mode_get(&(reg_ctx), &mode) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Get Sensitivity*/
  switch (mode) {
    case IIS2DLPC_CONT_LOW_PWR_12bit:
    case IIS2DLPC_SINGLE_LOW_PWR_12bit:
    case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit:
    case IIS2DLPC_SINGLE_LOW_LOW_NOISE_PWR_12bit:
      switch (full_scale) {
        case IIS2DLPC_2g:
          *Sensitivity = IIS2DLPC_ACC_SENSITIVITY_FOR_FS_2G_LOPOW1_MODE;
          break;

        case IIS2DLPC_4g:
          *Sensitivity = IIS2DLPC_ACC_SENSITIVITY_FOR_FS_4G_LOPOW1_MODE;
          break;

        case IIS2DLPC_8g:
          *Sensitivity = IIS2DLPC_ACC_SENSITIVITY_FOR_FS_8G_LOPOW1_MODE;
          break;

        case IIS2DLPC_16g:
          *Sensitivity = IIS2DLPC_ACC_SENSITIVITY_FOR_FS_16G_LOPOW1_MODE;
          break;

        default:
          *Sensitivity = -1.0f;
          return IIS2DLPC_ERROR;
          break;
      }
      break;

    case IIS2DLPC_HIGH_PERFORMANCE:
    case IIS2DLPC_CONT_LOW_PWR_4:
    case IIS2DLPC_CONT_LOW_PWR_3:
    case IIS2DLPC_CONT_LOW_PWR_2:
    case IIS2DLPC_SINGLE_LOW_PWR_4:
    case IIS2DLPC_SINGLE_LOW_PWR_3:
    case IIS2DLPC_SINGLE_LOW_PWR_2:
    case IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE:
    case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_4:
    case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_3:
    case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_2:
    case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_4:
    case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_3:
    case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_2:
      switch (full_scale) {
        case IIS2DLPC_2g:
          *Sensitivity = IIS2DLPC_ACC_SENSITIVITY_FOR_FS_2G_OTHER_MODES;
          break;

        case IIS2DLPC_4g:
          *Sensitivity = IIS2DLPC_ACC_SENSITIVITY_FOR_FS_4G_OTHER_MODES;
          break;

        case IIS2DLPC_8g:
          *Sensitivity = IIS2DLPC_ACC_SENSITIVITY_FOR_FS_8G_OTHER_MODES;
          break;

        case IIS2DLPC_16g:
          *Sensitivity = IIS2DLPC_ACC_SENSITIVITY_FOR_FS_16G_OTHER_MODES;
          break;

        default:
          *Sensitivity = -1.0f;
          return IIS2DLPC_ERROR;
          break;
      }
      break;

    default:
      *Sensitivity = -1.0f;
      return IIS2DLPC_ERROR;
      break;
  }

  return IIS2DLPC_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor Output Data Rate
 * @param Odr pointer
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::GetOutputDataRate(float *Odr)
{
  IIS2DLPCStatusTypeDef ret = IIS2DLPC_OK;
  iis2dlpc_odr_t odr_low_level;
  iis2dlpc_mode_t mode;

  /*Get current ODR*/
  if (iis2dlpc_data_rate_get(&(reg_ctx), &odr_low_level) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Get power mode selection from sensor */
  if (iis2dlpc_power_mode_get(&(reg_ctx), &mode) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  switch (odr_low_level) {
    case IIS2DLPC_XL_ODR_OFF:
    case IIS2DLPC_XL_SET_SW_TRIG:
    case IIS2DLPC_XL_SET_PIN_TRIG:
      *Odr = 0.0f;
      break;

    case IIS2DLPC_XL_ODR_1Hz6_LP_ONLY:
      switch (mode) {
        case IIS2DLPC_HIGH_PERFORMANCE:
        case IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE:
          *Odr = 12.5f;
          break;

        case IIS2DLPC_CONT_LOW_PWR_4:
        case IIS2DLPC_CONT_LOW_PWR_3:
        case IIS2DLPC_CONT_LOW_PWR_2:
        case IIS2DLPC_CONT_LOW_PWR_12bit:
        case IIS2DLPC_SINGLE_LOW_PWR_4:
        case IIS2DLPC_SINGLE_LOW_PWR_3:
        case IIS2DLPC_SINGLE_LOW_PWR_2:
        case IIS2DLPC_SINGLE_LOW_PWR_12bit:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_4:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_3:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_2:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_4:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_3:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_2:
        case IIS2DLPC_SINGLE_LOW_LOW_NOISE_PWR_12bit:
          *Odr = 1.6f;
          break;

        default:
          *Odr = -1.0f;
          ret = IIS2DLPC_ERROR;
          break;
      }
      break;

    case IIS2DLPC_XL_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case IIS2DLPC_XL_ODR_25Hz:
      *Odr = 25.0f;
      break;

    case IIS2DLPC_XL_ODR_50Hz:
      *Odr = 50.0f;
      break;

    case IIS2DLPC_XL_ODR_100Hz:
      *Odr = 100.0f;
      break;

    case IIS2DLPC_XL_ODR_200Hz:
      *Odr = 200.0f;
      break;

    case IIS2DLPC_XL_ODR_400Hz:
      switch (mode) {
        case IIS2DLPC_HIGH_PERFORMANCE:
        case IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE:
          *Odr = 400.0f;
          break;

        case IIS2DLPC_CONT_LOW_PWR_4:
        case IIS2DLPC_CONT_LOW_PWR_3:
        case IIS2DLPC_CONT_LOW_PWR_2:
        case IIS2DLPC_CONT_LOW_PWR_12bit:
        case IIS2DLPC_SINGLE_LOW_PWR_4:
        case IIS2DLPC_SINGLE_LOW_PWR_3:
        case IIS2DLPC_SINGLE_LOW_PWR_2:
        case IIS2DLPC_SINGLE_LOW_PWR_12bit:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_4:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_3:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_2:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_4:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_3:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_2:
        case IIS2DLPC_SINGLE_LOW_LOW_NOISE_PWR_12bit:
          *Odr = 200.0f;
          break;

        default:
          *Odr = -1.0f;
          ret = IIS2DLPC_ERROR;
          break;
      }
      break;

    case IIS2DLPC_XL_ODR_800Hz:
      switch (mode) {
        case IIS2DLPC_HIGH_PERFORMANCE:
        case IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE:
          *Odr = 800.0f;
          break;

        case IIS2DLPC_CONT_LOW_PWR_4:
        case IIS2DLPC_CONT_LOW_PWR_3:
        case IIS2DLPC_CONT_LOW_PWR_2:
        case IIS2DLPC_CONT_LOW_PWR_12bit:
        case IIS2DLPC_SINGLE_LOW_PWR_4:
        case IIS2DLPC_SINGLE_LOW_PWR_3:
        case IIS2DLPC_SINGLE_LOW_PWR_2:
        case IIS2DLPC_SINGLE_LOW_PWR_12bit:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_4:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_3:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_2:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_4:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_3:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_2:
        case IIS2DLPC_SINGLE_LOW_LOW_NOISE_PWR_12bit:
          *Odr = 200.0f;
          break;

        default:
          *Odr = -1.0f;
          ret = IIS2DLPC_ERROR;
          break;
      }
      break;

    case IIS2DLPC_XL_ODR_1k6Hz:
      switch (mode) {
        case IIS2DLPC_HIGH_PERFORMANCE:
        case IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE:
          *Odr = 1600.0f;
          break;

        case IIS2DLPC_CONT_LOW_PWR_4:
        case IIS2DLPC_CONT_LOW_PWR_3:
        case IIS2DLPC_CONT_LOW_PWR_2:
        case IIS2DLPC_CONT_LOW_PWR_12bit:
        case IIS2DLPC_SINGLE_LOW_PWR_4:
        case IIS2DLPC_SINGLE_LOW_PWR_3:
        case IIS2DLPC_SINGLE_LOW_PWR_2:
        case IIS2DLPC_SINGLE_LOW_PWR_12bit:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_4:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_3:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_2:
        case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_4:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_3:
        case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_2:
        case IIS2DLPC_SINGLE_LOW_LOW_NOISE_PWR_12bit:
          *Odr = 200.0f;
          break;

        default:
          *Odr = -1.0f;
          ret = IIS2DLPC_ERROR;
          break;
      }
      break;

    default:
      *Odr = -1.0f;
      ret = IIS2DLPC_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Set the IIS2DLPC accelerometer Output Data Rate
 * @param Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetOutputDataRate(float odr)
{
  return SetOutputDataRate_With_Mode(odr, IIS2DLPC_HIGH_PERFORMANCE_MODE, IIS2DLPC_LOW_NOISE_DISABLE);
}

/**
 * @brief Set the IIS2DLPC accelerometer output data rate with Operating Mode
 * @param odr the output data rate value to be set
 * @param Mode operating mode
 * @param Noise if low_noise is enabled or disabled
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetOutputDataRate_With_Mode(float odr, IIS2DLPC_Operating_Mode_t Mode, IIS2DLPC_Low_Noise_t Noise)
{
  if (acc_is_enabled == 1U) {
    return SetOutputDataRate_When_Enable(odr, Mode, Noise);
  } else {
    return SetOutputDataRate_When_Disable(odr, Mode, Noise);
  }
  return IIS2DLPC_OK;
}

/**
 * @brief Set the IIS2DLPC Ouput Data Rate When the component is enabled
 * @param Odr the output data rate value to be set
 * @param Mode the current operating mode
 * @param Noise if low_noise is enabled or disabled
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetOutputDataRate_When_Enable(float Odr, IIS2DLPC_Operating_Mode_t Mode, IIS2DLPC_Low_Noise_t Noise)
{
  iis2dlpc_odr_t new_odr;
  iis2dlpc_mode_t new_power_mode;

  switch (Mode) {
    case IIS2DLPC_HIGH_PERFORMANCE_MODE:
    default:
      switch (Noise) {
        case IIS2DLPC_LOW_NOISE_DISABLE:
        default:
          new_power_mode = IIS2DLPC_HIGH_PERFORMANCE;
          break;
        case IIS2DLPC_LOW_NOISE_ENABLE:
          new_power_mode = IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE;
          break;
      }
      if (Odr < 12.5f) {
        Odr = 12.5f;
      }
      break;

    case IIS2DLPC_LOW_POWER_MODE4:
      switch (Noise) {
        case IIS2DLPC_LOW_NOISE_DISABLE:
        default:
          new_power_mode = IIS2DLPC_CONT_LOW_PWR_4;
          break;
        case IIS2DLPC_LOW_NOISE_ENABLE:
          new_power_mode = IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_4;
          break;
      }

      if (Odr > 200.0f) {
        Odr = 200.0f;
      }
      break;
    case IIS2DLPC_LOW_POWER_MODE3:
      switch (Noise) {
        case IIS2DLPC_LOW_NOISE_DISABLE:
        default:
          new_power_mode = IIS2DLPC_CONT_LOW_PWR_3;
          break;
        case IIS2DLPC_LOW_NOISE_ENABLE:
          new_power_mode = IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_3;
          break;
      }

      if (Odr > 200.0f) {
        Odr = 200.0f;
      }
      break;
    case IIS2DLPC_LOW_POWER_MODE2:
      switch (Noise) {
        case IIS2DLPC_LOW_NOISE_DISABLE:
        default:
          new_power_mode = IIS2DLPC_CONT_LOW_PWR_2;
          break;
        case IIS2DLPC_LOW_NOISE_ENABLE:
          new_power_mode = IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_2;
          break;
      }

      if (Odr > 200.0f) {
        Odr = 200.0f;
      }
      break;
    case IIS2DLPC_LOW_POWER_MODE1:
      switch (Noise) {
        case IIS2DLPC_LOW_NOISE_DISABLE:
        default:
          new_power_mode = IIS2DLPC_CONT_LOW_PWR_12bit;
          break;
        case IIS2DLPC_LOW_NOISE_ENABLE:
          new_power_mode = IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit;
          break;
      }

      if (Odr > 200.0f) {
        Odr = 200.0f;
      }
      break;
  }

  new_odr = (Odr <= 1.6f) ? IIS2DLPC_XL_ODR_1Hz6_LP_ONLY
            : (Odr <=   12.5f) ? IIS2DLPC_XL_ODR_12Hz5
            : (Odr <=   25.0f) ? IIS2DLPC_XL_ODR_25Hz
            : (Odr <=   50.0f) ? IIS2DLPC_XL_ODR_50Hz
            : (Odr <=  100.0f) ? IIS2DLPC_XL_ODR_100Hz
            : (Odr <=  200.0f) ? IIS2DLPC_XL_ODR_200Hz
            : (Odr <=  400.0f) ? IIS2DLPC_XL_ODR_400Hz
            : (Odr <=  800.0f) ? IIS2DLPC_XL_ODR_800Hz
            :                    IIS2DLPC_XL_ODR_1k6Hz;

  /*Output data rate setting */
  if (iis2dlpc_data_rate_set(&(reg_ctx), new_odr) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Power Mode Setting */
  if (iis2dlpc_power_mode_set(&(reg_ctx), new_power_mode) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Store the current Odr, Mode and Noise values */
  acc_odr = Odr;
  acc_operating_mode = Mode;
  acc_low_noise = Noise;

  return IIS2DLPC_OK;
}

/*
 * @brief Set the IIS2DLPC Output Data Rate when component is disabled
 * @param Odr the output data rate value to be set
 * @param Mode the current operating mode
 * @param Noise if low_noise is enabled or disabled
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetOutputDataRate_When_Disable(float Odr, IIS2DLPC_Operating_Mode_t Mode, IIS2DLPC_Low_Noise_t Noise)
{
  acc_operating_mode = Mode;
  acc_low_noise = Noise;

  acc_odr = (Odr <=    1.6f) ? IIS2DLPC_XL_ODR_1Hz6_LP_ONLY
            : (Odr <=   12.5f) ? IIS2DLPC_XL_ODR_12Hz5
            : (Odr <=   25.0f) ? IIS2DLPC_XL_ODR_25Hz
            : (Odr <=   50.0f) ? IIS2DLPC_XL_ODR_50Hz
            : (Odr <=  100.0f) ? IIS2DLPC_XL_ODR_100Hz
            : (Odr <=  200.0f) ? IIS2DLPC_XL_ODR_200Hz
            : (Odr <=  400.0f) ? IIS2DLPC_XL_ODR_400Hz
            : (Odr <=  800.0f) ? IIS2DLPC_XL_ODR_800Hz
            :                   IIS2DLPC_XL_ODR_1k6Hz;

  return IIS2DLPC_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor full scale
 * @param Fullscale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::GetFullScale(int32_t *Fullscale)
{
  iis2dlpc_fs_t fs_low_level;

  /*Read FS*/
  if (iis2dlpc_full_scale_get(&(reg_ctx), &fs_low_level) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  switch (fs_low_level) {
    case IIS2DLPC_2g:
      *Fullscale = 2;
      break;

    case IIS2DLPC_4g:
      *Fullscale = 4;
      break;

    case IIS2DLPC_8g:
      *Fullscale = 8;
      break;

    case IIS2DLPC_16g:
      *Fullscale = 16;
      break;

    default:
      *Fullscale = -1;
      return IIS2DLPC_ERROR;
      break;
  }

  return IIS2DLPC_OK;
}

/**
 * @brief Set the IIS2DLPC accelerometer sensor full scale
 * @param fullscale the fullscale to be set
 * @retval  0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetFullScale(int32_t fullscale)
{
  iis2dlpc_fs_t new_fs;

  new_fs = (fullscale <= 2) ? IIS2DLPC_2g
           : (fullscale <= 4) ? IIS2DLPC_4g
           : (fullscale <= 8) ? IIS2DLPC_8g
           :                   IIS2DLPC_16g;

  if (iis2dlpc_full_scale_set(&(reg_ctx), new_fs) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor raw axes
 * @param value pointer where the raw values are written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::GetAxesRaw(int16_t *value)
{
  iis2dlpc_mode_t mode;
  axis3bit16_t data_raw;

  /*Get power mode*/
  if (iis2dlpc_power_mode_get(&reg_ctx, &mode) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Read raw data values*/
  if (iis2dlpc_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  switch (mode) {
    case IIS2DLPC_CONT_LOW_PWR_12bit:
    case IIS2DLPC_SINGLE_LOW_PWR_12bit:
    case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit:
    case IIS2DLPC_SINGLE_LOW_LOW_NOISE_PWR_12bit:
      /* Data format 12 bits. */
      value[0] = (data_raw.i16bit[0] / 16);
      value[1] = (data_raw.i16bit[1] / 16);
      value[2] = (data_raw.i16bit[2] / 16);
      break;

    case IIS2DLPC_HIGH_PERFORMANCE:
    case IIS2DLPC_CONT_LOW_PWR_4:
    case IIS2DLPC_CONT_LOW_PWR_3:
    case IIS2DLPC_CONT_LOW_PWR_2:
    case IIS2DLPC_SINGLE_LOW_PWR_4:
    case IIS2DLPC_SINGLE_LOW_PWR_3:
    case IIS2DLPC_SINGLE_LOW_PWR_2:
    case IIS2DLPC_HIGH_PERFORMANCE_LOW_NOISE:
    case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_4:
    case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_3:
    case IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_2:
    case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_4:
    case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_3:
    case IIS2DLPC_SINGLE_LOW_PWR_LOW_NOISE_2:
      /*Data format 14 bits. */
      value[0] = (data_raw.i16bit[0] / 4);
      value[1] = (data_raw.i16bit[1] / 4);
      value[2] = (data_raw.i16bit[2] / 4);
      break;

    default:
      return IIS2DLPC_ERROR;
      break;
  }
  return IIS2DLPC_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor axes
 * @param acceleration pointer where the axes are written
 * @retval 0 in case of success, an error code otherwise
*/
IIS2DLPCStatusTypeDef IIS2DLPCSensor::GetAxes(int32_t *acceleration)
{
  int16_t data_raw[3];
  float sensitivity = 0.0f;

  /*Read raw data values */
  if (GetAxesRaw(data_raw) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Get sensitivity */
  if (GetSensitivity(&sensitivity) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Calculate */
  acceleration[0] = (int32_t)((float) data_raw[0] * sensitivity);
  acceleration[1] = (int32_t)((float) data_raw[1] * sensitivity);
  acceleration[2] = (int32_t)((float) data_raw[2] * sensitivity);

  return IIS2DLPC_OK;
}

/**
 * @brief Get the IIS2DLPC accelerometer sensor data ready bit value
 * @param Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::GetDRDYStatus(uint8_t *status)
{
  if (iis2dlpc_flag_data_ready_get(&reg_ctx, status) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/**
 * @brief Set Self Test
 * @param val the value of self_test in reg
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetSelfTest(uint8_t val)
{
  iis2dlpc_st_t value;

  switch (val) {
    case 0:
    default:
      value = IIS2DLPC_XL_ST_DISABLE;
      break;

    case 1:
      value = IIS2DLPC_XL_ST_POSITIVE;
      break;

    case 2:
      value = IIS2DLPC_XL_ST_NEGATIVE;
      break;
  }
  if (iis2dlpc_self_test_set(&reg_ctx, value) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Get the Self Test value
 *  @param val pointer where the value of self_test is written
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::GetSelfTest(iis2dlpc_st_t *val)
{
  if (iis2dlpc_self_test_get(&reg_ctx, val) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Get the Status of the possible event
 *  @param Status pointer to a structur Event_Status_t
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::GetEventStatus(IIS2DLPC_Event_Status_t *Status)
{
  iis2dlpc_wake_up_src_t  wake_up_src;
  iis2dlpc_tap_src_t      tap_src;
  iis2dlpc_sixd_src_t     sixd_src;
  iis2dlpc_ctrl4_int1_pad_ctrl_t int1_ctrl;

  (void)memset((void *)Status, 0x0, sizeof(IIS2DLPC_Event_Status_t));

  if (iis2dlpc_read_reg(&reg_ctx, IIS2DLPC_WAKE_UP_SRC, (uint8_t *) &wake_up_src, 1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_read_reg(&reg_ctx, IIS2DLPC_TAP_SRC, (uint8_t *) &tap_src, 1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_read_reg(&reg_ctx, IIS2DLPC_SIXD_SRC, (uint8_t *) &sixd_src, 1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_read_reg(&reg_ctx, IIS2DLPC_CTRL4_INT1_PAD_CTRL, (uint8_t *) &int1_ctrl, 1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (int1_ctrl.int1_ff == 1U) {
    if (wake_up_src.ff_ia == 1U) {
      Status->FreeFallStatus = 1;
    }
  }

  if (int1_ctrl.int1_wu == 1U) {
    if (wake_up_src.wu_ia == 1U) {
      Status->WakeUpStatus = 1;
    }
  }

  if (int1_ctrl.int1_single_tap == 1U) {
    if (tap_src.single_tap == 1U) {
      Status->TapStatus = 1;
    }
  }

  if (int1_ctrl.int1_tap == 1U) {
    if (tap_src.double_tap == 1U) {
      Status->DoubleTapStatus = 1;
    }
  }

  if (int1_ctrl.int1_6d == 1U) {
    if (sixd_src._6d_ia == 1U) {
      Status->D6DOrientationStatus = 1;
    }
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Set the interrupt latch
 *  @param Status value to be written
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetInterruptLatch(uint8_t Status)
{
  if (Status > 1U) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_int_notification_set(&reg_ctx, (iis2dlpc_lir_t) Status) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }
  return IIS2DLPC_OK;
}

/*
 *  @brief Enable the detection of the Free Fall event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::EnableFreeFallDetection()
{
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  IIS2DLPCStatusTypeDef ret = IIS2DLPC_OK;

  /*Output Data Rate Selection */
  if (SetOutputDataRate(200.0f) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Full Scale selection*/
  if (SetFullScale(2) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*FF_DUR setting*/
  if (iis2dlpc_ff_dur_set(&reg_ctx, 0x06) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*WAKE_DUR setting */
  if (iis2dlpc_wkup_dur_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* FF_THS setting */
  if (iis2dlpc_ff_threshold_set(&reg_ctx, IIS2DLPC_FF_TSH_10LSb_FS2g) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable free fall event on INT1 */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }
  val1.int1_ff = PROPERTY_ENABLE;
  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the Free Fall event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::DisableFreeFallDetection()
{
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  /*Disable on INT1 and INT2 */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_ff = PROPERTY_DISABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*FF_DUR Setting*/
  if (iis2dlpc_ff_dur_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* FF_THS setting */
  if (iis2dlpc_ff_threshold_set(&reg_ctx, IIS2DLPC_FF_TSH_5LSb_FS2g) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Set the Threshold for the Free Fall event
 *  @param Threshold integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetFreeFallThreshold(uint8_t Threshold)
{
  iis2dlpc_ff_ths_t new_ths;

  new_ths = (Threshold <= 0) ? IIS2DLPC_FF_TSH_5LSb_FS2g
            : (Threshold = 1) ? IIS2DLPC_FF_TSH_7LSb_FS2g
            : (Threshold = 2) ? IIS2DLPC_FF_TSH_8LSb_FS2g
            : (Threshold = 3) ? IIS2DLPC_FF_TSH_10LSb_FS2g
            : (Threshold = 4) ? IIS2DLPC_FF_TSH_11LSb_FS2g
            : (Threshold = 5) ? IIS2DLPC_FF_TSH_13LSb_FS2g
            : (Threshold = 6) ? IIS2DLPC_FF_TSH_15LSb_FS2g
            :                   IIS2DLPC_FF_TSH_16LSb_FS2g;

  if (iis2dlpc_ff_threshold_set(&reg_ctx, new_ths) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }
  return IIS2DLPC_OK;
}

/*
 *  @brief Set the Duration for the Free Fall event
 *  @param Duration integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetFreeFallDuration(uint8_t Duration)
{
  if (iis2dlpc_ff_dur_set(&reg_ctx, Duration) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Enable the detection of the Wake Up event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::EnableWakeUpDetection()
{
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  IIS2DLPCStatusTypeDef ret = IIS2DLPC_OK;

  /*Output Data Rate selection */
  if (SetOutputDataRate(200.0f) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Full scale selection */
  if (SetFullScale(2) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* HP filter */

  /*Set wake up duration */
  if (iis2dlpc_wkup_dur_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Set wake up threshold */
  if (iis2dlpc_wkup_threshold_set(&reg_ctx, 0x02) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable wake up event on INT1 pin */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_wu = PROPERTY_ENABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the Wake Up event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::DisableWakeUpDetection()
{

  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  /* Disable wake up event on INT1 and INT2 pins */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_wu = PROPERTY_DISABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Reset wake up threshold */
  if (iis2dlpc_wkup_threshold_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* WAKE_DUR setting */
  if (iis2dlpc_wkup_dur_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Set the Threshold for the Wake Up event
 *  @param Threshold integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetWakeUpThreshold(uint8_t Threshold)
{
  /* Set wake up threshold */
  if (iis2dlpc_wkup_threshold_set(&reg_ctx, Threshold) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Set the Duration for the Wake Up event
 *  @param Duration integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetWakeUpDuration(uint8_t Duration)
{
  /* Set wake up duration */
  if (iis2dlpc_wkup_dur_set(&reg_ctx, Duration) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Enable the detection of the Single Tap event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::EnableSingleTapDetection()
{
  IIS2DLPCStatusTypeDef ret = IIS2DLPC_OK;
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  /*Output Data Rate selection */
  if (SetOutputDataRate_With_Mode(400, IIS2DLPC_HIGH_PERFORMANCE_MODE, IIS2DLPC_LOW_NOISE_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Full Scale selection */
  if (SetFullScale(2) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }


  /* Set tap threshold */
  if (iis2dlpc_tap_threshold_x_set(&reg_ctx, 0x09) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_threshold_y_set(&reg_ctx, 0x09) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_threshold_z_set(&reg_ctx, 0x09) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Set tap priority Z-Y-X */
  if (iis2dlpc_tap_axis_priority_set(&reg_ctx, IIS2DLPC_ZYX) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable X direction in tap recognition */
  if (iis2dlpc_tap_detection_on_x_set(&reg_ctx, PROPERTY_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable Y direction in tap recognition */
  if (iis2dlpc_tap_detection_on_y_set(&reg_ctx, PROPERTY_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable Z direction in tap recognition */
  if (iis2dlpc_tap_detection_on_z_set(&reg_ctx, PROPERTY_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Set quiet and shock time */
  if (iis2dlpc_tap_quiet_set(&reg_ctx, 0x01) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_shock_set(&reg_ctx, 0x02) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Only Single-Tap enabled */
  if (iis2dlpc_tap_mode_set(&reg_ctx, IIS2DLPC_ONLY_SINGLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable free fall event on INT1 pin */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_single_tap = PROPERTY_ENABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the Single Tap event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::DisableSingleTapDetection()
{
  IIS2DLPCStatusTypeDef ret = IIS2DLPC_OK;
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  /*Disable single tap event on INT1 */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_single_tap = PROPERTY_DISABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Reset tap quiet time window */
  if (iis2dlpc_tap_quiet_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Reset tap shock time window */
  if (iis2dlpc_tap_shock_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Set tap priority X-Y-Z */
  if (iis2dlpc_tap_axis_priority_set(&reg_ctx, IIS2DLPC_XYZ) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Disable direction in tap recognition */
  if (iis2dlpc_tap_detection_on_z_set(&reg_ctx, PROPERTY_DISABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_detection_on_y_set(&reg_ctx, PROPERTY_DISABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_detection_on_x_set(&reg_ctx, PROPERTY_DISABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }
  
  /* Reset tap threshold */
  if (iis2dlpc_tap_threshold_x_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_threshold_y_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_threshold_z_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return ret;
}

/*
 *  @brief Enable the detection of the Double Tap event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::EnableDoubleTapDetection()
{

  IIS2DLPCStatusTypeDef ret = IIS2DLPC_OK;
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  /*Output Data Rate selection */
  if (SetOutputDataRate_With_Mode(400, IIS2DLPC_HIGH_PERFORMANCE_MODE, IIS2DLPC_LOW_NOISE_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /*Full Scale selection */
  if (SetFullScale(2) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable X direction in tap recognition */
  if (iis2dlpc_tap_detection_on_x_set(&reg_ctx, PROPERTY_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable Y direction in tap recognition */
  if (iis2dlpc_tap_detection_on_y_set(&reg_ctx, PROPERTY_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable Z direction in tap recognition */
  if (iis2dlpc_tap_detection_on_z_set(&reg_ctx, PROPERTY_ENABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Set tap threshold */
  if (iis2dlpc_tap_threshold_x_set(&reg_ctx, 0x0C) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_threshold_y_set(&reg_ctx, 0x0C) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_threshold_z_set(&reg_ctx, 0x0C) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Set TAP priority Z-Y-X */
  if (iis2dlpc_tap_axis_priority_set(&reg_ctx, IIS2DLPC_ZYX) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Set quiet and shock time */
  if (iis2dlpc_tap_quiet_set(&reg_ctx, 0x03) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_shock_set(&reg_ctx, 0x03) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_dur_set(&reg_ctx, 0x07) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Both Single Double enabled */
  if (iis2dlpc_tap_mode_set(&reg_ctx, IIS2DLPC_BOTH_SINGLE_DOUBLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable free fall event on INT1*/
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_tap = PROPERTY_ENABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the Double Tap event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::DisableDoubleTapDetection()
{
  IIS2DLPCStatusTypeDef ret = IIS2DLPC_OK;
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  /*Disable single tap event on INT1. */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_single_tap = PROPERTY_DISABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Reset tap quiet time window */
  if (iis2dlpc_tap_quiet_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Reset tap shock time window */
  if (iis2dlpc_tap_shock_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Reset tap duration */
  if (iis2dlpc_tap_dur_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Set TAP priority X-Y-Z */
  if (iis2dlpc_tap_axis_priority_set(&reg_ctx, IIS2DLPC_XYZ) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Disable direction in tap recognition */
  if (iis2dlpc_tap_detection_on_z_set(&reg_ctx, PROPERTY_DISABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_detection_on_y_set(&reg_ctx, PROPERTY_DISABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_detection_on_x_set(&reg_ctx, PROPERTY_DISABLE) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Reset tap threshold */
  if (iis2dlpc_tap_threshold_x_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_threshold_y_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  if (iis2dlpc_tap_threshold_z_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return ret;
}

/*
 *  @brief Set the Threshold for the Tap event
 *  @param Threshold integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetTapThreshold(uint8_t Threshold)
{
  if (iis2dlpc_tap_threshold_x_set(&reg_ctx, Threshold) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Set the Shock Time for the Tap event
 *  @param Time integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetTapShockTime(uint8_t Time)
{
  if (iis2dlpc_tap_shock_set(&reg_ctx, Time) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Set the Quiet Time for the Tap event
 *  @param Time integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetTapQuietTime(uint8_t Time)
{
  if (iis2dlpc_tap_quiet_set(&reg_ctx, Time) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Set the Duration Time for the Tap event
 *  @param Time integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::SetTapDurationTime(uint8_t Time)
{
  if (iis2dlpc_tap_dur_set(&reg_ctx, Time) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Enable the detection of the 6DOrientation event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Enable6DOrientation()
{
  IIS2DLPCStatusTypeDef ret = IIS2DLPC_OK;
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  /* Output Data Rate selection */
  if (SetOutputDataRate(200.0f) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Full scale selection */
  if (SetFullScale(2) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* 6D orientation enabled */
  if (iis2dlpc_6d_threshold_set(&reg_ctx, 0x02) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Enable 6D orientation event on INT1 */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_6d = PROPERTY_ENABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return ret;
}

/*
 *  @brief Disable the detection of the 6DOrientation event
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Disable6DOrientation()
{
  iis2dlpc_ctrl4_int1_pad_ctrl_t val1;

  /* Disable 6D orientation event on INT1. */
  if (iis2dlpc_pin_int1_route_get(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  val1.int1_6d = PROPERTY_DISABLE;

  if (iis2dlpc_pin_int1_route_set(&reg_ctx, &val1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  /* Reset 6D orientation */
  if (iis2dlpc_6d_threshold_set(&reg_ctx, 0x00) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/*
 *  @brief Set the Threshold for the 6DOrientation event
 *  @param Threshold integer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Set6DOrientationThreshold(uint8_t Threshold)
{
  if (iis2dlpc_6d_threshold_set(&reg_ctx, Threshold) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }
  return IIS2DLPC_OK;
}

/*
 *  @brief Get the XL Axes for the 6DOrientation event
 *  @param XLow pointer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Get6DOrientationXL(uint8_t *XLow)
{
  iis2dlpc_sixd_src_t data;

  if (iis2dlpc_6d_src_get(&reg_ctx, &data) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  *XLow = data.xl;

  return IIS2DLPC_OK;
}

/*
 *  @brief Get the XH Axes for the 6DOrientation event
 *  @param XHigh pointer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Get6DOrientationXH(uint8_t *XHigh)
{
  iis2dlpc_sixd_src_t data;

  if (iis2dlpc_6d_src_get(&reg_ctx, &data) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  *XHigh = data.xh;

  return IIS2DLPC_OK;
}

/*
 *  @brief Get the YL Axes for the 6DOrientation event
 *  @param YLow pointer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Get6DOrientationYL(uint8_t *YLow)
{
  iis2dlpc_sixd_src_t data;

  if (iis2dlpc_6d_src_get(&reg_ctx, &data) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  *YLow = data.yl;

  return IIS2DLPC_OK;
}

/*
 *  @brief Get the YH Axes for the 6DOrientation event
 *  @param YHigh pointer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Get6DOrientationYH(uint8_t *YHigh)
{
  iis2dlpc_sixd_src_t data;

  if (iis2dlpc_6d_src_get(&reg_ctx, &data) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  *YHigh = data.yh;

  return IIS2DLPC_OK;
}

/*
 *  @brief Get the ZL Axes for the 6DOrientation event
 *  @param ZLow pointer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Get6DOrientationZL(uint8_t *ZLow)
{
  iis2dlpc_sixd_src_t data;

  if (iis2dlpc_6d_src_get(&reg_ctx, &data) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  *ZLow = data.zl;

  return IIS2DLPC_OK;
}

/*
 *  @brief Get the ZH Axes for the 6DOrientation event
 *  @param ZHigh pointer
 *  @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::Get6DOrientationZH(uint8_t *ZHigh)
{
  iis2dlpc_sixd_src_t data;

  if (iis2dlpc_6d_src_get(&reg_ctx, &data) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  *ZHigh = data.zh;

  return IIS2DLPC_OK;
}

/**
 * @brief  Get the IIS2MDC register value for magnetic sensor
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::ReadReg(uint8_t reg, uint8_t *data)
{
  if (iis2dlpc_read_reg(&reg_ctx, reg, data, 1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

/**
 * @brief  Set the IIS2MDC register value for magnetic sensor
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
IIS2DLPCStatusTypeDef IIS2DLPCSensor::WriteReg(uint8_t reg, uint8_t data)
{
  if (iis2dlpc_write_reg(&reg_ctx, reg, &data, 1) != IIS2DLPC_OK) {
    return IIS2DLPC_ERROR;
  }

  return IIS2DLPC_OK;
}

int32_t IIS2DLPC_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((IIS2DLPCSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t IIS2DLPC_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((IIS2DLPCSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
