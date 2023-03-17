/**
   @file RAK12033-IIM42652.cpp
   @author rakwireless.com
   @brief  IIM42652 configuration function realization.
   @version 0.1
   @date 2022-01-01
   @copyright Copyright (c) 2022
**/

#include "RAK12033-IIM42652.h"

/*!
 *  @brief  Initialize the class.
 *  @param  addr: The device address of IIM42652 IIC is 0x69.
 */
IIM42652::IIM42652(byte addr)
{
  _deviceAddress = addr;
}

/*!
 *  @brief  I2c bus write.
 *  @param  registerAddress  : Register address.
 *  @param  writeData        : Write data pointer.
 *  @param  size             : The length of the written data.
 */
void IIM42652::writeRegister(uint8_t registerAddress, uint8_t *writeData, uint8_t size)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(registerAddress);
  for (size_t i = 0; i < size; i++)
  {
    _i2cPort->write(writeData[i]);
  }
  _i2cPort->endTransmission();
}

/*!
 *  @brief  I2c bus read.
 *  @param  registerAddress : Register address.
 *  @param  readData        : Read data pointer.
 *  @param  size            : The length of the written data.
 */
void IIM42652::readRegister(uint8_t registerAddress, uint8_t *readData, uint8_t size)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(registerAddress);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, size);

  size_t i = 0;
  while (_i2cPort->available()) // slave may send less than requested
  {
    readData[i] = _i2cPort->read(); // receive a byte as a proper uint8_t
    i++;
  }
}

/*!
 *  @brief  Initalizes the IIM-42652 sensor.
 *  @param  wirePort      : IIC interface used.
 *  @param  deviceAddress : Device address should be 0x68.
 *  @return If the device init successful return true else return false.
 */
bool IIM42652::begin(TwoWire &wirePort, uint8_t deviceAddress)
{
  uint8_t i2cData = 0;
  uint8_t _sensor_id;

  _deviceAddress = deviceAddress;

  _i2cPort = &wirePort;

  LIB_LOG("begin", "RAK12033");
  delay(100);
  // readRegister(IIM42652_REG_WHO_AM_I , &_sensor_id , 1);
  get_device_id(&_sensor_id);
  LIB_LOG("begin", "ID = 0x%X", _sensor_id);

  if (_sensor_id == IIM42652_CHIP_ID)
  {
    soft_reset();
    return true;
  }
  else
    return false;
}

/*!
 *  @brief  Register bank selection.
 *  @param  bank_sel	: Selected bank. eg: IIM42652_SET_BANK_0.
 *  @return NULL.
 */
void IIM42652::bank_selection(uint8_t bank_sel)
{
  uint8_t tmp;

  writeRegister(IIM42652_REG_BANK_SEL, &bank_sel, 1);

  readRegister(IIM42652_REG_BANK_SEL, &tmp, 1);

  LIB_LOG("bank_selection", "bank_sel = %d ", tmp);

  delay(1);
}

/*!
 *  @brief  Get device ID.
 *  @param  device_id	: Device ID variable pointer.
 *  @return NULL.
 */
void IIM42652::get_device_id(uint8_t *device_id)
{
  readRegister(IIM42652_REG_WHO_AM_I, device_id, 1);
}

/*!
 *  @brief  Enable IIM42652 temperature measurement.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::temperature_enable(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);

  tmp &= ~IIM42652_SET_TEMPERATURE_DISABLED;

  writeRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

/*!
 *  @brief  Disable IIM42652 temperature measurement.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::temperature_disable(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);

  tmp |= IIM42652_SET_TEMPERATURE_DISABLED;

  writeRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

/*!
 *  @brief  IDLE mode. When Accel and Gyro are powered off.
            The chip will go to OFF state, since the RC oscillator will also be powered off.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::idle(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);

  tmp &= 0xEF;

  writeRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

/*!
 *  @brief  Exit IDLE mode. The RC oscillator is powered on even if Accel and Gyro are powered off.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::ex_idle(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);

  tmp |= ~0xEF;

  writeRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

/*!
 *  @brief  Enable IIM42652 gyroscope measurement.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::gyroscope_enable(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);

  tmp |= IIM42652_SET_GYRO_TLOW_NOISE_MODE;

  writeRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

/*!
 *  @brief  Disable IIM42652 gyroscope measurement.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::gyroscope_disable(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);

  tmp &= 0xF3;

  writeRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

/*!
 *  @brief  Enable IIM42652 acceleration measurement.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::accelerometer_enable(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);

  tmp |= IIM42652_SET_ACCEL_LOW_NOISE_MODE;

  writeRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

/*!
 *  @brief  Disable IIM42652 acceleration measurement.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::accelerometer_disable(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);

  tmp &= 0xFC;

  writeRegister(IIM42652_REG_PWR_MGMT0, &tmp, 1);
}

/*!
 *  @brief  Software reset IIM42652.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::soft_reset(void)
{
  uint8_t tmp;

  readRegister(IIM42652_REG_DEVICE_CONFIG, &tmp, 1);

  tmp |= 0x01;

  writeRegister(IIM42652_REG_DEVICE_CONFIG, &tmp, 1);

  delay(10);
}

/*!
 *  @brief  Get IIM42652 acceleration data.
 *  @param  accel_data	:Pointer to data of type @ IIM42652_axis_t.
 *  @return NULL.
 */
void IIM42652::get_accel_data(IIM42652_axis_t *accel_data)
{
  uint8_t rx_buf[6];
  uint16_t tmp;

  readRegister(IIM42652_REG_ACCEL_DATA_X1_UI, rx_buf, 6);

  tmp = rx_buf[0];
  tmp <<= 8;
  tmp |= rx_buf[1];

  accel_data->x = (int16_t)tmp;

  tmp = rx_buf[2];
  tmp <<= 8;
  tmp |= rx_buf[3];

  accel_data->y = (int16_t)tmp;

  tmp = rx_buf[4];
  tmp <<= 8;
  tmp |= rx_buf[5];

  accel_data->z = (int16_t)tmp;
}

/*!
 *  @brief  Get IIM42652 gyroscope data.
 *  @param  gyro_data	:Pointer to data of type @ IIM42652_axis_t.
 *  @return NULL.
 */
void IIM42652::get_gyro_data(IIM42652_axis_t *gyro_data)
{
  uint8_t rx_buf[6];
  uint16_t tmp;

  readRegister(IIM42652_REG_GYRO_DATA_X1_UI, rx_buf, 6);

  tmp = rx_buf[0];
  tmp <<= 8;
  tmp |= rx_buf[1];

  gyro_data->x = (int16_t)tmp;

  tmp = rx_buf[2];
  tmp <<= 8;
  tmp |= rx_buf[3];

  gyro_data->y = (int16_t)tmp;

  tmp = rx_buf[4];
  tmp <<= 8;
  tmp |= rx_buf[5];

  gyro_data->z = (int16_t)tmp;
}

/*!
 *  @brief  Get IIM42652 temperature data.
 *  @param  temperature	:Pointer to temperature variable.
 *  @return NULL.
 */
void IIM42652::get_temperature(float *temperature)
{
  uint8_t rx_buf[2];
  int16_t tmp;

  readRegister(IIM42652_REG_TEMP_DATA1_UI, rx_buf, 2);

  tmp = rx_buf[0];
  tmp <<= 8;
  tmp |= rx_buf[1];

  *temperature = (float)tmp;
  *temperature /= 132.48;
  *temperature += 25;
}

/*!
 *  @brief  Set IIM42652 ACC full scale range.
 *  @param  accel_fsr_g	:Full scal range value. Reference@ IIM42652_ACCEL_CONFIG0_FS_SEL_t.
 *  @return NULL.
 */
void IIM42652::set_accel_fsr(IIM42652_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g)
{
  uint8_t accel_cfg_0_reg;

  readRegister(IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);

  accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
  accel_cfg_0_reg |= (uint8_t)accel_fsr_g<< BIT_ACCEL_CONFIG0_FS_SEL_POS;

  writeRegister(IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
}

/*!
 *  @brief  Set IIM42652 ACC output data rate.
 *  @param  frequency	:Output data rate value. Reference@ IIM42652_ACCEL_CONFIG0_ODR_t.
 *  @return NULL.
 */
void IIM42652::set_accel_frequency(const IIM42652_ACCEL_CONFIG0_ODR_t frequency)
{
  uint8_t accel_cfg_0_reg;

  readRegister(IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);

  accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_ODR_MASK;
  accel_cfg_0_reg |= (uint8_t)frequency;

  writeRegister(IIM42652_REG_ACCEL_CONFIG0, &accel_cfg_0_reg, 1);
}

/*!
 *  @brief  Set IIM42652 GYRO full scale range.
 *  @param  gyro_fsr_dps	:Full scal range value. Reference@ IIM42652_GYRO_CONFIG0_FS_SEL_t.
 *  @return NULL.
 */
void IIM42652::set_gyro_fsr(IIM42652_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps)
{
  uint8_t gyro_cfg_0_reg;

  readRegister(IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);

  gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_FS_SEL_MASK;
  gyro_cfg_0_reg |= (uint8_t)gyro_fsr_dps;

  writeRegister(IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
}

/*!
 *  @brief  Set IIM42652 GYRO output data rate.
 *  @param  frequency	:Output data rate value. Reference@ IIM42652_GYRO_CONFIG0_ODR_t.
 *  @return NULL.
 */
void IIM42652::set_gyro_frequency(const IIM42652_GYRO_CONFIG0_ODR_t frequency)
{
  uint8_t gyro_cfg_0_reg;

  readRegister(IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);

  gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_ODR_MASK;
  gyro_cfg_0_reg |= (uint8_t)frequency;

  writeRegister(IIM42652_REG_GYRO_CONFIG0, &gyro_cfg_0_reg, 1);
}

/*!
 *  @brief  Get WOM interrupt flag, clears on read.
 *  @param  NULL.
 *  @return Return the value in register INT_STATUS2.
 *			For details, refer to the description of INT_STATUS2 in the date sheet.
 */
uint8_t IIM42652::get_WOM_INT(void)
{
  uint8_t data;

  readRegister(IIM42652_REG_INT_STATUS2, &data, 1);

  LIB_LOG("get_WOM_INT", "IIM42652_REG_INT_STATUS2 = %d ", data);

  return data;
}

/*!
 *  @brief  Wake on motion configuration.
 *  @param  x_th	:Threshold value for the Wake on Motion Interrupt for X-axis accelerometer
           WoM thresholds are expressed in fixed “mg” independent of the selected
           Range [0g : 1g]; Resolution 1g/256=~3.9mg
 *  @param  y_th	:Threshold value for the Wake on Motion Interrupt for Y-axis accelerometer
           WoM thresholds are expressed in fixed “mg” independent of the selected
           Range [0g : 1g]; Resolution 1g/256=~3.9mg
 *  @param  z_th	:Threshold value for the Wake on Motion Interrupt for Z-axis accelerometer
           WoM thresholds are expressed in fixed “mg” independent of the selected
           Range [0g : 1g]; Resolution 1g/256=~3.9mg
 *  @return NULL.
 */
void IIM42652::wake_on_motion_configuration(const uint8_t x_th, const uint8_t y_th, const uint8_t z_th)
{
  uint8_t data[3];

  readRegister(IIM42652_REG_INT_CONFIG, data, 1);
  data[0] |= 0x02; // eg: BIT_INT_SOURCE1_WOM_Z_INT1_EN
  LIB_LOG("wake_on_motion_configuration", "data[0] = 0x%X", data[0]);
  writeRegister(IIM42652_REG_INT_CONFIG, data, 1);

  // Set memory bank 4.
  bank_selection(IIM42652_SET_BANK_4);

  data[0] = x_th; /* Set X threshold */
  data[1] = y_th; /* Set Y threshold */
  data[2] = z_th; /* Set Z threshold */
  writeRegister(IIM42652_REG_ACCEL_WOM_X_THR, data, 3);

  readRegister(IIM42652_REG_ACCEL_WOM_X_THR, data, 1);

  LIB_LOG("wake_on_motion_configuration", "IIM42652_REG_ACCEL_WOM_X_THR = %d ", data[0]);
  LIB_LOG("wake_on_motion_configuration", "IIM42652_REG_ACCEL_WOM_X_THR = %d ", data[1]);
  LIB_LOG("wake_on_motion_configuration", "IIM42652_REG_ACCEL_WOM_X_THR = %d ", data[2]);

  delay(1);

  // Set memory bank 0.
  bank_selection(IIM42652_SET_BANK_0);

  readRegister(IIM42652_REG_INT_SOURCE1, data, 1);
  data[0] |= X_INT1_EN | Y_INT1_EN | Z_INT1_EN; // eg: BIT_INT_SOURCE1_WOM_Z_INT1_EN
  LIB_LOG("wake_on_motion_configuration", "data[0] = 0x%X", data[0]);
  writeRegister(IIM42652_REG_INT_SOURCE1, data, 1);

  delay(100); // Wait 100 milliseconds.

  // Turn on WOM feature by setting WOM_INT_MODE to 0, WOM_MODE to 1,
  // SMD_MODE to 1 (Register 0x56h in Bank 0)
  readRegister(IIM42652_REG_SMD_CONFIG, data, 1);
  data[0] |= 0x05;
  LIB_LOG("wake_on_motion_configuration", "data[0] = 0x%X", data[0]);
  writeRegister(IIM42652_REG_SMD_CONFIG, data, 1);
}

/*!
 *  @brief  Set ACC to low power mode.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::enable_accel_low_power_mode(void)
{
  uint8_t pwr_mgmt0_reg;

  // Enable/Switch the accelerometer in/to low power mode.
  readRegister(IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg &= ~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
  pwr_mgmt0_reg |= IIM42652_PWR_MGMT_0_ACCEL_MODE_LP;
  writeRegister(IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);

  // (Register 0x4Dh in Bank 0), ACCEL_LP_CLK_SEL = 0, for low power mode.
  readRegister(IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg &= ~BIT_ACCEL_LP_CLK_SEL_MASK;
  pwr_mgmt0_reg |= IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC;
  LIB_LOG("enable_accel_low_power_mode", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);

  delay(1);
}

/*!
 *  @brief  To do.
 *  @param  NULL.
 *  @return NULL.
 */
void IIM42652::pedometer_configuration()
{
  uint8_t pwr_mgmt0_reg;

  // Enable/Switch the accelerometer in/to low power mode.
  readRegister(IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg &= ~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
  pwr_mgmt0_reg |= IIM42652_PWR_MGMT_0_ACCEL_MODE_LP;
  writeRegister(IIM42652_REG_PWR_MGMT0, &pwr_mgmt0_reg, 1);

  // (Register 0x4Dh in Bank 0), ACCEL_LP_CLK_SEL = 0, for low power mode.
  readRegister(IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg &= ~BIT_ACCEL_LP_CLK_SEL_MASK;
  pwr_mgmt0_reg |= IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC;
  LIB_LOG("pedometer_configuration", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_INTF_CONFIG1, &pwr_mgmt0_reg, 1);

  // Set DMP ODR = 50 Hz and turn on Pedometer feature (Register 0x56h in Bank 0)
  readRegister(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
  //  pwr_mgmt0_reg &= ~BIT_ACCEL_LP_CLK_SEL_MASK;
  pwr_mgmt0_reg |= 0x22;
  LIB_LOG("pedometer_configuration", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
  delay(1);

  readRegister(IIM42652_REG_SIGNAL_PATH_RESET, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg |= 0x20;
  LIB_LOG("pedometer_configuration", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_SIGNAL_PATH_RESET, &pwr_mgmt0_reg, 1);
  delay(1);

  // Set LOW_ENERGY_AMP_TH_SEL to 10 (Register 0x40h in Bank 4)
  bank_selection(IIM42652_SET_BANK_4);
  readRegister(IIM42652_REG_APEX_CONFIG1, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg |= 0xA0;
  LIB_LOG("pedometer_configuration", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_APEX_CONFIG1, &pwr_mgmt0_reg, 1);

  // Set DMP_INIT_EN to 1 (Register 0x4Bh in Bank 0)
  bank_selection(IIM42652_SET_BANK_0);
  readRegister(IIM42652_REG_SIGNAL_PATH_RESET, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg |= 0x40;
  LIB_LOG("pedometer_configuration", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_SIGNAL_PATH_RESET, &pwr_mgmt0_reg, 1);

  delay(50);

  bank_selection(IIM42652_SET_BANK_4);
  readRegister(IIM42652_REG_INT_SOURCE6, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg |= 0x20;
  LIB_LOG("pedometer_configuration", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_INT_SOURCE6, &pwr_mgmt0_reg, 1);

  // Disable freefall by setting FF_ENABLE to 0 (Register 0x56h in Bank 0)
  bank_selection(IIM42652_SET_BANK_0);
  readRegister(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg &= 0xFB;
  LIB_LOG("pedometer_configuration", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);

  readRegister(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
  pwr_mgmt0_reg |= 0x20;
  LIB_LOG("pedometer_configuration", "pwr_mgmt0_reg = 0x%X", pwr_mgmt0_reg);
  writeRegister(IIM42652_REG_APEX_CONFIG0, &pwr_mgmt0_reg, 1);
}

/*!
 *  @brief  To do.
 *  @param  NULL.
 *  @return NULL.
 */
uint16_t IIM42652::get_pedometer_data(void)
{
  uint8_t date[2];
  uint16_t tmp;

  readRegister(IIM42652_REG_APEX_DATA0, date, 2);

  tmp = date[1];
  tmp <<= 8;
  tmp |= date[0];

  return tmp;
}
