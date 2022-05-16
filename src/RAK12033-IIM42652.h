/**
   @file RAK12033-IIM42652.h
   @author rakwireless.com
   @brief  TDK 6-axis digital output sense IC library.
   @version 0.1
   @date 2022-01-01
   @copyright Copyright (c) 2022
**/

#ifndef __RAK12033 - IIM42652_h__
#define __RAK12033 -IIM42652_h__

#include <Arduino.h>
#include <Wire.h>
#include "stdint.h"

#define LIB_DEBUG 0
#if LIB_DEBUG > 0
#define LIB_LOG(tag, ...)        \
  do                             \
  {                              \
    if (tag)                     \
      Serial.printf("#Debug ");  \
    Serial.printf("<%s> ", tag); \
    Serial.printf(__VA_ARGS__);  \
    Serial.printf("\n");         \
  } while (0)
#else
#define LIB_LOG(...)
#endif

/**
 * @brief 6DOF IMU 17 device address setting.
 * @details Specified setting for device slave address selection of 6DOF IMU driver.
 */
#define IIM42652_SET_DEV_ADDR 0x69

/**
 * @brief   6DOF IMU description user bank 0 register.
 * @details Specified user bank 0 register for description of 6DOF IMU driver.
 */
#define IIM42652_REG_DEVICE_CONFIG 0x11
#define IIM42652_REG_DRIVE_CONFIG 0x13
#define IIM42652_REG_INT_CONFIG 0x14
#define IIM42652_REG_FIFO_CONFIG 0x16
#define IIM42652_REG_TEMP_DATA1_UI 0x1D
#define IIM42652_REG_TEMP_DATA0_UI 0x1E
#define IIM42652_REG_ACCEL_DATA_X1_UI 0x1F
#define IIM42652_REG_ACCEL_DATA_X0_UI 0x20
#define IIM42652_REG_ACCEL_DATA_Y1_UI 0x21
#define IIM42652_REG_ACCEL_DATA_Y0_UI 0x22
#define IIM42652_REG_ACCEL_DATA_Z1_UI 0x23
#define IIM42652_REG_ACCEL_DATA_Z0_UI 0x24
#define IIM42652_REG_GYRO_DATA_X1_UI 0x25
#define IIM42652_REG_GYRO_DATA_X0_UI 0x26
#define IIM42652_REG_GYRO_DATA_Y1_UI 0x27
#define IIM42652_REG_GYRO_DATA_Y0_UI 0x28
#define IIM42652_REG_GYRO_DATA_Z1_UI 0x29
#define IIM42652_REG_GYRO_DATA_Z0_UI 0x2A
#define IIM42652_REG_TMST_FSYNCH 0x2B
#define IIM42652_REG_TMST_FSYNCL 0x2C
#define IIM42652_REG_INT_STATUS 0x2D
#define IIM42652_REG_FIFO_COUNTH 0x2E
#define IIM42652_REG_FIFO_COUNTL 0x2F
#define IIM42652_REG_FIFO_DATA 0x30
#define IIM42652_REG_APEX_DATA0 0x31
#define IIM42652_REG_APEX_DATA1 0x32
#define IIM42652_REG_APEX_DATA2 0x33
#define IIM42652_REG_APEX_DATA3 0x34
#define IIM42652_REG_APEX_DATA4 0x35
#define IIM42652_REG_APEX_DATA5 0x36
#define IIM42652_REG_INT_STATUS2 0x37
#define IIM42652_REG_INT_STATUS3 0x38
#define IIM42652_REG_SIGNAL_PATH_RESET 0x4B
#define IIM42652_REG_INTF_CONFIG0 0x4C
#define IIM42652_REG_INTF_CONFIG1 0x4D
#define IIM42652_REG_PWR_MGMT0 0x4E
#define IIM42652_REG_GYRO_CONFIG0 0x4F
#define IIM42652_REG_ACCEL_CONFIG0 0x50
#define IIM42652_REG_GYRO_CONFIG1 0x51
#define IIM42652_REG_GYRO_ACCEL_CONFIG0 0x52
#define IIM42652_REG_ACCEL_CONFIG1 0x53
#define IIM42652_REG_TMST_CONFIG 0x54
#define IIM42652_REG_APEX_CONFIG0 0x56
#define IIM42652_REG_SMD_CONFIG 0x57
#define IIM42652_REG_FIFO_CONFIG1 0x5F
#define IIM42652_REG_FIFO_CONFIG2 0x60
#define IIM42652_REG_FIFO_CONFIG3 0x61
#define IIM42652_REG_FSYNC_CONFIG 0x62
#define IIM42652_REG_INT_CONFIG0 0x63
#define IIM42652_REG_INT_CONFIG1 0x64
#define IIM42652_REG_INT_SOURCE0 0x65
#define IIM42652_REG_INT_SOURCE1 0x66
#define IIM42652_REG_INT_SOURCE3 0x68
#define IIM42652_REG_INT_SOURCE4 0x69
#define IIM42652_REG_FIFO_LOST_PKT0 0x6C
#define IIM42652_REG_FIFO_LOST_PKT1 0x6D
#define IIM42652_REG_SELF_TEST_CONFIG 0x70
#define IIM42652_REG_WHO_AM_I 0x75
#define IIM42652_REG_BANK_SEL 0x76
#define IIM42652_CHIP_ID 0x6F

/**
 * @brief   6DOF IMU description user bank 1 register.
 * @details Specified user bank 1 registerfor description of 6DOF IMU driver.
 */
#define IIM42652_REG_SENSOR_CONFIG0 0x03
#define IIM42652_REG_GYRO_CONFIG_STATIC2 0x0B
#define IIM42652_REG_GYRO_CONFIG_STATIC3 0x0C
#define IIM42652_REG_GYRO_CONFIG_STATIC4 0x0D
#define IIM42652_REG_GYRO_CONFIG_STATIC5 0x0E
#define IIM42652_REG_GYRO_CONFIG_STATIC6 0x0F
#define IIM42652_REG_GYRO_CONFIG_STATIC7 0x10
#define IIM42652_REG_GYRO_CONFIG_STATIC8 0x11
#define IIM42652_REG_GYRO_CONFIG_STATIC9 0x12
#define IIM42652_REG_GYRO_CONFIG_STATIC10 0x13
#define IIM42652_REG_XG_ST_DATA 0x5F
#define IIM42652_REG_YG_ST_DATA 0x60
#define IIM42652_REG_ZG_ST_DATA 0x61
#define IIM42652_REG_TMSTVAL0 0x62
#define IIM42652_REG_TMSTVAL1 0x63
#define IIM42652_REG_TMSTVAL2 0x64
#define IIM42652_REG_INTF_CONFIG4 0x7A
#define IIM42652_REG_INTF_CONFIG5 0x7B
#define IIM42652_REG_INTF_CONFIG6 0x7C

/**
 * @brief 6DOF IMU description user bank 2 register.
 * @details Specified user bank 2 register for description of 6DOF IMU driver.
 */
#define IIM42652_REG_ACCEL_CONFIG_STATIC2 0x03
#define IIM42652_REG_ACCEL_CONFIG_STATIC3 0x04
#define IIM42652_REG_ACCEL_CONFIG_STATIC4 0x05
#define IIM42652_REG_XA_ST_DATA 0x3B
#define IIM42652_REG_YA_ST_DATA 0x3C
#define IIM42652_REG_ZA_ST_DATA 0x3D

/**
 * @brief 6DOF IMU description user bank 3 register.
 * @details Specified user bank 3 register for description of 6DOF IMU driver.
 */
#define IIM42652_REG_PU_PD_CONFIG1 0x06
#define IIM42652_REG_PU_PD_CONFIG2 0x0E

/**
 * @brief 6DOF IMU description user bank 4 register.
 * @details Specified user bank 4 register for description of 6DOF IMU driver.
 */
#define IIM42652_REG_FDR_CONFIG 0x09
#define IIM42652_REG_APEX_CONFIG1 0x40
#define IIM42652_REG_APEX_CONFIG2 0x41
#define IIM42652_REG_APEX_CONFIG3 0x42
#define IIM42652_REG_APEX_CONFIG4 0x43
#define IIM42652_REG_APEX_CONFIG5 0x44
#define IIM42652_REG_APEX_CONFIG6 0x45
#define IIM42652_REG_APEX_CONFIG7 0x46
#define IIM42652_REG_APEX_CONFIG8 0x47
#define IIM42652_REG_APEX_CONFIG9 0x48
#define IIM42652_REG_APEX_CONFIG10 0x49
#define IIM42652_REG_ACCEL_WOM_X_THR 0x4A
#define IIM42652_REG_ACCEL_WOM_Y_THR 0x4B
#define IIM42652_REG_ACCEL_WOM_Z_THR 0x4C
#define IIM42652_REG_INT_SOURCE6 0x4D
#define IIM42652_REG_INT_SOURCE7 0x4E
#define IIM42652_REG_INT_SOURCE8 0x4F
#define IIM42652_REG_INT_SOURCE9 0x50
#define IIM42652_REG_INT_SOURCE10 0x51
#define IIM42652_REG_OFFSET_USER0 0x77
#define IIM42652_REG_OFFSET_USER1 0x78
#define IIM42652_REG_OFFSET_USER2 0x79
#define IIM42652_REG_OFFSET_USER3 0x7A
#define IIM42652_REG_OFFSET_USER4 0x7B
#define IIM42652_REG_OFFSET_USER5 0x7C
#define IIM42652_REG_OFFSET_USER6 0x7D
#define IIM42652_REG_OFFSET_USER7 0x7E
#define IIM42652_REG_OFFSET_USER8 0x7F

/**
 * @brief 6DOF IMU description setting.
 * @details Specified setting for description of 6DOF IMU driver.
 */
#define IIM42652_SET_TEMPERATURE_ENABLED 0xDF
#define IIM42652_SET_TEMPERATURE_DISABLED 0x20

#define IIM42652_SET_GYRO_OFF_MODE 0x00
#define IIM42652_SET_GYRO_STANDBY_MODE 0x04
#define IIM42652_SET_GYRO_TLOW_NOISE_MODE 0x0C

#define IIM42652_SET_ACCEL_OFF_MODE 0x00
#define IIM42652_SET_ACCEL_LOW_POWER_MODE 0x02
#define IIM42652_SET_ACCEL_LOW_NOISE_MODE 0x03

#define IIM42652_SET_GYRO_FS_SEL_2000_dps 0x00
#define IIM42652_SET_GYRO_FS_SEL_1000_dps 0x01
#define IIM42652_SET_GYRO_FS_SEL_500_dps 0x02
#define IIM42652_SET_GYRO_FS_SEL_250_dps 0x03
#define IIM42652_SET_GYRO_FS_SEL_125_dps 0x04
#define IIM42652_SET_GYRO_FS_SEL_62_5_dps 0x05
#define IIM42652_SET_GYRO_FS_SEL_31_25_dps 0x06
#define IIM42652_SET_GYRO_FS_SEL_16_625_dps 0x07

#define IIM42652_SET_GYRO_ODR_32kHz 0x01
#define IIM42652_SET_GYRO_ODR_16kHz 0x02
#define IIM42652_SET_GYRO_ODR_8kHz 0x03
#define IIM42652_SET_GYRO_ODR_4kHz 0x04
#define IIM42652_SET_GYRO_ODR_2kHz 0x05
#define IIM42652_SET_GYRO_ODR_1kHz 0x06
#define IIM42652_SET_GYRO_ODR_200Hz 0x07
#define IIM42652_SET_GYRO_ODR_100Hz 0x08
#define IIM42652_SET_GYRO_ODR_50Hz 0x09
#define IIM42652_SET_GYRO_ODR_25Hz 0x0A
#define IIM42652_SET_GYRO_ODR_12_5Hz 0x0B

#define IIM42652_SET_GYRO_UI_FILT_ORD_1st 0x00
#define IIM42652_SET_GYRO_UI_FILT_ORD_2st 0x01
#define IIM42652_SET_GYRO_UI_FILT_ORD_3st 0x02

#define IIM42652_SET_GYRO_DEC2_M2_ORD_3st 0x02

#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_2 0x00
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_4 0x01
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_5 0x02
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_8 0x03
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_10 0x04
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_16 0x05
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_20 0x06
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_40 0x07
#define IIM42652_SET_GYRO_UI_FILT_BW_LOW_LATENCY_0 0x0E
#define IIM42652_SET_GYRO_UI_FILT_BW_LOW_LATENCY_1 0x0F

#define IIM42652_SET_ACCEL_FS_SEL_16g 0x00
#define IIM42652_SET_ACCEL_FS_SEL_8g 0x01
#define IIM42652_SET_ACCEL_FS_SEL_4g 0x02
#define IIM42652_SET_ACCEL_FS_SEL_2g 0x03

#define IIM42652_SET_ACCEL_ODR_32kHz 0x01
#define IIM42652_SET_ACCEL_ODR_16kHz 0x02
#define IIM42652_SET_ACCEL_ODR_8kHz 0x03
#define IIM42652_SET_ACCEL_ODR_4kHz 0x04
#define IIM42652_SET_ACCEL_ODR_2kHz 0x05
#define IIM42652_SET_ACCEL_ODR_1kHz 0x06
#define IIM42652_SET_ACCEL_ODR_200Hz 0x07
#define IIM42652_SET_ACCEL_ODR_100Hz 0x08
#define IIM42652_SET_ACCEL_ODR_50Hz 0x09
#define IIM42652_SET_ACCEL_ODR_25Hz 0x0A
#define IIM42652_SET_ACCEL_ODR_12_5Hz 0x0B

#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_2 0x00
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_4 0x01
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_5 0x02
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_8 0x03
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_10 0x04
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_16 0x05
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_20 0x06
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_40 0x07
#define IIM42652_SET_ACCEL_UI_FILT_BW_LOW_LATENCY_0 0x0E
#define IIM42652_SET_ACCEL_UI_FILT_BW_LOW_LATENCY_1 0x0F

#define IIM42652_SET_ACCEL_UI_FILT_ORD_1st 0x00
#define IIM42652_SET_ACCEL_UI_FILT_ORD_2st 0x01
#define IIM42652_SET_ACCEL_UI_FILT_ORD_3st 0x02

#define IIM42652_SET_ACCEL_DEC2_M2_ORD_3st 0x02

#define IIM42652_SET_BANK_0 0x00
#define IIM42652_SET_BANK_1 0x01
#define IIM42652_SET_BANK_2 0x02
#define IIM42652_SET_BANK_3 0x03
#define IIM42652_SET_BANK_4 0x04

/**
 * @brief   Dummy data.
 * @details Definition of dummy data.
 */
#define DUMMY 0x00
#define BIT_MASK_BIT_0 0x01
#define BIT_MASK_BIT_1 0x02
#define BIT_MASK_BIT_2 0x04
#define BIT_MASK_BIT_3 0x08
#define BIT_MASK_BIT_4 0x10
#define BIT_MASK_BIT_5 0x20
#define BIT_MASK_BIT_6 0x40
#define BIT_MASK_BIT_7 0x80

/*
 * MPUREG_INT_SOURCE0
 * Register Name: INT_SOURCE0
 */
#define BIT_INT_UI_FSYNC_INT_EN_POS 6
#define BIT_INT_PLL_RDY_INT_EN_POS 5
#define BIT_INT_RESET_DONE_INT_EN_POS 4
#define BIT_INT_UI_DRDY_INT_EN_POS 3
#define BIT_INT_FIFO_THS_INT_EN_POS 2
#define BIT_INT_FIFO_FULL_INT_EN_POS 1
#define BIT_INT_UI_AGC_RDY_INT_EN_POS 0

#define BIT_INT_SOURCE0_UI_FSYNC_INT1_EN 0x40
#define BIT_INT_SOURCE0_PLL_RDY_INT1_EN 0x20
#define BIT_INT_SOURCE0_RESET_DONE_INT1_EN 0x10
#define BIT_INT_SOURCE0_UI_DRDY_INT1_EN 0x08
#define BIT_INT_SOURCE0_FIFO_THS_INT1_EN 0x04
#define BIT_INT_SOURCE0_FIFO_FULL_INT1_EN 0x02
#define BIT_INT_SOURCE0_UI_AGC_RDY_INT1_EN 0x01

/*
 * MPUREG_INT_SOURCE1
 * Register Name: INT_SOURCE1
 */
#define BIT_INT_SMD_INT_EN_POS 3
#define BIT_INT_WOM_Z_INT_EN_POS 2
#define BIT_INT_WOM_Y_INT_EN_POS 1
#define BIT_INT_WOM_X_INT_EN_POS 0

#define BIT_INT_SOURCE1_SMD_INT1_EN 0x08
#define BIT_INT_SOURCE1_WOM_Z_INT1_EN 0x04
#define BIT_INT_SOURCE1_WOM_Y_INT1_EN 0x02
#define BIT_INT_SOURCE1_WOM_X_INT1_EN 0x01

#define X_INT1_EN BIT_INT_SOURCE1_WOM_X_INT1_EN
#define Y_INT1_EN BIT_INT_SOURCE1_WOM_Y_INT1_EN
#define Z_INT1_EN BIT_INT_SOURCE1_WOM_Z_INT1_EN

#define WOM_X_INT 0x01
#define WOM_Y_INT 0x02
#define WOM_Z_INT 0x04

/*
 * MPUREG_INT_SOURCE3
 * Register Name: INT_SOURCE3
 */
#define BIT_INT_SOURCE3_UI_FSYNC_INT2_EN 0x40
#define BIT_INT_SOURCE3_PLL_RDY_INT2_EN 0x20
#define BIT_INT_SOURCE3_RESET_DONE_INT2_EN 0x10
#define BIT_INT_SOURCE3_UI_DRDY_INT2_EN 0x08
#define BIT_INT_SOURCE3_FIFO_THS_INT2_EN 0x04
#define BIT_INT_SOURCE3_FIFO_FULL_INT2_EN 0x02
#define BIT_INT_SOURCE3_UI_AGC_RDY_INT2_EN 0x01

/*
 * MPUREG_INT_SOURCE4
 * Register Name: INT_SOURCE4
 */
#define BIT_INT_SOURCE4_SMD_INT2_EN 0x08
#define BIT_INT_SOURCE4_WOM_Z_INT2_EN 0x04
#define BIT_INT_SOURCE4_WOM_Y_INT2_EN 0x02
#define BIT_INT_SOURCE4_WOM_X_INT2_EN 0x01

/*
 * MPUREG_INT_SOURCE6_B4
 * Register Name: INT_SOURCE6
 */
#define BIT_INT_STEP_DET_INT_EN_POS 5
#define BIT_INT_STEP_CNT_OVFL_INT_EN_POS 4
#define BIT_INT_TILT_DET_INT_EN_POS 3
#define BIT_INT_LOWG_DET_INT_EN_POS 2
#define BIT_INT_FF_DET_INT_EN_POS 1
#define BIT_INT_TAP_DET_INT_EN_POS 0

#define BIT_INT_SOURCE6_STEP_DET_INT1_EN 0x20
#define BIT_INT_SOURCE6_STEP_CNT_OVFL_INT1_EN 0x10
#define BIT_INT_SOURCE6_TILT_DET_INT1_EN 0x8
#define BIT_INT_SOURCE6_LOWG_DET_INT1_EN 0x4
#define BIT_INT_SOURCE6_FF_DET_INT1_EN 0x2
#define BIT_INT_SOURCE6_TAP_DET_INT1_EN 0x1

/*
 * MPUREG_INT_SOURCE7_B4
 * Register Name: INT_SOURCE7
 */
#define BIT_INT_SOURCE7_STEP_DET_INT2_EN 0x20
#define BIT_INT_SOURCE7_STEP_CNT_OVFL_INT2_EN 0x10
#define BIT_INT_SOURCE7_TILT_DET_INT2_EN 0x8
#define BIT_INT_SOURCE7_LOWG_DET_INT2_EN 0x4
#define BIT_INT_SOURCE7_FF_DET_INT2_EN 0x2
#define BIT_INT_SOURCE7_TAP_DET_INT2_EN 0x1

/*
 * MPUREG_INT_SOURCE8_B4
 * Register Name: INT_SOURCE8
 */
#define BIT_INT_OIS1_DRDY_IBI_EN_POS 6
#define BIT_INT_UI_FSYNC_IBI_EN_POS 5
#define BIT_INT_PLL_RDY_IBI_EN_POS 4
#define BIT_INT_UI_DRDY_IBI_EN_POS 3
#define BIT_INT_FIFO_THS_IBI_EN_POS 2
#define BIT_INT_FIFO_FULL_IBI_EN_POS 1
#define BIT_INT_UI_AGC_RDY_IBI_EN_POS 0

#define BIT_INT_SOURCE8_OIS1_DRDY_IBI_EN 0x40
#define BIT_INT_SOURCE8_UI_FSYNC_IBI_EN 0x20
#define BIT_INT_SOURCE8_PLL_RDY_IBI_EN 0x10
#define BIT_INT_SOURCE8_UI_DRDY_IBI_EN 0x08
#define BIT_INT_SOURCE8_FIFO_THS_IBI_EN 0x04
#define BIT_INT_SOURCE8_FIFO_FULL_IBI_EN 0x02
#define BIT_INT_SOURCE8_UI_AGC_RDY_IBI_EN 0x01

/*
 * MPUREG_INT_SOURCE9_B4
 * Register Name: INT_SOURCE9
 */
#define BIT_INT_SMD_IBI_EN_POS 4
#define BIT_INT_WOM_Z_IBI_EN_POS 3
#define BIT_INT_WOM_Y_IBI_EN_POS 2
#define BIT_INT_WOM_X_IBI_EN_POS 1

#define BIT_INT_SOURCE9_SMD_IBI_EN 0x10
#define BIT_INT_SOURCE9_WOM_Z_IBI_EN 0x08
#define BIT_INT_SOURCE9_WOM_Y_IBI_EN 0x04
#define BIT_INT_SOURCE9_WOM_X_IBI_EN 0x02

/*
 * MPUREG_INTF_CONFIG1
 * Register Name: INTF_CONFIG1
 */

/* ACCEL_LP_CLK_SEL */
#define BIT_ACCEL_LP_CLK_SEL_POS 3
#define BIT_ACCEL_LP_CLK_SEL_MASK (0x01 << BIT_ACCEL_LP_CLK_SEL_POS)

typedef enum
{
  IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC = (0x00 << BIT_ACCEL_LP_CLK_SEL_POS),
  IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC = (0x01 << BIT_ACCEL_LP_CLK_SEL_POS),
} IIM42652_INTF_CONFIG1_ACCEL_LP_CLK_t;

/*
 * MPUREG_INT_SOURCE10_B4
 * Register Name: INT_SOURCE10
 */
#define BIT_INT_STEP_DET_IBI_EN_POS 5
#define BIT_INT_STEP_CNT_OVFL_IBI_EN_POS 4
#define BIT_INT_TILT_DET_IBI_EN_POS 3
#define BIT_INT_LOWG_DET_IBI_EN_POS 2
#define BIT_INT_FF_DET_IBI_EN_POS 1
#define BIT_INT_TAP_DET_IBI_EN_POS 0

#define BIT_INT_SOURCE10_STEP_DET_IBI_EN 0x20
#define BIT_INT_SOURCE10_STEP_CNT_OVFL_IBI_EN 0x10
#define BIT_INT_SOURCE10_TILT_DET_IBI_EN 0x08
#define BIT_INT_SOURCE10_LOWG_DET_IBI_EN 0x04
#define BIT_INT_SOURCE10_FF_DET_IBI_EN 0x02
#define BIT_INT_SOURCE10_TAP_DET_IBI_EN 0x01

/*
 * MPUREG_GYRO_ACCEL_CONFIG0
 * Register Name: GYRO_ACCEL_CONFIG0
 */

/* ACCEL_UI_FILT_BW_IND */
#define BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS 4
#define BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK (0xF << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS)

typedef enum
{
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_40 = (0x7 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_20 = (0x6 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_16 = (0x5 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_10 = (0x4 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_8 = (0x3 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_5 = (0x2 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_4 = (0x1 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_2 = (0x0 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
} IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t;

typedef enum
{
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_16 = (0x6 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
  IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_1 = (0x1 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
} IIM42652_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t;

/* GYRO_UI_FILT_BW_IND */
#define BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_POS 0
#define BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK 0x0F

typedef enum
{
  IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_40 = 0x07,
  IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_20 = 0x06,
  IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_16 = 0x05,
  IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_10 = 0x04,
  IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_8 = 0x03,
  IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_5 = 0x02,
  IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_4 = 0x01,
  IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_2 = 0x00,
} IIM42652_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t;

/* GYRO_MODE */
#define BIT_PWR_MGMT_0_GYRO_MODE_POS 2
#define BIT_PWR_MGMT_0_GYRO_MODE_MASK (0x03 << BIT_PWR_MGMT_0_GYRO_MODE_POS)

typedef enum
{
  IIM42652_PWR_MGMT_0_GYRO_MODE_LN = (0x03 << BIT_PWR_MGMT_0_GYRO_MODE_POS),
  IIM42652_PWR_MGMT_0_GYRO_MODE_STANDBY = (0x01 << BIT_PWR_MGMT_0_GYRO_MODE_POS),
  IIM42652_PWR_MGMT_0_GYRO_MODE_OFF = (0x00 << BIT_PWR_MGMT_0_GYRO_MODE_POS),
} IIM42652_PWR_MGMT_0_GYRO_MODE_t;

/* ACCEL_MODE */
#define BIT_PWR_MGMT_0_ACCEL_MODE_POS 0
#define BIT_PWR_MGMT_0_ACCEL_MODE_MASK 0x03

typedef enum
{
  IIM42652_PWR_MGMT_0_ACCEL_MODE_LN = 0x03,
  IIM42652_PWR_MGMT_0_ACCEL_MODE_LP = 0x02,
  IIM42652_PWR_MGMT_0_ACCEL_MODE_OFF = 0x00,
} IIM42652_PWR_MGMT_0_ACCEL_MODE_t;

/**
 * @brief   6DOF IMU return value data.
 * @details Predefined enum values for driver return values.
 */
typedef enum
{
  IIM42652_OK = 0,
  IIM42652_ERROR = -1

} IIM42652_return_value_t;

/**
 * @brief ACCEL_FS_SEL
 */
#define BIT_ACCEL_CONFIG0_FS_SEL_POS 5
#define BIT_ACCEL_CONFIG0_FS_SEL_MASK (0x7 << BIT_ACCEL_CONFIG0_FS_SEL_POS)

/**
 * @brief Accelerometer FSR selection
 */
typedef enum
{
  IIM42652_ACCEL_CONFIG0_FS_SEL_RESERVED = (0x4 << BIT_ACCEL_CONFIG0_FS_SEL_POS),
  IIM42652_ACCEL_CONFIG0_FS_SEL_2g = (0x3 << BIT_ACCEL_CONFIG0_FS_SEL_POS),  /*!< 2g*/
  IIM42652_ACCEL_CONFIG0_FS_SEL_4g = (0x2 << BIT_ACCEL_CONFIG0_FS_SEL_POS),  /*!< 4g*/
  IIM42652_ACCEL_CONFIG0_FS_SEL_8g = (0x1 << BIT_ACCEL_CONFIG0_FS_SEL_POS),  /*!< 8g*/
  IIM42652_ACCEL_CONFIG0_FS_SEL_16g = (0x0 << BIT_ACCEL_CONFIG0_FS_SEL_POS), /*!< 16g*/
} IIM42652_ACCEL_CONFIG0_FS_SEL_t;

/**
 * @brief  ACCEL_ODR
 */
#define BIT_ACCEL_CONFIG0_ODR_POS 0
#define BIT_ACCEL_CONFIG0_ODR_MASK 0x0F

/**
 * @brief  Accelerometer ODR selection
 */
typedef enum
{
  IIM42652_ACCEL_CONFIG0_ODR_500_HZ = 0xF,    /*!< 500 Hz (2 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_1_5625_HZ = 0xE, /*!< 1.5625 Hz (640 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_3_125_HZ = 0xD,  /*!< 3.125 Hz (320 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_6_25_HZ = 0xC,   /*!< 6.25 Hz (160 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_12_5_HZ = 0xB,   /*!< 12.5 Hz (80 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_25_HZ = 0xA,     /*!< 25 Hz (40 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_50_HZ = 0x9,     /*!< 50 Hz (20 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_100_HZ = 0x8,    /*!< 100 Hz (10 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_200_HZ = 0x7,    /*!< 200 Hz (5 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_1_KHZ = 0x6,     /*!< 1 KHz (1 ms)*/
  IIM42652_ACCEL_CONFIG0_ODR_2_KHZ = 0x5,     /*!< 2 KHz (500 us)*/
  IIM42652_ACCEL_CONFIG0_ODR_4_KHZ = 0x4,     /*!< 4 KHz (250 us)*/
  IIM42652_ACCEL_CONFIG0_ODR_8_KHZ = 0x3,     /*!< 8 KHz (125 us)*/
  IIM42652_ACCEL_CONFIG0_ODR_16_KHZ = 0x2,    /*!< 16 KHz (62.5 us)*/
  IIM42652_ACCEL_CONFIG0_ODR_32_KHZ = 0x1,    /*!< 32 KHz (31.25 us)*/
} IIM42652_ACCEL_CONFIG0_ODR_t;

/* GYRO_FS_SEL*/
#define BIT_GYRO_CONFIG0_FS_SEL_POS 5
#define BIT_GYRO_CONFIG0_FS_SEL_MASK (7 << BIT_GYRO_CONFIG0_FS_SEL_POS)

/** @brief Gyroscope FSR selection
 */
typedef enum
{
  IIM42652_GYRO_CONFIG0_FS_SEL_16dps = (7 << BIT_GYRO_CONFIG0_FS_SEL_POS),   /*!< 16dps*/
  IIM42652_GYRO_CONFIG0_FS_SEL_31dps = (6 << BIT_GYRO_CONFIG0_FS_SEL_POS),   /*!< 31dps*/
  IIM42652_GYRO_CONFIG0_FS_SEL_62dps = (5 << BIT_GYRO_CONFIG0_FS_SEL_POS),   /*!< 62dps*/
  IIM42652_GYRO_CONFIG0_FS_SEL_125dps = (4 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 125dps*/
  IIM42652_GYRO_CONFIG0_FS_SEL_250dps = (3 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 250dps*/
  IIM42652_GYRO_CONFIG0_FS_SEL_500dps = (2 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 500dps*/
  IIM42652_GYRO_CONFIG0_FS_SEL_1000dps = (1 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 1000dps*/
  IIM42652_GYRO_CONFIG0_FS_SEL_2000dps = (0 << BIT_GYRO_CONFIG0_FS_SEL_POS), /*!< 2000dps*/
} IIM42652_GYRO_CONFIG0_FS_SEL_t;

/* GYRO_ODR */
#define BIT_GYRO_CONFIG0_ODR_POS 0
#define BIT_GYRO_CONFIG0_ODR_MASK 0x0F

/** @brief Gyroscope ODR selection
 */
typedef enum
{
  IIM42652_GYRO_CONFIG0_ODR_500_HZ = 0x0F,  /*!< 500 Hz (2 ms)*/
  IIM42652_GYRO_CONFIG0_ODR_12_5_HZ = 0x0B, /*!< 12.5 Hz (80 ms)*/
  IIM42652_GYRO_CONFIG0_ODR_25_HZ = 0x0A,   /*!< 25 Hz (40 ms)*/
  IIM42652_GYRO_CONFIG0_ODR_50_HZ = 0x09,   /*!< 50 Hz (20 ms)*/
  IIM42652_GYRO_CONFIG0_ODR_100_HZ = 0x08,  /*!< 100 Hz (10 ms)*/
  IIM42652_GYRO_CONFIG0_ODR_200_HZ = 0x07,  /*!< 200 Hz (5 ms)*/
  IIM42652_GYRO_CONFIG0_ODR_1_KHZ = 0x06,   /*!< 1 KHz (1 ms)*/
  IIM42652_GYRO_CONFIG0_ODR_2_KHZ = 0x05,   /*!< 2 KHz (500 us)*/
  IIM42652_GYRO_CONFIG0_ODR_4_KHZ = 0x04,   /*!< 4 KHz (250 us)*/
  IIM42652_GYRO_CONFIG0_ODR_8_KHZ = 0x03,   /*!< 8 KHz (125 us)*/
  IIM42652_GYRO_CONFIG0_ODR_16_KHZ = 0x02,  /*!< 16 KHz (62.5 us)*/
  IIM42652_GYRO_CONFIG0_ODR_32_KHZ = 0x01,  /*!< 32 KHz (31.25 us)*/
} IIM42652_GYRO_CONFIG0_ODR_t;

/*
 * MPUREG_SMD_CONFIG
 * Register Name: SMD_CONFIG
 */

/* WOM_INT_MODE */
#define BIT_SMD_CONFIG_WOM_INT_MODE_POS 3
#define BIT_SMD_CONFIG_WOM_INT_MODE_MASK (0x1 << BIT_SMD_CONFIG_WOM_INT_MODE_POS)

typedef enum
{
  IIM42652_SMD_CONFIG_WOM_INT_MODE_ANDED = (0x01 << BIT_SMD_CONFIG_WOM_INT_MODE_POS),
  IIM42652_SMD_CONFIG_WOM_INT_MODE_ORED = (0x00 << BIT_SMD_CONFIG_WOM_INT_MODE_POS),
} IIM42652_SMD_CONFIG_WOM_INT_MODE_t;

/* WOM_MODE */
#define BIT_SMD_CONFIG_WOM_MODE_POS 2
#define BIT_SMD_CONFIG_WOM_MODE_MASK (0x1 << BIT_SMD_CONFIG_WOM_MODE_POS)

typedef enum
{
  IIM42652_SMD_CONFIG_WOM_MODE_CMP_PREV = (0x01 << BIT_SMD_CONFIG_WOM_MODE_POS),
  IIM42652_SMD_CONFIG_WOM_MODE_CMP_INIT = (0x00 << BIT_SMD_CONFIG_WOM_MODE_POS),
} IIM42652_SMD_CONFIG_WOM_MODE_t;

/* SMD_MODE */
#define BIT_SMD_CONFIG_SMD_MODE_POS 0
#define BIT_SMD_CONFIG_SMD_MODE_MASK 0x3

typedef enum
{
  IIM42652_SMD_CONFIG_SMD_MODE_LONG = 0x03,
  IIM42652_SMD_CONFIG_SMD_MODE_SHORT = 0x02,
  IIM42652_SMD_CONFIG_SMD_MODE_WOM = 0x01,
  IIM42652_SMD_CONFIG_SMD_MODE_DISABLED = 0x00,
} IIM42652_SMD_CONFIG_SMD_MODE_t;

/* Interrupt enum state for INT1, INT2, and IBI */
typedef enum
{
  IIM42652_DISABLE = 0,
  IIM42652_ENABLE
} IIM42652_interrupt_value;

/** @brief IIM42652 set of interrupt enable flag
 */
typedef struct
{
  IIM42652_interrupt_value IIM42652_UI_FSYNC;
  IIM42652_interrupt_value IIM42652_UI_DRDY;
  IIM42652_interrupt_value IIM42652_FIFO_THS;
  IIM42652_interrupt_value IIM42652_FIFO_FULL;
  IIM42652_interrupt_value IIM42652_SMD;
  IIM42652_interrupt_value IIM42652_WOM_X;
  IIM42652_interrupt_value IIM42652_WOM_Y;
  IIM42652_interrupt_value IIM42652_WOM_Z;
  IIM42652_interrupt_value IIM42652_STEP_DET;
  IIM42652_interrupt_value IIM42652_STEP_CNT_OVFL;
  IIM42652_interrupt_value IIM42652_TILT_DET;
  IIM42652_interrupt_value IIM42652_FF_DET;
  IIM42652_interrupt_value IIM42652_LOWG_DET;
  IIM42652_interrupt_value IIM42652_TAP_DET;
} IIM42652_interrupt_parameter_t;

/**
 * @brief   6DOF IMU structure object.
 * @details Axis structure object definition of 6DOF IMU driver.
 */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;

} IIM42652_axis_t;

/**
 * @brief   6DOF IMU configuration structure object.
 * @details Gyro configuration structure object definition of 6DOF IMU driver.
 */
typedef struct
{
  uint8_t gyro_fs_sel;
  uint8_t gyro_odr;
  uint8_t gyro_ui_filt_ord;
  uint8_t gyro_dec2_m2_ord;
  uint8_t gyro_ui_filt_bw;

} IIM42652_gyro_cfg_t;

/**
 * @brief   6DOF IMU Accelerometer configuration structure object.
 * @details Accelerometer configuration structure object definition of 6DOF IMU driver.
 */
typedef struct
{
  uint8_t accel_fs_sel;
  uint8_t accel_odr;
  uint8_t accel_ui_filt_bw;
  uint8_t accel_ui_filt_ord;
  uint8_t accel_dec2_m2_ord;
} IIM42652_accel_cfg_t;

class IIM42652
{
public:
  IIM42652(byte addr = IIM42652_SET_DEV_ADDR);
  bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = IIM42652_SET_DEV_ADDR);

  void get_device_id(uint8_t *device_id);
  void bank_selection(uint8_t bank_sel);
  void temperature_enable(void);
  void temperature_disable(void);
  void gyroscope_enable(void);
  void gyroscope_disable(void);
  void accelerometer_enable(void);
  void accelerometer_disable(void);
  void idle(void);
  void ex_idle(void);

  void soft_reset(void);
  void get_accel_data(IIM42652_axis_t *accel_data);
  void get_gyro_data(IIM42652_axis_t *gyro_data);
  void get_temperature(float *temperature);
  void set_accel_fsr(IIM42652_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g);
  void set_accel_frequency(const IIM42652_ACCEL_CONFIG0_ODR_t frequency);
  void set_gyro_fsr(IIM42652_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps);
  void set_gyro_frequency(const IIM42652_GYRO_CONFIG0_ODR_t frequency);

  void wake_on_motion_configuration(const uint8_t x_th, const uint8_t y_th, const uint8_t z_th);
  uint8_t get_WOM_INT(void);
  void enable_accel_low_power_mode(void);
  void pedometer_configuration();
  uint16_t get_pedometer_data(void);
  void writeRegister(uint8_t registerAddress, uint8_t *writeData, uint8_t size);
  void readRegister(uint8_t registerAddress, uint8_t *readData, uint8_t size);

private:
  TwoWire *_i2cPort = &Wire; // The generic connection to user's chosen I2C hardware
  uint8_t _deviceAddress;
};

#endif