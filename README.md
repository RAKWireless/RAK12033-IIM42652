| <center><img src="./assets/rakstar.jpg" alt="RAKstar" width=25%></center>  | ![RAKWireless](./assets/RAK-Whirls.png) | [![Build Status](https://github.com/RAKWireless/RAK2033-IIM42652/workflows/RAK%20Library%20Build%20CI/badge.svg)](https://github.com/RAKWireless/RAK2033-IIM42652/actions) |
| -- | -- | -- |

# <RAK12033>

RAK12033-IIM42652 is written by RAK for TDK IIM42652. It provides 6-axis data and temperature data reading, measurement range setting, threshold trigger wake-up and other functions.

[*RAKWireless RAK12033*](https://store.rakwireless.com/collections/wisblock-sensor)

# Documentation

* **[Product Repository](https://github.com/RAKWireless/RAK12033-IIM42652)** - Product repository for the RAKWireless RAK12033 6-Axis module.
* **[Documentation](https://docs.RAKWireless.com/Product-Categories/WisBlock/RAK12033/Overview/)** - Documentation and Quick Start Guide for the RAK12033 6-Axis module.

# Installation

In Arduino IDE open Sketch->Include Library->Manage Libraries then search for RAK12033.

In PlatformIO open PlatformIO Home, switch to libraries and search for RAK12033.
Or install the library project dependencies by adding

```log
lib_deps =
  RAKWireless/RAKWireless IIM42652 6-Axis library
```

into **`platformio.ini`**

For manual installation download the archive, unzip it and place the RAK12033-IIM42652 folder into the library directory.
In Arduino IDE this is usually <arduinosketchfolder>/libraries/
In PlatformIO this is usually <user/.platformio/lib>

# Usage

The library provides IIM42652 class, which allows communication with IIM42652 via IIC. These examples show how to use RAK12033.

- [RAK12033_6_Axis_BasicReadings_IIM_42652](./examples/RAK12033_6_Axis_BasicReadings_IIM_42652) Get IIM-42652 sensor data and output data on the serial port.
- [RAK12033_6_Axis_Interrupt_IIM_42652](./examples/RAK12033_6_Axis_Interrupt_IIM_42652) Trigger an interrupt when the acceleration on the axis exceeds the threshold.
- [RAK12033_6_Axis_SetRange_IIM_42652](./examples/RAK12033_6_Axis_SetRange_IIM_42652) Setting the ACC and GYRO data output rate and measurement range.
- [RAK12033_6_Axis_Calibration_IIM_42652](./examples/RAK12033_6_Axis_Calibration_IIM_42652) Perform zero offset calibration on the gyro output within 5 seconds after reset.  Keep the gyro absolutely still during calibration. Only the zero offset of the gyroscope is calibrated. Using the arithmetic mean calibration method.  Can be changed to a calibration method more suitable for your application.

## This class provides the following methods:

**bool begin(TwoWire &wirePort, uint8_t deviceAddress)**

Initalizes the IIM42652 sensor.

#### Parameters:

| Direction | Name          | Function                                                     |
| --------- | ------------- | ------------------------------------------------------------ |
| in        | wirePort      | IIC interface used.                                          |
| in        | deviceAddress | Device address should be 0x68.                               |
| return    |               | If the device init successful return true else return false. |

**void bank_selection ( uint8_t bank_sel ) **

Register bank selection.

#### Parameters:

| Direction | Name     | Function                                                     |
| --------- | -------- | ------------------------------------------------------------ |
| in        | bank_sel | Selected bank.The parameters can be the following values :<br /> IIM42652_SET_BANK_0<br /> IIM42652_SET_BANK_1<br /> IIM42652_SET_BANK_2<br /> IIM42652_SET_BANK_3<br /> IIM42652_SET_BANK_0 |
| return    |          | none                                                         |

**void get_device_id ( uint8_t *device_id ) **

Get device ID.

#### Parameters:

| Direction | Name      | Function                    |
| --------- | --------- | --------------------------- |
| in        | device_id | Device ID variable pointer. |
| return    |           | none                        |

**void temperature_enable ( void ) **

Enable IIM42652 temperature measurement.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void temperature_disable( void ) **

Disable IIM42652 temperature measurement.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void gyroscope_enable ( void )**

Enable IIM42652 gyroscope measurement.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void gyroscope_disable( void )**

Disable IIM42652 gyroscope measurement.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void accelerometer_enable ( void )**

Enable IIM42652 acceleration measurement.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void accelerometer_disable( void )**

Disable IIM42652 acceleration measurement.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void idle( void )**

IDLE mode. When Accel and Gyro are powered off. The chip will go to OFF state, since the RC oscillator will also be powered off.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void ex_idle( void )**

Exit IDLE mode. The RC oscillator is powered on even if Accel and Gyro are powered off.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void soft_reset( void )**

Software reset IIM42652.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void get_accel_data ( IIM42652_axis_t *accel_data ) **

Get IIM42652 acceleration data.

#### Parameters:

| Direction | Name       | Function                                                     |
| --------- | ---------- | ------------------------------------------------------------ |
| in        | accel_data | Pointer to data of type .@ IIM42652_axis_t.<br />typedef struct <br/>{<br/>  int16_t x;<br/>  int16_t y;<br/>  int16_t z;<br/>} IIM42652_axis_t; |
| return    |            | none                                                         |

**void get_gyro_data( IIM42652_axis_t *gyro_data) **

Get IIM42652 gyroscope data.

#### Parameters:

| Direction | Name      | Function                                                     |
| --------- | --------- | ------------------------------------------------------------ |
| in        | gyro_data | Pointer to data of type .@ IIM42652_axis_t.<br />typedef struct <br/>{<br/>  int16_t x;<br/>  int16_t y;<br/>  int16_t z;<br/>} IIM42652_axis_t; |
| return    |           | none                                                         |

**void get_temperature ( float *temperature )  **

Get IIM42652 temperature data.

#### Parameters:

| Direction | Name        | Function                         |
| --------- | ----------- | -------------------------------- |
| in        | temperature | Pointer to temperature variable. |
| return    |             | none                             |

**void set_accel_fsr(IIM42652_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g)**

Set IIM42652 ACC full scale range.

#### Parameters:

| Direction | Name        | Function                                                     |
| --------- | ----------- | ------------------------------------------------------------ |
| in        | accel_fsr_g | Full scal range value. Reference@ IIM42652_ACCEL_CONFIG0_FS_SEL_t. |
| return    |             | none                                                         |

**void set_accel_frequency(const IIM42652_ACCEL_CONFIG0_ODR_t frequency)**

Set IIM42652 ACC output data rate.

#### Parameters:

| Direction | Name      | Function                                                     |
| --------- | --------- | ------------------------------------------------------------ |
| in        | frequency | Output data rate value. Reference@ IIM42652_ACCEL_CONFIG0_ODR_t. |
| return    |           | none                                                         |

**void set_gyro_fsr(IIM42652_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps)**

Set IIM42652 GYRO full scale range.

#### Parameters:

| Direction | Name         | Function                                                     |
| --------- | ------------ | ------------------------------------------------------------ |
| in        | gyro_fsr_dps | Full scal range value. Reference@ IIM42652_GYRO_CONFIG0_FS_SEL_t. |
| return    |              | none                                                         |

**void set_gyro_frequency(const IIM42652_GYRO_CONFIG0_ODR_t frequency)**

Set IIM42652 GYRO output data rate.

#### Parameters:

| Direction | Name      | Function                                                     |
| --------- | --------- | ------------------------------------------------------------ |
| in        | frequency | Output data rate value. Reference@ IIM42652_GYRO_CONFIG0_ODR_t. |
| return    |           | none                                                         |

**uint8_t get_WOM_INT(void)**

Get WOM interrupt flag, clears on read.

#### Parameters:

| Direction | Name | Function                                                     |
| --------- | ---- | ------------------------------------------------------------ |
| return    |      | Return the value in register INT_STATUS2. <br/>For details, refer to the description of INT_STATUS2 in the date sheet. |

**void wake_on_motion_configuration(const uint8_t x_th, const uint8_t y_th, const uint8_t z_th )**

Wake on motion configuration.

#### Parameters:

| Direction | Name | Function                                                     |
| --------- | ---- | ------------------------------------------------------------ |
| in        | x_th | Threshold value for the Wake on Motion Interrupt for X-axis accelerometer WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg |
| in        | y_th | Threshold value for the Wake on Motion Interrupt for Y-axis accelerometer WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg |
| in        | z_th | Threshold value for the Wake on Motion Interrupt for Z-axis accelerometer WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg |
| return    |      | none                                                         |

**void enable_accel_low_power_mode(void)**

Set ACC to low power mode.

#### Parameters:

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |
