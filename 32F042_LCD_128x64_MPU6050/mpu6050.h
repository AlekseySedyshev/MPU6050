/*
 *
 * Modified by Alex Sedyshev for STM32F042x6.. (CMSIS )
 *  https://github.com/AlekseySedyshev
 * 
*/

/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/10/hal-library-30-mpu6050-for-stm32fxxx
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   MPU6050 library for STM32Fxxx devices
 *	
@verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef MPU6050_H
#define MPU6050_H
#include "stm32f0xx.h"
#include "stdint.h"


/**
 * @defgroup TM_MPU6050_Macros
 * @brief    Library defines
 * @{
 */

/**
 * @brief  Data rates predefined constants
 * @{
 */
#define TM_MPU6050_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define TM_MPU6050_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define TM_MPU6050_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define TM_MPU6050_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define TM_MPU6050_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define TM_MPU6050_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define TM_MPU6050_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define TM_MPU6050_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum _TM_MPU6050_Device_t {
	TM_MPU6050_Device_0 = 0x00, /*!< AD0 pin is set to low */
	TM_MPU6050_Device_1 = 0x02  /*!< AD0 pin is set to high */
} TM_MPU6050_Device_t;

/**
 * @brief  MPU6050 result enumeration	
 */
/**
 * @brief  Parameters for accelerometer range
 */
typedef enum _TM_MPU6050_Accelerometer_t {
	TM_MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	TM_MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	TM_MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	TM_MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} TM_MPU6050_Accelerometer_t;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum _TM_MPU6050_Gyroscope_t {
	TM_MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	TM_MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	TM_MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	TM_MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} TM_MPU6050_Gyroscope_t;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct _TM_MPU6050_t {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. Only for private use */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float Temperature;       /*!< Temperature in degrees */
} TM_MPU6050_t;

/**
 * @brief  Interrupts union and structure
 */
typedef union _TM_MPU6050_Interrupt_t {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} TM_MPU6050_Interrupt_t;

/* @defgroup TM_MPU6050_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref TM_MPU6050_t structure
 * @param  DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be TM_MPU6050_Device_0,
 *          but if AD0 pin is high, then you should use TM_MPU6050_Device_1
 *          
 *          Parameter can be a value of @ref TM_MPU6050_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref TM_MPU6050_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref TM_MPU6050_Gyroscope_t enumeration
 * @retval Initialization status:
 *            - TM_MPU6050_Result_t: Everything OK
 *            - Other member: in other cases
 */
void MPU6050_Init(TM_MPU6050_t* DataStruct, TM_MPU6050_Device_t DeviceNumber, TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure indicating MPU6050 device
 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref TM_MPU6050_Gyroscope_t enumeration
 * @retval Member of @ref TM_MPU6050_Result_t enumeration
 */
void MPU6050_SetGyroscope(TM_MPU6050_t* DataStruct, TM_MPU6050_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief  Sets accelerometer sensitivity
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure indicating MPU6050 device
 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref TM_MPU6050_Accelerometer_t enumeration
 * @retval Member of @ref TM_MPU6050_Result_t enumeration
 */
void MPU6050_SetAccelerometer(TM_MPU6050_t* DataStruct, TM_MPU6050_Accelerometer_t AccelerometerSensitivity);

/**
 * @brief  Sets output data rate
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure indicating MPU6050 device
 * @param  rate: Data rate value. An 8-bit value for prescaler value
 * @retval Member of @ref TM_MPU6050_Result_t enumeration
 */
void MPU6050_SetDataRate(TM_MPU6050_t* DataStruct, uint8_t rate);

/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref TM_MPU6050_Result_t enumeration
 */
void MPU6050_EnableInterrupts(TM_MPU6050_t* DataStruct);

/**
 * @brief  Disables interrupts
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref TM_MPU6050_Result_t enumeration
 */
void MPU6050_DisableInterrupts(TM_MPU6050_t* DataStruct);

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure indicating MPU6050 device
 * @param  *InterruptsStruct: Pointer to @ref TM_MPU6050_Interrupt_t structure to store status in
 * @retval Member of @ref TM_MPU6050_Result_t enumeration
 */
void MPU6050_ReadInterrupts(TM_MPU6050_t* DataStruct, TM_MPU6050_Interrupt_t* InterruptsStruct);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
void MPU6050_ReadAccelerometer(TM_MPU6050_t* DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
void MPU6050_ReadGyroscope(TM_MPU6050_t* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
void MPU6050_ReadTemperature(TM_MPU6050_t* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
void MPU6050_ReadAll(TM_MPU6050_t* DataStruct);
void 			writeReg8(uint8_t reg, uint8_t value);
uint8_t 	readReg8(uint8_t reg);
void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);
#endif
