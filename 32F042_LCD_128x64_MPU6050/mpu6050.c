
/*
 *
 * Modified by Alex Sedyshev for STM32F042x6.. (CMSIS )
 *  https://github.com/AlekseySedyshev
 * 
*/

/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */


#include "stm32f0xx.h"
#include "mpu6050.h"

/* Default I2C address */
#define MPU6050_I2C_ADDR					0xD0

/* Who I am register value */
#define MPU6050_I_AM							0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO					0x01
#define MPU6050_SMPLRT_DIV				0x19
#define MPU6050_CONFIG						0x1A
#define MPU6050_GYRO_CONFIG				0x1B
#define MPU6050_ACCEL_CONFIG			0x1C
#define MPU6050_MOTION_THRESH			0x1F
#define MPU6050_INT_PIN_CFG				0x37
#define MPU6050_INT_ENABLE				0x38
#define MPU6050_INT_STATUS				0x3A
#define MPU6050_ACCEL_XOUT_H			0x3B
#define MPU6050_ACCEL_XOUT_L			0x3C
#define MPU6050_ACCEL_YOUT_H			0x3D
#define MPU6050_ACCEL_YOUT_L			0x3E
#define MPU6050_ACCEL_ZOUT_H			0x3F
#define MPU6050_ACCEL_ZOUT_L			0x40
#define MPU6050_TEMP_OUT_H				0x41
#define MPU6050_TEMP_OUT_L				0x42
#define MPU6050_GYRO_XOUT_H				0x43
#define MPU6050_GYRO_XOUT_L				0x44
#define MPU6050_GYRO_YOUT_H				0x45
#define MPU6050_GYRO_YOUT_L				0x46
#define MPU6050_GYRO_ZOUT_H				0x47
#define MPU6050_GYRO_ZOUT_L				0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL					0x6A
#define MPU6050_PWR_MGMT_1				0x6B
#define MPU6050_PWR_MGMT_2				0x6C
#define MPU6050_FIFO_COUNTH				0x72
#define MPU6050_FIFO_COUNTL				0x73
#define MPU6050_FIFO_R_W					0x74
#define MPU6050_WHO_AM_I					0x75

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000	((float) 32.8)
#define MPU6050_GYRO_SENS_2000	((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)


void 			writeReg8(uint8_t reg, uint8_t value)																						{//Write a 8-bit register
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (MPU6050_I2C_ADDR << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = value;	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
}
uint8_t 	readReg8(uint8_t reg)																														{//Read an 8-bit register
	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (MPU6050_I2C_ADDR << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){};
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (MPU6050_I2C_ADDR << I2C_CR2_SADD_Pos) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_RXNE)){};		
	uint8_t value=I2C1->RXDR;
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
	return value; 		
}



void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)													{// writeMulti
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (MPU6050_I2C_ADDR << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while (count-- > 0) {
		while(!(I2C1->ISR & I2C_ISR_TXE)){};  
		I2C1->TXDR= *(src++);
		while((I2C1->ISR & I2C_ISR_TC)){};
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
}



void readMulti(uint8_t reg, uint8_t * dst, uint8_t count)																	{// readMulti
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (MPU6050_I2C_ADDR << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY	
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (MPU6050_I2C_ADDR << I2C_CR2_SADD_Pos) | (count << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while (count-- > 0) {
		
		while(!(I2C1->ISR & I2C_ISR_RXNE)){};
		*(dst++) = I2C1->RXDR;
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
}




void MPU6050_Init(TM_MPU6050_t* DataStruct, TM_MPU6050_Device_t DeviceNumber, TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity) {
	
	uint8_t temp;
	
	
	/* Check who am I */
	temp=readReg8(MPU6050_WHO_AM_I);
	if (temp != MPU6050_I_AM) { return; }
	
	/* Wakeup MPU6050 */
	writeReg8(MPU6050_PWR_MGMT_1, 0x00);
	
	/* Set sample rate to 1kHz */
	MPU6050_SetDataRate(DataStruct, TM_MPU6050_DataRate_1KHz);
	
	/* Config accelerometer */
	MPU6050_SetAccelerometer(DataStruct, AccelerometerSensitivity);
	
	/* Config accelerometer */
	MPU6050_SetGyroscope(DataStruct, GyroscopeSensitivity);

}

void MPU6050_SetGyroscope(TM_MPU6050_t* DataStruct, TM_MPU6050_Gyroscope_t GyroscopeSensitivity) {
	uint8_t temp;
	
	/* Config gyroscope */
	temp	= readReg8(MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	writeReg8(MPU6050_GYRO_CONFIG, temp);
	
	switch (GyroscopeSensitivity) {
		case TM_MPU6050_Gyroscope_250s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250; 
			break;
		case TM_MPU6050_Gyroscope_500s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500; 
			break;
		case TM_MPU6050_Gyroscope_1000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000; 
			break;
		case TM_MPU6050_Gyroscope_2000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000; 
		default:
			break;
	}
	
}

void MPU6050_SetAccelerometer(TM_MPU6050_t* DataStruct, TM_MPU6050_Accelerometer_t AccelerometerSensitivity) {
	uint8_t temp;
	
	/* Config accelerometer */
	temp = (readReg8(MPU6050_ACCEL_CONFIG)& 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	writeReg8(MPU6050_ACCEL_CONFIG, temp);
	
	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case TM_MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2; 
			break;
		case TM_MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4; 
			break;
		case TM_MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8; 
			break;
		case TM_MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16; 
		default:
			break;
	}
}

void MPU6050_SetDataRate(TM_MPU6050_t* DataStruct, uint8_t rate) {
	/* Set data sample rate */
	writeReg8(MPU6050_SMPLRT_DIV, rate); 
}
	

void MPU6050_EnableInterrupts(TM_MPU6050_t* DataStruct) {
	uint8_t temp;	
	
	/* Enable interrupts for data ready and motion detect */
	writeReg8(MPU6050_INT_ENABLE, 0x21);
	
	/* Clear IRQ flag on any read operation */
	temp=(readReg8(MPU6050_INT_PIN_CFG) | 0x10);
	writeReg8(MPU6050_INT_PIN_CFG, temp);
	
}

void MPU6050_DisableInterrupts(TM_MPU6050_t* DataStruct) {
	/* Disable interrupts */
	writeReg8(MPU6050_INT_ENABLE, 0x00);
}

void MPU6050_ReadInterrupts(TM_MPU6050_t* DataStruct, TM_MPU6050_Interrupt_t* InterruptsStruct) {
	uint8_t read;
	
	/* Reset structure */
	InterruptsStruct->Status = 0;
	
	/* Read interrupts status register */
	read = readReg8(MPU6050_INT_STATUS);
	
	/* Fill value */
	InterruptsStruct->Status = read;
}

void MPU6050_ReadAccelerometer(TM_MPU6050_t* DataStruct) {
	uint8_t data[6];
	
	/* Read accelerometer data */
	readMulti(MPU6050_ACCEL_XOUT_H, data, 6);
	
	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
	
}

void MPU6050_ReadGyroscope(TM_MPU6050_t* DataStruct) {
	uint8_t data[6];
	
	/* Read gyroscope data */
	readMulti(MPU6050_GYRO_XOUT_H, data, 6);
	
	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

}

void MPU6050_ReadTemperature(TM_MPU6050_t* DataStruct) {
	uint8_t data[2];
	int16_t temp;
	
	/* Read temperature */
	readMulti(MPU6050_TEMP_OUT_H, data, 2);
	
	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	
}

void MPU6050_ReadAll(TM_MPU6050_t* DataStruct) {
	uint8_t data[14];
	int16_t temp;
	
	/* Read full raw data, 14bytes */
	readMulti(MPU6050_ACCEL_XOUT_H, data, 14);
	
	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

}


