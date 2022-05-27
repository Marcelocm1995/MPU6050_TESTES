/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen MAJERLE
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
#include "MPU6050.h"
#include <math.h>

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

#define MPU6050_ADDR 0x00D0

#define RAD_TO_DEG 57.295779513082320876798154814105

const double Accel_Z_corrector = 14418.0;
uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

MPU6050_Result_t MPU6050_Init(MPU6050_t* DataStruct, MPU6050_Device_t DeviceNumber) 
{
	uint8_t temp;
	
	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;
	
	/* Check if device is connected */
	if ( HAL_I2C_IsDeviceReady(&hi2c1, DataStruct->Address, 10, 1000) != HAL_OK)
	{
		/* Return error */
		return MPU6050_Result_DeviceNotConnected;
	}
	
	/* Check who am I */
	uint8_t myreg[1] = {MPU6050_WHO_AM_I};
	
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, myreg, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}
	
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, &temp, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}
	
	if (temp != MPU6050_I_AM) {
		/* Return error */
		return MPU6050_Result_DeviceInvalid;
	}
	
	/* Wakeup MPU6050 */

		uint8_t d[2];			
		/* Format array to send */
		d[0] = MPU6050_PWR_MGMT_1;
		d[1] = 0x00;
		
		if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, d, 2, 1000) != HAL_OK) {
			/* Check error */
			
			/* Return error */
			return MPU6050_Result_Error;
		} 	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_SetGyroscope(MPU6050_t* DataStruct, MPU6050_Gyroscope_t GyroscopeSensitivity) 
{
	uint8_t temp;
	uint8_t data;
	
	/* Config gyroscope */
	uint8_t myreg[1] = {MPU6050_GYRO_CONFIG};
	/* Send address */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, myreg, 1, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, &data, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}
		
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	
	uint8_t d[2];
		
	/* Format array to send */
	d[0] = MPU6050_GYRO_CONFIG;
	d[1] = temp;
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, d, 2, 1000) != HAL_OK) 
	{
		/* Return error */
		return MPU6050_Result_Error;
	} 
		
	switch (GyroscopeSensitivity) {
		case MPU6050_Gyroscope_250s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250; 
			break;
		case MPU6050_Gyroscope_500s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500; 
			break;
		case MPU6050_Gyroscope_1000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000; 
			break;
		case MPU6050_Gyroscope_2000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000; 
		default:
			break;
	}
	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_SetAccelerometer(MPU6050_t* DataStruct, MPU6050_Accelerometer_t AccelerometerSensitivity) {
	uint8_t temp;
	uint8_t myreg[1] = {MPU6050_ACCEL_CONFIG};
	/* Config accelerometer */	
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, myreg, 1, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, &temp, 1, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
	
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	
	uint8_t d[2];
		
	/* Format array to send */
	d[0] = MPU6050_ACCEL_CONFIG;
	d[1] = temp;
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, d, 2, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	} 
	
	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2; 
			break;
		case MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4; 
			break;
		case MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8; 
			break;
		case MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16; 
		default:
			break;
	}
	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_SetDataRate(MPU6050_t* DataStruct, uint8_t rate) 
{
	uint8_t d[2];
	/* Format array to send */
	d[0] = MPU6050_SMPLRT_DIV;
	d[1] = rate;
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, d, 2, 1000) != HAL_OK) {
		/* Check error */
				
		/* Return error */
		return MPU6050_Result_Error;
	} 
	
	/* Return OK */
	return MPU6050_Result_Ok;
}
	

MPU6050_Result_t MPU6050_EnableInterrupts(MPU6050_t* DataStruct) 
{
	uint8_t temp;	
	
	/* Enable interrupts for data ready and motion detect */
	uint8_t d[2];
		
	/* Format array to send */
	d[0] = MPU6050_INT_ENABLE;
	d[1] = 0x21;
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, d, 2, 1000) != HAL_OK) 
	{		
		/* Return error */
		return MPU6050_Result_Error;
	} 
		
	/* Clear IRQ flag on any read operation */
	uint8_t adr[2];
	uint8_t data;
	
	/* Format I2C address */
	adr[0] = (DataStruct->Address >> 8) & 0xFF; /* High byte */
	adr[1] = (DataStruct->Address) & 0xFF;      /* Low byte */
	
	/* Send address */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, adr, 2, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, &data, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}
		
	temp |= 0x10;
		
	/* Format array to send */
	d[0] = MPU6050_INT_PIN_CFG;
	d[1] = data;
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, d, 2, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	} 
	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_DisableInterrupts(MPU6050_t* DataStruct) 
{
	/* Disable interrupts */
	uint8_t d[2];
		
	/* Format array to send */
	d[0] = MPU6050_INT_ENABLE;
	d[1] = 0x00;
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, d, 2, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	} 
	
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadInterrupts(MPU6050_t* DataStruct, MPU6050_Interrupt_t* InterruptsStruct) 
{
	uint8_t read;
	uint8_t myreg[1] = {MPU6050_INT_STATUS};
	/* Reset structure */
	InterruptsStruct->Status = 0;
	
	/* Read interrupts status register */
	
	/* Send address */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, myreg, 1, 1000) != HAL_OK) 
		{
		return MPU6050_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, &read, 1, 1000) != HAL_OK) 
	{	
		return MPU6050_Result_Error;
	}

	/* Fill value */
	InterruptsStruct->Status = read;
	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadAccelerometer(MPU6050_t* DataStruct) 
{
	uint8_t data[6];
	uint8_t myreg[1] = {MPU6050_ACCEL_XOUT_H};
	/* Read accelerometer data */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, myreg, 1, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, data, 6, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
		
	/* Format */
	DataStruct->Accel_X_RAW  = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accel_Y_RAW  = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accel_Z_RAW  = (int16_t)(data[4] << 8 | data[5]);

	/*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW * DataStruct->Acce_Mult;
    DataStruct->Ay = DataStruct->Accel_Y_RAW * DataStruct->Acce_Mult;
    DataStruct->Az = DataStruct->Accel_Z_RAW * DataStruct->Acce_Mult;
	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadGyroscope(MPU6050_t* DataStruct) 
{
	uint8_t data[6];
	uint8_t myreg[1] = {MPU6050_GYRO_XOUT_H};
	/* Read gyroscope data */
	
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, myreg, 1, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, data, 6, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
		
	/* Format */
	DataStruct->Gyro_X_RAW  = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyro_Y_RAW  = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyro_Z_RAW  = (int16_t)(data[4] << 8 | data[5]);

	 /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW * DataStruct->Gyro_Mult;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW * DataStruct->Gyro_Mult;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW * DataStruct->Gyro_Mult;
	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadTemperature(MPU6050_t* DataStruct) 
{
	uint8_t data[2];
	int16_t temp;
	uint8_t myreg[1] = {MPU6050_TEMP_OUT_H};
	/* Read temperature */
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, myreg, 1, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, data, 2, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
		
	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result_t MPU6050_ReadAll(MPU6050_t* DataStruct) {
	uint8_t data[14];
	int16_t temp;
	uint8_t myreg[1] = {MPU6050_ACCEL_XOUT_H};
	/* Read full raw data, 14bytes */

	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DataStruct->Address, myreg, 1, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DataStruct->Address, data, 14, 1000) != HAL_OK) 
	{
		return MPU6050_Result_Error;
	}
		
	/* Format accelerometer data */
	DataStruct->Accel_X_RAW = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accel_Y_RAW = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accel_Z_RAW = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	/* Format gyroscope data */
	DataStruct->Gyro_X_RAW  = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyro_Y_RAW  = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyro_Z_RAW  = (int16_t)(data[12] << 8 | data[13]);

	DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

	// Kalman angle solve
    double dt = (double) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);

	/* Return OK */
	return MPU6050_Result_Ok;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) 
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}
