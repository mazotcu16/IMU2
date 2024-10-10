/*
 * 9250_sensor.h
 *
 *  Created on: Oct 7, 2024
 *      Author: PC_3884
 */


#include "9250_sensorDEF.h"
#include "stm32h7xx_hal.h"
extern UART_HandleTypeDef huart3;


#ifndef INC_9250_SENSOR_H_
#define INC_9250_SENSOR_H_

#define WHO_AM_I_9250_ANS 0x71
#define WHO_AM_I          0x75
#define AD0_LOW           0x68
#define AD0_HIGH          0x69
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B
#define I2C_TIMOUT_MS     1000
#define REG_USR_CTRL      107
#define REG_DATA          59
#define REGg_DATA		  67
#define MPU9250_INT_PIN   55

#define WINDOW_SIZE 10  // Hareketli ortalama penceresinin boyutu

extern float accelXBuffer[WINDOW_SIZE];
extern float accelYBuffer[WINDOW_SIZE];
extern float accelZBuffer[WINDOW_SIZE];

extern int accelXIndex;
extern int accelYIndex;
extern int accelZIndex;

extern int accelXCount;
extern int accelYCount;
extern int accelZCount;

float applyMovingAverageFilter(float newValue, float* dataBuffer, int* index, int* count);



enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0,
  MFS_16BITS
};


typedef struct {
	float	Accel_X_RAW;
	float	Accel_Y_RAW;
	float	Accel_Z_RAW;

}mpudata_t;

typedef struct {
	float	Gyro_X_RAW;
	float	Gyro_Y_RAW;
	float	Gyro_Z_RAW;
}mpudatag_t;

typedef struct {
	float	Magneto_X_RAW;
	float	Magneto_Y_RAW;
	float	Magneto_Z_RAW;
}mpudatam_t;

typedef struct {
	float temp_raw;
	float temp_o;

}temp_t;




void initMPU9250(I2C_HandleTypeDef *I2Cx);
void initAK8963(I2C_HandleTypeDef *I2Cx);
uint8_t MPU9255_Init(I2C_HandleTypeDef *hi2c5);
//void MPU9250SelfTest(I2C_HandleTypeDef *I2Cx, float * destination);
void getAres();
void getGres();
void getMres();
void MPU9250_Read_Raw_acc(mpudata_t * mpudata_st);
void MPU9250_Read_Raw_gyro(mpudatag_t * mpudatag_st);
void MPU9250_Read_Raw_mag(mpudatam_t * mpudatam_st);
void readTemperature();
void sendSensorData(mpudata_t *mpudata, mpudatag_t *mpudatag);
//void calibrateMPU9250(I2C_HandleTypeDef *hi2c5, float * dest1, float * dest2);
//void MPU9250_init(void);

#endif /* INC_9250_SENSOR_H_ */
