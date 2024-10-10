/*
 * IMU.c
 *
 *  Created on: Sep 11, 2024
 *      Author: PC_6285_YÄ°
 */

//merhababenguven

//ayse

#include <IMU2.h>
#include "MPU9250/MPU9250.h"
extern I2C_HandleTypeDef hi2c5;


mpudata_t mpu_st = {0};
mpudatag_t mpug_st = {0};
mpudatam_t mpum_st = {0};


void IMU2_Init()
{
	  initMPU9250(&hi2c5);
	  initAK8963(&hi2c5);
	  MPU9255_Init(&hi2c5);
}
void IMU2_Get_Data(IMU *IMU_t)
{
	  MPU9250_Read_Raw_acc(&mpu_st);
	  MPU9250_Read_Raw_gyro(&mpug_st);
	  MPU9250_Read_Raw_mag(&mpum_st);

      IMU_t -> Ivmeolcer_t.x_ekseni = mpu_st.Accel_X_RAW;
      IMU_t -> Ivmeolcer_t.y_ekseni = mpu_st.Accel_Y_RAW;
      IMU_t -> Ivmeolcer_t.z_ekseni = mpu_st.Accel_Z_RAW;
      IMU_t -> Aciolcer_t.x_ekseni  = mpug_st.Gyro_X_RAW;
      IMU_t -> Aciolcer_t.y_ekseni  = mpug_st.Gyro_Y_RAW;
      IMU_t -> Aciolcer_t.z_ekseni  = mpug_st.Gyro_Z_RAW;
      IMU_t -> Manyetometre_t.x_ekseni = mpum_st.Magneto_X_RAW;
      IMU_t -> Manyetometre_t.y_ekseni = mpum_st.Magneto_Y_RAW;
      IMU_t -> Manyetometre_t.z_ekseni = mpum_st.Magneto_Z_RAW;

}
