/*
 * IMU.h
 *
 *  Created on: Sep 11, 2024
 *      Author: PC_6285_YÄ°
 */

#ifndef IMU_IMU_H_
#define IMU_IMU_H_
#include "stdint.h"

typedef struct{
float x_ekseni;
float y_ekseni;
float z_ekseni;
}Ivmeolcer;

typedef struct{
float x_ekseni;
float y_ekseni;
float z_ekseni;
}Aciolcer;

typedef struct{
float x_ekseni;
float y_ekseni;
float z_ekseni;
}Manyetometre;

typedef struct{
Ivmeolcer Ivmeolcer_t;
Aciolcer Aciolcer_t;
Manyetometre Manyetometre_t;
}IMU;

void IMU2_Init();
void IMU2_Get_Data(IMU *IMU_t);
#endif /* IMU_IMU_H_ */
