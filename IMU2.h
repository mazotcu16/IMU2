/*
 * IMU.h
 *
 *  Created on: Sep 11, 2024
 *      Author: PC_6285_YÄ°
 */

#ifndef IMU2_IMU2_H_
#define IMU2_IMU2_H_
#include "stdint.h"

typedef struct{
float x_ekseni;
float y_ekseni;
float z_ekseni;
}Ivmeolcer2;

typedef struct{
float x_ekseni;
float y_ekseni;
float z_ekseni;
}Aciolcer2;

typedef struct{
float x_ekseni;
float y_ekseni;
float z_ekseni;
}Manyetometre2;

typedef struct{
Ivmeolcer2 Ivmeolcer_t;
Aciolcer2 Aciolcer_t;
Manyetometre2 Manyetometre_t;
}IMU2;

void IMU2_Init();
void IMU2_Get_Data(IMU2 *IMU_t);
#endif /* IMU_IMU_H_ */
