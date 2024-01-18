#ifndef _IMU_H_
#define	_IMU_H_

#include "math.h"

#define GYRO_TO_RAD(x) ((float)x*0.0001331580545) //gyro*250/2^15/57.3
#define GYRO_TO_DEG(x) ((float)x*0.0076293945) //gyro*250/2^15

u8 Acc_Get_ErrData(short ax,short ay,short az,u8 enable);
u8 Gyro_Get_ErrData(short gx,short gy,short gz,u8 enable);
u8 Baro_Get_ErrData(float altitude,u8 enable);
void Acc_Calibrate(short *ax,short *ay,short *az);
void IMUupdate(short *gx,short *gy,short *gz,short ax,short ay,short az,float *roll,float *pitch,float *yaw);

#endif
