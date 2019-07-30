#ifndef INS_Task_H
#define INS_Task_H

#include "main.h"
#include "AHRS.h"
#include "IST8310driver.h"
#include "mpu6500driver.h"
#include "mpu6500reg.h"
#include "mpu6500driver_middleware.h"

#define MPU6500_RX_BUF_DATA_OFFSET 0

#define IST8310_RX_BUF_DATA_OFFSET 15

#define IMU_BOARD_INSTALL_SPIN_MATRIX \
	{0.0f, 1.0f, 0.0f},               \
		{-1.0f, 0.0f, 0.0f},          \
	{                                 \
		0.0f, 0.0f, 1.0f              \
	}

extern void INS_Init(void);
extern void INS_Calc(void);
extern uint8_t INS_Cali(void);

extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);

#endif
