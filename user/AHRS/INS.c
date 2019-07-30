#include "INS.h"

#define MPU6500_DATA_READY_EXIT_INIT() GPIOB_Exti8_GPIO_Init() //初始化mpu6500的 外部中断 使用PB8 外部中断线 8

#define MPU6500_DATA_READY_EXIT_IRQHandler EXTI9_5_IRQHandler //宏定义外部中断函数，使用了line8外部中断

#define MPU6500_DATA_READY_EXIT_Line EXTI_Line8 //宏定义外部中断线

static uint8_t mpu6500_spi_rxbuf[23];
static mpu6500_real_data_t mpu6500_real_data;
static ist8310_real_data_t ist8310_real_data;

static fp32 Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度
static fp32 gyro_cali_offset[3] = {0.0f, 0.0f, 0.0f};
static fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};						//陀螺仪零漂
static fp32 Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
static fp32 Accel_Offset[3] = {0.0f, 0.0f, 0.0f};						//加速度零漂
static ist8310_real_data_t ist8310_real_data;							//转换成国际单位的IST8310数据
static fp32 Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
									  {0.0f, 1.0f, 0.0f},
									  {0.0f, 0.0f, 1.0f}}; //磁力计校准线性度
static fp32 Mag_Offset[3] = {0.0f, 0.0f, 0.0f};			   //磁力计零漂

static const float TimingTime = 0.001f; //任务运行的时间 单位 s

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

static fp32 INS_Angle[3] = {0.0f, 0.0f, 0.0f};		//欧拉角 单位 rad
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数

void INS_Init(void)
{

	while (mpu6500_init() != MPU6500_NO_ERROR)
	{
		;
	}

	while (ist8310_init() != IST8310_NO_ERROR)
	{
		;
	}

	//初始化mpu6500的数据准备的外部中断
	MPU6500_DATA_READY_EXIT_INIT();
}

void MPU6500_DATA_READY_EXIT_IRQHandler(void)
{
	/***************************************************/
	/* from switch.c */
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		EXTI_ClearFlag(EXTI_Line5);
		if (Switch[SWITCH_NAME_FRONT].exit_func != NULL)
			(*Switch[SWITCH_NAME_FRONT].exit_func)();
	}

	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		EXTI_ClearFlag(EXTI_Line6);
		if (Switch[SWITCH_NAME_REAR].exit_func != NULL)
			(*Switch[SWITCH_NAME_REAR].exit_func)();
	}
	/***************************************************/

	if (EXTI_GetITStatus(MPU6500_DATA_READY_EXIT_Line) != RESET)
	{

		EXTI_ClearITPendingBit(MPU6500_DATA_READY_EXIT_Line);

		mpu6500_SPI_NS_L();

		mpu6500_read_muli_reg(MPU_INT_STATUS, mpu6500_spi_rxbuf, 23);

		mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET), &mpu6500_real_data);

		ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET), &ist8310_real_data);
	}
}

static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310)
{
	for (uint8_t i = 0; i < 3; i++)
	{
		gyro[i] = mpu6500->gyro[0] * Gyro_Scale_Factor[i][0] + mpu6500->gyro[1] * Gyro_Scale_Factor[i][1] + mpu6500->gyro[2] * Gyro_Scale_Factor[i][2] + Gyro_Offset[i];
		accel[i] = mpu6500->accel[0] * Accel_Scale_Factor[i][0] + mpu6500->accel[1] * Accel_Scale_Factor[i][1] + mpu6500->accel[2] * Accel_Scale_Factor[i][2] + Accel_Offset[i];
		mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2] + Mag_Offset[i];
	}
}

const fp32 *get_INS_angle_point(void)
{
	return INS_Angle;
}

const fp32 *get_MPU6500_Gyro_Data_Point(void)
{
	return INS_gyro;
}

const fp32 *get_MPU6500_Accel_Data_Point(void)
{
	return INS_accel;
}

void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
	gyro_cali_offset[0] = cali_offset[0];
	gyro_cali_offset[1] = cali_offset[1];
	gyro_cali_offset[2] = cali_offset[2];
}

uint8_t INS_Cali(void)
{
	static uint16_t start_gyro_cali_time = 0;
	if (start_gyro_cali_time == 0)
	{
		Gyro_Offset[0] = gyro_cali_offset[0];
		Gyro_Offset[1] = gyro_cali_offset[1];
		Gyro_Offset[2] = gyro_cali_offset[2];
		start_gyro_cali_time++;
		return 0;
	}
	else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
	{
		gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
		return 0;
	}
	else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
	{
		start_gyro_cali_time++;
	}
	return 1;
}

void INS_Calc(void)
{
	static uint8_t updata_count = 0;
	static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
	static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
	static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
	static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

	IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);

	if (updata_count == 0)
	{
		//初始化四元数
		AHRS_init(INS_quat, INS_accel, INS_mag);
		get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

		accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
		accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
		accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
		updata_count++;
	}
	else
	{
		//加速度计低通滤波
		accel_fliter_1[0] = accel_fliter_2[0];
		accel_fliter_2[0] = accel_fliter_3[0];

		accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

		accel_fliter_1[1] = accel_fliter_2[1];
		accel_fliter_2[1] = accel_fliter_3[1];

		accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

		accel_fliter_1[2] = accel_fliter_2[2];
		accel_fliter_2[2] = accel_fliter_3[2];

		accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

		//更新四元数
		AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);
		get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);
	}
	INS_Cali();
}
