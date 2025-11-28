#include "MPU6500.h"

float dt;
int start_time;
int end_time;
int timer_PSC;
int timer_clock;

struct MPU6500_Offset offset_accel = {
    .offset_x = 0.0f,
    .offset_y = 0.0f,
    .offset_z = 0.0f
};

float beta = betaDef;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/*==============================================================================================*/
/*========================================定时器相关函数========================================*/ 
/*==============================================================================================*/
void Get_dt_timer_start(struct MPU6500_Data *mpu6500_data)
{
    HAL_TIM_Base_Start(mpu6500_data->htim);
}

void Get_dt_timer_stop(struct MPU6500_Data *mpu6500_data)
{
    HAL_TIM_Base_Stop(mpu6500_data->htim);
}

void Get_dt_timer_reset(struct MPU6500_Data *mpu6500_data)
{
    __HAL_TIM_SET_COUNTER(mpu6500_data->htim, 0);
}

int Get_dt_timer_get(struct MPU6500_Data *mpu6500_data)
{
    return __HAL_TIM_GET_COUNTER(mpu6500_data->htim);
}

void Get_dt_timer_Init(void)
{
    timer_PSC = TIM3->PSC + 1;
    timer_clock = HAL_RCC_GetSysClockFreq();
}

/*==============================================================================================*/
/*=========================================I2C相关函数==========================================*/ 
/*==============================================================================================*/

HAL_StatusTypeDef MPU6500_Transmit(struct MPU6500_Data *mpu6500_data,uint8_t reg, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef res;
    res = HAL_I2C_Mem_Write(mpu6500_data->hi2c, mpu6500_data->mpu6500_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
    return res;
}

HAL_StatusTypeDef MPU6500_Receive(struct MPU6500_Data *mpu6500_data, uint8_t reg, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef res;
    res = HAL_I2C_Mem_Read(mpu6500_data->hi2c, mpu6500_data->mpu6500_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
    return res;
}

/*==============================================================================================*/
/*=========================================Madgwick方法==========================================*/ 
/*==============================================================================================*/

/*
 * name:快速计算模值倒数函数
 * func:快速计算模值倒数
 * in:x:数据
 * out:x的模值倒数
 * PS:None
 *
 * */
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*
 * name:四元数转化欧拉角函数
 * func:将四元数转化为欧拉角
 * in:angle[3]:角度的x,y,z轴
 * out:void
 * PS:None
 *
 * */
void computeAngles(float angle[3])
{
    const float rad2deg = 57.295779513f; // 180/PI
    
    // Pitch (X-axis rotation)
    angle[0] = asinf(-2.0f * (q1*q3 - q0*q2)) * rad2deg;
    
    // Roll (Y-axis rotation)
    angle[1] = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * rad2deg;
    
    // Yaw (Z-axis rotation) - 使用标准公式避免跳变
    angle[2] = atan2f(2.0f*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * rad2deg;
    
    // 将Yaw角归一化到0-360度范围
    if(angle[2] < 0) angle[2] += 360.0f;
}

/*
 * name:3位归一化函数
 * func:归一化
 * in:*x,*y,*z:数据的指针
 * out:void
 * PS:None
 *
 * */
void Norm_Data(float *x,float *y,float *z)
{
	float recipNorm;

	recipNorm = invSqrt(*x * *x + *y * *y + *z * *z);
	*x *= recipNorm;
	*y *= recipNorm;
	*z *= recipNorm;
}

/*
 * name:4位归一化函数
 * func:归一化四元数
 * in:*w,*x,*y,*z:数据的指针
 * out:void
 * PS:None
 *
 * */
void Norm_Data_Q(float *w,float *x,float *y,float *z)
{
	float recipNorm;

	recipNorm = invSqrt(*w * *w + *x * *x + *y * *y + *z * *z);
	*w *= recipNorm;
	*x *= recipNorm;
	*y *= recipNorm;
	*z *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

/*
 * name:无磁力计Madgwick方法
 * func:无磁力计下的Madgwick方法,用于计算姿态解算
 * in:acc[3]:角速度的x,y,z轴	gyro[3]:角速度的x,y,z轴
 * out:void
 * PS:未使用该方法的yaw轴
 *
 * */
void MadgwickAHRSupdateIMU(struct MPU6500_Data *mpu6500_data, float acc[3],float gyro[3])
{
	float gx=gyro[0];
	float gy=gyro[1];
	float gz=gyro[2];
	float ax=acc[0];
	float ay=acc[1];
	float az=acc[2];

	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		Norm_Data(&ax,&ay,&az);

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		Norm_Data_Q(&s0,&s1,&s2,&s3);

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

    end_time = Get_dt_timer_get(mpu6500_data);
    Get_dt_timer_stop(mpu6500_data);
    Get_dt_timer_reset(mpu6500_data);

    dt = (end_time - start_time)  * timer_PSC / timer_clock;

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

    Get_dt_timer_start(mpu6500_data);
    start_time = Get_dt_timer_get(mpu6500_data);

	// Normalise quaternion
	Norm_Data_Q(&q0,&q1,&q2,&q3);
}

/*
 * name:Madgwick方法
 * func:有磁力计下的Madgwick方法,用于计算姿态解算
 * in:acc[3]:角速度的x,y,z轴	gyro[3]:角速度的x,y,z轴	mag[3]:磁力计的x,y,z轴
 * out:void
 * PS:未使用该方法的yaw轴
 *
 * */
void MadgwickAHRSupdate(struct MPU6500_Data *mpu6500_data, float acc[3],float gyro[3],float mag[3])
{
	float gx=gyro[0];
	float gy=gyro[1];
	float gz=gyro[2];
	float ax=acc[0];
	float ay=acc[1];
	float az=acc[2];
	float mx=mag[0];
	float my=mag[1];
	float mz=mag[2];

	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(mpu6500_data, acc, gyro);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		Norm_Data(&ax,&ay,&az);

		// Normalise magnetometer measurement
		Norm_Data(&mx,&my,&mz);

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		Norm_Data_Q(&s0,&s1,&s2,&s3);

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

    end_time = Get_dt_timer_get(mpu6500_data);
    Get_dt_timer_stop(mpu6500_data);
    Get_dt_timer_reset(mpu6500_data);

    dt = (float)((end_time - start_time) * timer_PSC) / timer_clock;

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

    Get_dt_timer_start(mpu6500_data);
    start_time = Get_dt_timer_get(mpu6500_data);

	// Normalise quaternion
	Norm_Data_Q(&q0,&q1,&q2,&q3);
}

/*==============================================================================================*/
/*===========================================辅助函数===========================================*/ 
/*==============================================================================================*/

/*brief  获取Simple_Div值
 * @param  odr: 陀螺仪输出数据速率
 * @retval Simple_Div值
 */
uint8_t MPU6500_Get_SimpleDiv(uint16_t odr)
{
    int gyroFs;
    int accelRate;
    uint8_t simpleDiv;

    if(MPU6500_REG_GYRO_CONFIG_FCHOICE_B != 0)
        gyroFs = 32000;
    else if(MPU6500_REG_CONFIG_DLPF_CFG == 0 && MPU6500_REG_CONFIG_DLPF_CFG == 7)
        gyroFs = 8000;
    else 
        gyroFs = 1000;

    if(MPU6500_REG_ACCEL_CONFIG_2_ACCEL_FCHOICE_B != 0)
        accelRate = 4000;
    else 
        accelRate = 1000;

    if(accelRate >= gyroFs)
        simpleDiv = (uint8_t)((gyroFs / odr) - 1);
    else
        simpleDiv = (uint8_t)((accelRate / odr) - 1);

    if(simpleDiv > 255) simpleDiv = 255;

    return simpleDiv;
}

/*brief  MPU6500 FSYNC配置
 * @param  mpu6500_fsync_config: MPU6500 FSYNC配置结构体指针
 * @retval None
 */
void MPU6500_FSYNC_Config(struct MPU6500_FSYNC_Config *mpu6500_fsync_config)
{
    switch(MPU6500_FSYNC_MODE)
    {
        case 0: //关闭FSYNC
            mpu6500_fsync_config->config_ext_sync_set = 0x00;
            mpu6500_fsync_config->int_pin_cfg_actl_fsync = 0x00;
            mpu6500_fsync_config->int_pin_cfg_fsync_int_mode_en = 0x00;
            break;
        case 1: //数据同步模式
            mpu6500_fsync_config->config_ext_sync_set = MPU6500_FSYNC_SYNC_MODE;
            mpu6500_fsync_config->int_pin_cfg_actl_fsync = 0x00;
            mpu6500_fsync_config->int_pin_cfg_fsync_int_mode_en = 0x00;
            break;
        case 2: //中断模式
            mpu6500_fsync_config->config_ext_sync_set = 0x00;
            mpu6500_fsync_config->int_pin_cfg_actl_fsync = MPU6500_FSYNC_INT_MODE_ACTL << 3;
            mpu6500_fsync_config->int_pin_cfg_fsync_int_mode_en = 0x04;
            break;
    }
}

/*brief  配置陀螺仪偏置寄存器
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU6500_GyroOffset_Config(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data[2] = {0};
    HAL_StatusTypeDef res;
    struct MPU6500_Data add_mpu6500_data = {0};
    struct MPU6500_Offset offset_gyro = {0};
    int checkTime = 5;

    HAL_Delay(1000);

    for(int i = 0; i < checkTime; i++)
    {
        MPU6500_Read_Gyro(mpu6500_data);
        add_mpu6500_data.gyro_x += mpu6500_data->gyro_x;
        add_mpu6500_data.gyro_y += mpu6500_data->gyro_y;
        add_mpu6500_data.gyro_z += mpu6500_data->gyro_z;
        HAL_Delay(1000 / MPU6500_ACCEL_ODR);
    }
    
    offset_gyro.offset_x = (0 - add_mpu6500_data.gyro_x / checkTime * MPU6500_GYRO_SENSITIVITY) / 4;
    offset_gyro.offset_y = (0 - add_mpu6500_data.gyro_y / checkTime * MPU6500_GYRO_SENSITIVITY) / 4;
    offset_gyro.offset_z = (0 - add_mpu6500_data.gyro_z / checkTime * MPU6500_GYRO_SENSITIVITY) / 4;

    data[0] = ((int16_t)offset_gyro.offset_x >> 8) & 0xFF; // 高8位
    data[1] = (int16_t)offset_gyro.offset_x & 0xFF;        // 低8位
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_XG_OFFSET_H, data, 2);
    if (res != HAL_OK) return res;
    data[0] = ((int16_t)offset_gyro.offset_y >> 8) & 0xFF;
    data[1] = (int16_t)offset_gyro.offset_y & 0xFF;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_YG_OFFSET_H, data, 2);
    if (res != HAL_OK) return res;
    data[0] = ((int16_t)offset_gyro.offset_z >> 8) & 0xFF;
    data[1] = (int16_t)offset_gyro.offset_z & 0xFF;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_ZG_OFFSET_H, data, 2);
    if (res != HAL_OK) return res;

    return HAL_OK;
}

/*brief  重置FIFO
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU6500_FIFO_RST(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data;
    HAL_StatusTypeDef res;

    //重置FIFO
    data = 0x04;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_USER_CTRL, &data, 1);

    return res;
}

/*brief  读取FIFO计数值
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Read_FIFO_Count(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data[2];
    HAL_StatusTypeDef res;

    res = MPU6500_Receive(mpu6500_data, MPU6500_REG_FIFO_COUNTH, data, 2);
    if (res != HAL_OK) return MPU6500_ERROR;

    mpu6500_data->fifo_count = (uint16_t)(data[0] << 8 | data[1]);

    mpu6500_data->fifo_count = (uint16_t)(mpu6500_data->fifo_count / MPU6500_FIFO_READ_GROUP_SIZE) * MPU6500_FIFO_READ_GROUP_SIZE;

    return MPU6500_OK;
}

/*==============================================================================================*/
/*==========================================主要函数============================================*/ 
/*==============================================================================================*/

/**
 * @brief  MPU6500初始化
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Init(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data;
    HAL_StatusTypeDef res;
    uint8_t simpleDiv;
    struct MPU6500_FSYNC_Config mpu6500_fsync_config;

    //获取Simple_Div
    simpleDiv = MPU6500_Get_SimpleDiv(MPU6500_GYRO_ODR);
    //FSYNC配置
    MPU6500_FSYNC_Config(&mpu6500_fsync_config);

    HAL_Delay(100);

    //重启MPU6500
    data = 0x80;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_PWR_MGMT_1, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    HAL_Delay(100);

    //唤醒MPU6500,找到最佳时钟源
    data = 0x01;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_PWR_MGMT_1, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //读取MPU6500的ID
    res = MPU6500_Receive(mpu6500_data, MPU6500_REG_WHO_AM_I, &data, 1);
    if(res != HAL_OK) return MPU6500_ERROR;
    if(data != 0x70) return MPU6500_ERROR_ID;

    //重置FIFO
    if(MPU6500_FIFO_ENABLE == 1)
    {
        res = MPU6500_FIFO_RST(mpu6500_data);
        if (res != HAL_OK) return MPU6500_ERROR;
        HAL_Delay(100);

        data = 0x40;
        res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_USER_CTRL, &data, 1);
        if (res != HAL_OK) return MPU6500_ERROR;
        HAL_Delay(100);

        data = MPU6500_FIFO_ENABLE_TEMP_OUT << 7 | MPU6500_FIFO_ENABLE_GYRO_XOUT << 6 | MPU6500_FIFO_ENABLE_GYRO_YOUT << 5 | MPU6500_FIFO_ENABLE_GYRO_ZOUT << 4 | MPU6500_FIFO_ENABLE_ACCEL_OUT << 3;
        res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_FIFO_EN, &data, 1);
        if (res != HAL_OK) return MPU6500_ERROR;
    }

    //配置采样率分频器
    data = simpleDiv;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_SMPLRT_DIV, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //CONFIG
    data = MPU6500_FIFO_MODE << 6 | mpu6500_fsync_config.config_ext_sync_set | MPU6500_REG_CONFIG_DLPF_CFG;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_CONFIG, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //Gyroscope Configuration
    data = MPU6500_REG_GYRO_CONFIG_FS_SEL | MPU6500_REG_GYRO_CONFIG_FCHOICE_B;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_GYRO_CONFIG, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //Accelerometer Configuration
    data = MPU6500_REG_ACCEL_CONFIG_AFS_SEL;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_ACCEL_CONFIG, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //Accelerometer Configuration 2
    data = MPU6500_REG_ACCEL_CONFIG_2_ACCEL_FCHOICE_B | MPU6500_REG_ACCEL_CONFIG_2_A_DLPF_CFG;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_ACCEL_CONFIG_2, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //配置偏置寄存器
    res = MPU6500_GyroOffset_Config(mpu6500_data);
    if(res != HAL_OK) return MPU6500_ERROR;

    //INT_PIN_CFG
    data = MPU6500_INT_PIN_CFG_ACTL << 7 | MPU6500_INT_PIN_CFG_OPEN << 6 | MPU6500_INT_PIN_CFG_LATCH_INT_EN << 5 | MPU6500_INT_PIN_CFG_INT_ANYRD_2CLEAR << 4 | mpu6500_fsync_config.int_pin_cfg_actl_fsync | mpu6500_fsync_config.int_pin_cfg_fsync_int_mode_en;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_INT_PIN_CFG, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //INT_ENABLE
    data = MPU6500_INT_ENABLE_WOM_EN << 6 | MPU6500_INT_ENABLE_FIFO_OVERFLOW_EN << 4 | MPU6500_INT_ENABLE_FSYNC_INT_EN << 3 | MPU6500_INT_ENABLE_RAW_RDY_EN;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_INT_ENABLE, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //初始化定时器
    Get_dt_timer_Init();

    return MPU6500_OK;
}

/*brief  MPU6500全功耗模式配置
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Full_Power_Mode(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data;
    HAL_StatusTypeDef res;

    struct MPU6500_FSYNC_Config mpu6500_fsync_config;
    MPU6500_FSYNC_Config(&mpu6500_fsync_config);

    MPU6500_Exit_Weak_en_Motion(mpu6500_data);

    //唤醒MPU6500,找到最佳时钟源
    data = 0x01;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_PWR_MGMT_1, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    data = 0x00;        // 开启三轴陀螺仪
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_PWR_MGMT_2, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //CONFIG
    data = MPU6500_FIFO_MODE << 6 | mpu6500_fsync_config.config_ext_sync_set | MPU6500_REG_CONFIG_DLPF_CFG;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_CONFIG, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //Gyroscope Configuration
    data = MPU6500_REG_GYRO_CONFIG_FS_SEL | MPU6500_REG_GYRO_CONFIG_FCHOICE_B;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_GYRO_CONFIG, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //Accelerometer Configuration
    data = MPU6500_REG_ACCEL_CONFIG_AFS_SEL;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_ACCEL_CONFIG, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //Accelerometer Configuration 2
    data = MPU6500_REG_ACCEL_CONFIG_2_ACCEL_FCHOICE_B | MPU6500_REG_ACCEL_CONFIG_2_A_DLPF_CFG;
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_ACCEL_CONFIG_2, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    return MPU6500_OK;
}

/*brief  MPU6500低功耗模式配置
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Low_Power_Mode(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data;
    HAL_StatusTypeDef res;

    MPU6500_Enter_Weak_en_Motion(mpu6500_data);

    // 进入低功耗模式
    data = 0x08;        // 低功耗模式下加速度计输出频率为3.91Hz
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_LP_ACCEL_ODR, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    data = 0x28;        // 开启循环模式,禁用温度传感器
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_PWR_MGMT_1, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    data = 0x07;        // 关闭三轴陀螺仪
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_PWR_MGMT_2, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    return MPU6500_OK;
}

/*brief  使能运动唤醒功能
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Enter_Weak_en_Motion(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data;
    HAL_StatusTypeDef res;

    //Accelerometer Configuration 2
    data = 0x02;        // 加速度计低通滤波器设置为94Hz
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_ACCEL_CONFIG_2, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    data = 0x96;        // 运动唤醒阈值设置为4*150=600mg
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_WOM_THR, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    data = 0xC0;        // 开启运动唤醒中断
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_ACCEL_INTEL_CTRL, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //INT_ENABLE
    data = MPU6500_INT_ENABLE_WOM_EN << 6;      // 只开启运动唤醒中断
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_INT_ENABLE, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    return MPU6500_OK;
}

/*brief  关闭运动唤醒功能
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Exit_Weak_en_Motion(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data;
    HAL_StatusTypeDef res;

    data = 0x00;        // 关闭运动唤醒中断
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_ACCEL_INTEL_CTRL, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    //INT_ENABLE
    data = MPU6500_INT_ENABLE_FIFO_OVERFLOW_EN << 4 | MPU6500_INT_ENABLE_FSYNC_INT_EN << 3 | MPU6500_INT_ENABLE_RAW_RDY_EN;      // 开启除运动唤醒中断外的所有允许配置的中断
    res = MPU6500_Transmit(mpu6500_data, MPU6500_REG_INT_ENABLE, &data, 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    return MPU6500_OK;
}

/*brief  读取加速度数据
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Read_Accel(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data[6];
    HAL_StatusTypeDef res;

    res = MPU6500_Receive(mpu6500_data, MPU6500_REG_ACCEL_XOUT_H, data, 6);
    if (res != HAL_OK) return MPU6500_ERROR;

    mpu6500_data->accel_x = (int16_t)(data[0] << 8 | data[1]) / MPU6500_ACCEL_SENSITIVITY * G;
    mpu6500_data->accel_y = (int16_t)(data[2] << 8 | data[3]) / MPU6500_ACCEL_SENSITIVITY * G;
    mpu6500_data->accel_z = (int16_t)(data[4] << 8 | data[5]) / MPU6500_ACCEL_SENSITIVITY * G;

    lowPassFilter_accel(mpu6500_data);

    return MPU6500_OK;
}

/*brief  读取角速度数据
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Read_Gyro(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data[6];
    HAL_StatusTypeDef res;

    res = MPU6500_Receive(mpu6500_data, MPU6500_REG_GYRO_XOUT_H, data, 6);
    if (res != HAL_OK) return MPU6500_ERROR;

    mpu6500_data->gyro_x = (int16_t)(data[0] << 8 | data[1]) / MPU6500_GYRO_SENSITIVITY;
    mpu6500_data->gyro_y = (int16_t)(data[2] << 8 | data[3]) / MPU6500_GYRO_SENSITIVITY;
    mpu6500_data->gyro_z = (int16_t)(data[4] << 8 | data[5]) / MPU6500_GYRO_SENSITIVITY;

    lowPassFilter_gyro(mpu6500_data); 

    return MPU6500_OK;
}

/*brief  读取温度数据
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Read_Temp(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data[2];
    HAL_StatusTypeDef res;

    res = MPU6500_Receive(mpu6500_data, MPU6500_REG_TEMP_OUT_H, data, 2);
    if (res != HAL_OK) return MPU6500_ERROR;

    mpu6500_data->temp = (int16_t)(data[0] << 8 | data[1]) / 340.0 + 36.53;

    return MPU6500_OK;
}

/*brief  读取中断状态寄存器
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Read_INT_Status(struct MPU6500_Data *mpu6500_data)
{
    HAL_StatusTypeDef res;

    res = MPU6500_Receive(mpu6500_data, MPU6500_REG_INT_STATUS, &(mpu6500_data->int_status), 1);
    if (res != HAL_OK) return MPU6500_ERROR;

    return MPU6500_OK;
}



/*brief  读取FIFO数据
 * @param  mpu6500_data: MPU6500数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t MPU6500_Read_FIFO(struct MPU6500_Data *mpu6500_data)
{
    uint8_t data[512] = {0};
    HAL_StatusTypeDef res;
    struct MPU6500_Data temp_mpu6500_data;
    struct MPU6500_Data add_temp_mpu6500_data;

    MPU6500_Read_FIFO_Count(mpu6500_data);

    res = MPU6500_Receive(mpu6500_data, MPU6500_REG_FIFO_R_W, data, mpu6500_data->fifo_count);
    if (res != HAL_OK) return MPU6500_ERROR;

    //解析FIFO数据
    for(int i = 0; i < mpu6500_data->fifo_count / MPU6500_FIFO_READ_GROUP_SIZE; i++)
    {
        int index = i * MPU6500_FIFO_READ_GROUP_SIZE;
        if(MPU6500_FIFO_ENABLE_ACCEL_OUT == 1)
        {
            temp_mpu6500_data.accel_x = (int16_t)(data[index] << 8 | data[index + 1]);
            temp_mpu6500_data.accel_y = (int16_t)(data[index + 2] << 8 | data[index + 3]);
            temp_mpu6500_data.accel_z = (int16_t)(data[index + 4] << 8 | data[index + 5]);
            index += 6;
            add_temp_mpu6500_data.accel_x += temp_mpu6500_data.accel_x;
            add_temp_mpu6500_data.accel_y += temp_mpu6500_data.accel_y;
            add_temp_mpu6500_data.accel_z += temp_mpu6500_data.accel_z;
        }
        if(MPU6500_FIFO_ENABLE_TEMP_OUT == 1)
        {
            temp_mpu6500_data.temp = (int16_t)(data[index] << 8 | data[index + 1]);
            index += 2;
            add_temp_mpu6500_data.temp += temp_mpu6500_data.temp;
        }
        if(MPU6500_FIFO_ENABLE_GYRO_XOUT == 1)
        {
            temp_mpu6500_data.gyro_x = (int16_t)(data[index] << 8 | data[index + 1]);
            index += 2;
            add_temp_mpu6500_data.gyro_x += temp_mpu6500_data.gyro_x;
        }
        if(MPU6500_FIFO_ENABLE_GYRO_YOUT == 1)
        {
            temp_mpu6500_data.gyro_y = (int16_t)(data[index] << 8 | data[index + 1]);
            index += 2;
            add_temp_mpu6500_data.gyro_y += temp_mpu6500_data.gyro_y;
        }
        if(MPU6500_FIFO_ENABLE_GYRO_ZOUT == 1)
        {
            temp_mpu6500_data.gyro_z = (int16_t)(data[index] << 8 | data[index + 1]);
            index += 2;
            add_temp_mpu6500_data.gyro_z += temp_mpu6500_data.gyro_z;
        }
    }

    if(MPU6500_FIFO_ENABLE_ACCEL_OUT == 1)
    {
        mpu6500_data->accel_x = add_temp_mpu6500_data.accel_x / mpu6500_data->fifo_count * MPU6500_FIFO_READ_GROUP_SIZE;
        mpu6500_data->accel_y = add_temp_mpu6500_data.accel_y / mpu6500_data->fifo_count * MPU6500_FIFO_READ_GROUP_SIZE;
        mpu6500_data->accel_z = add_temp_mpu6500_data.accel_z / mpu6500_data->fifo_count * MPU6500_FIFO_READ_GROUP_SIZE;
    }
    if(MPU6500_FIFO_ENABLE_TEMP_OUT == 1)
        mpu6500_data->temp = add_temp_mpu6500_data.temp / mpu6500_data->fifo_count * MPU6500_FIFO_READ_GROUP_SIZE;
    if(MPU6500_FIFO_ENABLE_GYRO_XOUT == 1)
        mpu6500_data->gyro_x = add_temp_mpu6500_data.gyro_x / mpu6500_data->fifo_count * MPU6500_FIFO_READ_GROUP_SIZE;
    if(MPU6500_FIFO_ENABLE_GYRO_YOUT == 1)
        mpu6500_data->gyro_y = add_temp_mpu6500_data.gyro_y / mpu6500_data->fifo_count * MPU6500_FIFO_READ_GROUP_SIZE;
    if(MPU6500_FIFO_ENABLE_GYRO_ZOUT == 1)
        mpu6500_data->gyro_z = add_temp_mpu6500_data.gyro_z / mpu6500_data->fifo_count * MPU6500_FIFO_READ_GROUP_SIZE;

    res = MPU6500_FIFO_RST(mpu6500_data);
    if (res != HAL_OK) return MPU6500_ERROR;

    return MPU6500_OK;
}

/*
 * name:角度更新函数
 * func:更新姿态角度
 * in:mpu6500_data:MPU6500数据结构体指针	qmc5883P_data:QMC5883P数据结构体指针
 * out:void
 * PS:None
 *
 * */
void Angle_Update(struct MPU6500_Data *mpu6500_data, struct QMC5883P_Data *qmc5883P_data)
{
    float acc[3] = {0};
    float gyro[3] = {0};
    float mag[3] = {0};

    acc[0] = mpu6500_data->accel_x;
    acc[1] = mpu6500_data->accel_y;
    acc[2] = mpu6500_data->accel_z;

    gyro[0] = mpu6500_data->gyro_x * 0.01745329252f; // 乘以度转弧度
    gyro[1] = mpu6500_data->gyro_y * 0.01745329252f;
    gyro[2] = mpu6500_data->gyro_z * 0.01745329252f;

    mag[0] = qmc5883P_data->mag_x;
    mag[1] = qmc5883P_data->mag_y;
    mag[2] = qmc5883P_data->mag_z;

    MadgwickAHRSupdate(mpu6500_data, acc, gyro, mag);
    float angles[3] = {0};
    computeAngles(angles);
    mpu6500_data->pitch = angles[0];
    mpu6500_data->roll  = angles[1];
    mpu6500_data->yaw   = angles[2];
}

/*
 * name:获取世界坐标系下的加速度函数
 * func:计算世界坐标系下的加速度
 * in:mpu6500_data:MPU6500数据结构体指针
 * out:void
 * PS:None
 *
 * */
void MPU6500_GetWorldAccel(struct MPU6500_Data *mpu6500_data)
{
    // 直接使用四元数构建旋转矩阵，避免欧拉角问题
    float R[3][3];
    
    // 从四元数计算旋转矩阵（从机体到世界坐标系）
    R[0][0] = 1.0f - 2.0f*(q2*q2 + q3*q3);
    R[0][1] = 2.0f*(q1*q2 - q0*q3);
    R[0][2] = 2.0f*(q1*q3 + q0*q2);
    
    R[1][0] = 2.0f*(q1*q2 + q0*q3);
    R[1][1] = 1.0f - 2.0f*(q1*q1 + q3*q3);
    R[1][2] = 2.0f*(q2*q3 - q0*q1);
    
    R[2][0] = 2.0f*(q1*q3 - q0*q2);
    R[2][1] = 2.0f*(q2*q3 + q0*q1);
    R[2][2] = 1.0f - 2.0f*(q1*q1 + q2*q2);

    // 计算世界坐标系下的加速度（包含重力）
    mpu6500_data->accel_north = R[0][0] * mpu6500_data->accel_x + R[0][1] * mpu6500_data->accel_y + R[0][2] * mpu6500_data->accel_z;
    mpu6500_data->accel_east  = R[1][0] * mpu6500_data->accel_x + R[1][1] * mpu6500_data->accel_y + R[1][2] * mpu6500_data->accel_z;
    mpu6500_data->accel_down  = R[2][0] * mpu6500_data->accel_x + R[2][1] * mpu6500_data->accel_y + R[2][2] * mpu6500_data->accel_z;

    // mpu6500_data->accel_north = round(mpu6500_data->accel_north * 100) / 100;
    // mpu6500_data->accel_east  = round(mpu6500_data->accel_east * 100) / 100;
    // mpu6500_data->accel_down  = round(mpu6500_data->accel_down * 100) / 100 + 0.1f;

    // 减去重力加速度
    mpu6500_data->accel_down -= G;

    return;
}

/*
 * name:速度更新函数
 * func:更新速度
 * in:mpu6500_data:MPU6500数据结构体指针
 * out:void
 * PS:None
 *
 * */
void MPU6500_UpdataRate(struct MPU6500_Data *mpu6500_data)
{
    float dt_accel = 1.0f / MPU6500_ACCEL_ODR;

    static float temp1[3];

    // if(temp[0] - mpu6500_data->accel_north > a || temp[0] - mpu6500_data->accel_north < -a)
        // mpu6500_data->rate_x += mpu6500_data->accel_north * dt_accel;
    // if(temp[1] - mpu6500_data->accel_east > a || temp[1] - mpu6500_data->accel_east < -a)
        // mpu6500_data->rate_y += mpu6500_data->accel_east * dt_accel;
    // if(temp[2] - mpu6500_data->accel_down > a || temp[2] - mpu6500_data->accel_down < -a)
        // mpu6500_data->rate_z += mpu6500_data->accel_down * dt_accel;

    mpu6500_data->rate_x += mpu6500_data->accel_north * dt_accel;
    mpu6500_data->rate_y += mpu6500_data->accel_east * dt_accel;
    mpu6500_data->rate_z += mpu6500_data->accel_down * dt_accel;

    mpu6500_data->rate_x = mpu6500_data->rate_x * 0.3f + temp1[0] * 0.7f;
    mpu6500_data->rate_y = mpu6500_data->rate_y * 0.3f + temp1[1] * 0.7f;
    mpu6500_data->rate_z = mpu6500_data->rate_z * 0.3f + temp1[2] * 0.7f;

    temp1[0] = mpu6500_data->rate_x;
    temp1[1] = mpu6500_data->rate_y;
    temp1[2] = mpu6500_data->rate_z;

    return;
}

/*
 * name:位移更新函数
 * func:更新位移
 * in:mpu6500_data:MPU6500数据结构体指针
 * out:void
 * PS:None
 *
 * */
void MPU6500_UpdataDistination(struct MPU6500_Data *mpu6500_data)
{
    float dt_accel = 1.0f / MPU6500_ACCEL_ODR;

    mpu6500_data->distination_x += mpu6500_data->rate_x * dt_accel;
    mpu6500_data->distination_y += mpu6500_data->rate_y * dt_accel;
    mpu6500_data->distination_z += mpu6500_data->rate_z * dt_accel;
}

/*==============================================================================================*/
/*==========================================数据滤波============================================*/ 
/*==============================================================================================*/

/* name:低通滤波函数
 * func:对加速度进行低通滤波
 * in:mpu6500_data:MPU6500数据结构体指针
 * out:void
 * PS:None
 *
 * */
void lowPassFilter_accel(struct MPU6500_Data *mpu6500_data) 
{
    static float prev_output[3] = {0};
    uint8_t alpha_shift = 3;  // α = 1/8，越大越平滑

    prev_output[0] = prev_output[0] + ((mpu6500_data->accel_x - prev_output[0]) / pow(2 , alpha_shift));
    prev_output[1] = prev_output[1] + ((mpu6500_data->accel_y - prev_output[1]) / pow(2 , alpha_shift));
    prev_output[2] = prev_output[2] + ((mpu6500_data->accel_z - prev_output[2]) / pow(2 , alpha_shift));

    mpu6500_data->accel_x = prev_output[0];
    mpu6500_data->accel_y = prev_output[1];
    mpu6500_data->accel_z = prev_output[2];
}

/* name:低通滤波函数
 * func:对陀螺仪进行低通滤波
 * in:mpu6500_data:MPU6500数据结构体指针
 * out:void
 * PS:None
 *
 * */
void lowPassFilter_gyro(struct MPU6500_Data *mpu6500_data) 
{
    static float prev_output[3] = {0};
    uint8_t alpha_shift = 3;  // α = 1/8，越大越平滑

    prev_output[0] = prev_output[0] + ((mpu6500_data->gyro_x - prev_output[0]) / pow(2 , alpha_shift));
    prev_output[1] = prev_output[1] + ((mpu6500_data->gyro_y - prev_output[1]) / pow(2 , alpha_shift));
    prev_output[2] = prev_output[2] + ((mpu6500_data->gyro_z - prev_output[2]) / pow(2 , alpha_shift));

    mpu6500_data->gyro_x = prev_output[0];
    mpu6500_data->gyro_y = prev_output[1];
    mpu6500_data->gyro_z = prev_output[2];
}

