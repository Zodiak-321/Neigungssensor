#ifndef __MPU6500_H__
#define __MPU6500_H__

#include "main.h"
#include "QMC5883P.h"
#include <math.h>
#include "stm32f1xx_it.h"

/*==============================================================================================*/
/*==========================================用户配置区==========================================*/ 
/*==============================================================================================*/

#define MPU6500_ADDR_1                              0x68            // MPU6500 I2C address 不可修改
#define MPU6500_ADDR_2                              0x69            // MPU6500 I2C address 不可修改

#define G                                           9.80665f        // 重力加速度

#define betaDef					                    0.98f			// 角速度震荡系数(p)	Hz越大该值越小,1.02左右适用于50Hz左右       可以调整该值来改变互补滤波中加速度和角速度的权重,减缓yaw延迟
#define lowPassFilterFactorACC                      3               // 低通滤波系数,值越大滤波效果越明显
#define lowPassFilterFactorGYRO                     3               // 低通滤波系数,值越大滤波效果越明显

// 输出配置
#define MPU6500_ACCEL_ODR                           300             //Hz
#define MPU6500_GYRO_ODR                            300             //Hz
#define _MPU6500_REG_GYRO_CONFIG_FS_SEL             0               // 0:±250dps, 1:±500dps, 2:±1000dps, 3:±2000dps
#define _MPU6500_REG_ACCEL_CONFIG_AFS_SEL           0               // 0:±2g, 1:±4g, 2:±8g, 3:±16g
// #define _MPU6500_GYRO_USE_HIGH_SPEED_MODE        1               //可填写值为0~1,数字越大速率越大,Fs=32KHz
#define _MPU6500_GYRO_USE_HIGH_LOW_MODE             1               //可填写值为1~7,数字越大速率越小(7为特殊,比1更大的速率更大)(除7和1为8KHz外,其余为1KHz)
// #define _MPU6500_ACCEL_USE_HIGH_SPEED_MODE       1               //不可改写数值,Rate=4KHz
#define _MPU6500_ACCEL_USE_HIGH_LOW_MODE            0               //可填写值为0~7,数字越大速率越小(7为特殊,与0相同),Rate=1KHz

// FSYNC配置
#define MPU6500_FSYNC_MODE                          0               // FSYNC模式选择,0为关闭FSYNC,1为数据同步模式,2为中断模式
#if MPU6500_FSYNC_MODE == 0
    #define MPU6500_FSYNC_SYNC_MODE                 0x00            // 禁止配置
    #define MPU6500_FSYNC_INT_MODE_ACTL             0x00            // 禁止配置
#elif MPU6500_FSYNC_MODE == 1
    #define MPU6500_FSYNC_SYNC_MODE MPU6500_FSYNC_SYNC_MODE_TEMP_OUT_L  // 请见下方FSYNC同步模式选择 
    #define MPU6500_FSYNC_INT_MODE_ACTL             0x00            // 禁止配置
#elif MPU6500_FSYNC_MODE == 2
    #define MPU6500_FSYNC_SYNC_MODE                 0x00            // 禁止配置
    #define MPU6500_FSYNC_INT_MODE_ACTL             0               // FSYNC中断模式下INT引脚的电平选择,0为高电平有效,1为低电平有效
#endif

// INT配置
#define MPU6500_INT_ENABLE_WOM_EN                   0               // 0为禁止,1为允许,运动唤醒中断
#define MPU6500_INT_ENABLE_FIFO_OVERFLOW_EN         0               // 0为禁止,1为允许,FIFO溢出中断
#define MPU6500_INT_ENABLE_FSYNC_INT_EN             0               // 0为禁止,1为允许,FSYNC传输到中断
#define MPU6500_INT_ENABLE_RAW_RDY_EN               1               // 0为禁止,1为允许,原始数据就绪中断
#if MPU6500_INT_ENABLE_WOM_EN + MPU6500_INT_ENABLE_FIFO_OVERFLOW_EN + MPU6500_INT_ENABLE_FSYNC_INT_EN + MPU6500_INT_ENABLE_RAW_RDY_EN != 0
    #define MPU6500_INT_PIN_CFG_ACTL                0               // INT引脚电平选择,0为高电平有效,1为低电平有效
    #define MPU6500_INT_PIN_CFG_OPEN                0               // INT引脚输出类型选择,0为推挽输出,1为开漏输出
    #define MPU6500_INT_PIN_CFG_LATCH_INT_EN        1               // INT引脚中断信号保持方式选择,0为维持50us后自动清除,1为保持状态直到被清零
    #define MPU6500_INT_PIN_CFG_INT_ANYRD_2CLEAR    0               // INT引脚中断状态清除方式选择,0为通过读取INT_STATUS寄存器清除,1为通过任何寄存器读取清除
#else       
    #define MPU6500_INT_PIN_CFG_ACTL                0x00            // 禁止配置
    #define MPU6500_INT_PIN_CFG_OPEN                0x00            // 禁止配置
    #define MPU6500_INT_PIN_CFG_LATCH_INT_EN        0x00            // 禁止配置
    #define MPU6500_INT_PIN_CFG_INT_ANYRD_2CLEAR    0x00            // 禁止配置
#endif

// FIFO配置
#define MPU6500_FIFO_ENABLE                         0               // 0为禁止,1为允许,FIFO使能
#if MPU6500_FIFO_ENABLE == 1
    #define MPU6500_FIFO_MODE                       1               // FIFO工作模式选择,0为循环写入模式,1为停止写入模式
    #define MPU6500_FIFO_ENABLE_TEMP_OUT            0               // 0为禁止,1为允许,温度数据
    #define MPU6500_FIFO_ENABLE_GYRO_XOUT           1               // 0为禁止,1为允许,陀螺仪X轴数据
    #define MPU6500_FIFO_ENABLE_GYRO_YOUT           1               // 0为禁止,1为允许,陀螺仪Y轴数据
    #define MPU6500_FIFO_ENABLE_GYRO_ZOUT           1               // 0为禁止,1为允许,陀螺仪Z轴数据
    #define MPU6500_FIFO_ENABLE_ACCEL_OUT           1               // 0为禁止,1为允许,加速度数据
#else
    #define MPU6500_FIFO_MODE                       0x00            // 禁止配置
    #define MPU6500_FIFO_ENABLE_TEMP_OUT            0x00            // 禁止配置
    #define MPU6500_FIFO_ENABLE_GYRO_XOUT           0x00            // 禁止配置
    #define MPU6500_FIFO_ENABLE_GYRO_YOUT           0x00            // 禁止配置
    #define MPU6500_FIFO_ENABLE_GYRO_ZOUT           0x00            // 禁止配置
    #define MPU6500_FIFO_ENABLE_ACCEL_OUT           0x00            // 禁止配置
#endif

/*==============================================================================================*/
/*=============================================定义=============================================*/ 
/*==============================================================================================*/

#if _MPU6500_REG_ACCEL_CONFIG_AFS_SEL == 0
    #define MPU6500_ACCEL_SENSITIVITY 16384.0f   // LSB/g
#elif _MPU6500_REG_ACCEL_CONFIG_AFS_SEL == 1
    #define MPU6500_ACCEL_SENSITIVITY 8192.0f   // LSB/g
#elif _MPU6500_REG_ACCEL_CONFIG_AFS_SEL == 2
    #define MPU6500_ACCEL_SENSITIVITY 4096.0f   // LSB/g
#elif _MPU6500_REG_ACCEL_CONFIG_AFS_SEL == 3
    #define MPU6500_ACCEL_SENSITIVITY 2048.0f   // LSB/g
#endif

#if _MPU6500_REG_GYRO_CONFIG_FS_SEL == 0
    #define MPU6500_GYRO_SENSITIVITY 131.0f   // LSB/°/s
#elif _MPU6500_REG_GYRO_CONFIG_FS_SEL == 1
    #define MPU6500_GYRO_SENSITIVITY 65.5f   // LSB/°/s
#elif _MPU6500_REG_GYRO_CONFIG_FS_SEL == 2
    #define MPU6500_GYRO_SENSITIVITY 32.8f   // LSB/°/s
#elif _MPU6500_REG_GYRO_CONFIG_FS_SEL == 3
    #define MPU6500_GYRO_SENSITIVITY 16.4f   // LSB/°/s
#endif

#ifdef _MPU6500_GYRO_USE_HIGH_SPEED_MODE
    #define MPU6500_REG_GYRO_CONFIG_FCHOICE_B _MPU6500_GYRO_USE_HIGH_SPEED_MODE ? 0x01 : 0x02
    #define MPU6500_REG_CONFIG_DLPF_CFG 0x00
#elif (_MPU6500_GYRO_USE_HIGH_LOW_MODE == 0 || _MPU6500_GYRO_USE_HIGH_LOW_MODE)
    #define MPU6500_REG_GYRO_CONFIG_FCHOICE_B 0x00
    #define MPU6500_REG_CONFIG_DLPF_CFG _MPU6500_GYRO_USE_HIGH_LOW_MODE
#endif

#ifdef _MPU6500_ACCEL_USE_HIGH_SPEED_MODE
    #define MPU6500_REG_ACCEL_CONFIG_2_ACCEL_FCHOICE_B _MPU6500_ACCEL_USE_HIGH_SPEED_MODE ? 0x08 : 0x00
    #define MPU6500_REG_ACCEL_CONFIG_2_A_DLPF_CFG 0x00
#elif (_MPU6500_ACCEL_USE_HIGH_LOW_MODE == 0 || _MPU6500_ACCEL_USE_HIGH_LOW_MODE)
    #define MPU6500_REG_ACCEL_CONFIG_2_ACCEL_FCHOICE_B 0x00
    #define MPU6500_REG_ACCEL_CONFIG_2_A_DLPF_CFG _MPU6500_ACCEL_USE_HIGH_LOW_MODE
#endif

#if _MPU6500_REG_GYRO_CONFIG_FS_SEL == 0
    #define MPU6500_REG_GYRO_CONFIG_FS_SEL  MPU6500_REG_GYRO_CONFIG_FS_SEL_250
#elif _MPU6500_REG_GYRO_CONFIG_FS_SEL == 1
    #define MPU6500_REG_GYRO_CONFIG_FS_SEL  MPU6500_REG_GYRO_CONFIG_FS_SEL_500
#elif _MPU6500_REG_GYRO_CONFIG_FS_SEL == 2
    #define MPU6500_REG_GYRO_CONFIG_FS_SEL  MPU6500_REG_GYRO_CONFIG_FS_SEL_1000
#elif _MPU6500_REG_GYRO_CONFIG_FS_SEL == 3
    #define MPU6500_REG_GYRO_CONFIG_FS_SEL  MPU6500_REG_GYRO_CONFIG_FS_SEL_2000
#endif

#if _MPU6500_REG_ACCEL_CONFIG_AFS_SEL == 0
    #define MPU6500_REG_ACCEL_CONFIG_AFS_SEL  MPU6500_REG_ACCEL_CONFIG_AFS_SEL_2G
#elif _MPU6500_REG_ACCEL_CONFIG_AFS_SEL == 1
    #define MPU6500_REG_ACCEL_CONFIG_AFS_SEL  MPU6500_REG_ACCEL_CONFIG_AFS_SEL_4G
#elif _MPU6500_REG_ACCEL_CONFIG_AFS_SEL == 2
    #define MPU6500_REG_ACCEL_CONFIG_AFS_SEL  MPU6500_REG_ACCEL_CONFIG_AFS_SEL_8G
#elif _MPU6500_REG_ACCEL_CONFIG_AFS_SEL == 3
    #define MPU6500_REG_ACCEL_CONFIG_AFS_SEL  MPU6500_REG_ACCEL_CONFIG_AFS_SEL_16G
#endif

#if MPU6500_FIFO_ENABLE == 1
    #define MPU6500_FIFO_READ_GROUP_SIZE (2 * (MPU6500_FIFO_ENABLE_TEMP_OUT + MPU6500_FIFO_ENABLE_GYRO_XOUT + MPU6500_FIFO_ENABLE_GYRO_YOUT + MPU6500_FIFO_ENABLE_GYRO_ZOUT + MPU6500_FIFO_ENABLE_ACCEL_OUT * 3))
#else 
    #define MPU6500_FIFO_READ_GROUP_SIZE 1
#endif

// 陀螺仪量程设置
#define MPU6500_REG_GYRO_CONFIG_FS_SEL_250      0x00
#define MPU6500_REG_GYRO_CONFIG_FS_SEL_500      0x80
#define MPU6500_REG_GYRO_CONFIG_FS_SEL_1000     0x10
#define MPU6500_REG_GYRO_CONFIG_FS_SEL_2000     0x18

// 加速度量程设置
#define MPU6500_REG_ACCEL_CONFIG_AFS_SEL_2G     0x00
#define MPU6500_REG_ACCEL_CONFIG_AFS_SEL_4G     0x80
#define MPU6500_REG_ACCEL_CONFIG_AFS_SEL_8G     0x10
#define MPU6500_REG_ACCEL_CONFIG_AFS_SEL_16G    0x18

// FSYNC同步模式选择
#define MPU6500_FSYNC_SYNC_MODE_TEMP_OUT_L      0x08
#define MPU6500_FSYNC_SYNC_MODE_GYRO_XOUT_L     0x10
#define MPU6500_FSYNC_SYNC_MODE_GYRO_YOUT_L     0x18   
#define MPU6500_FSYNC_SYNC_MODE_GYRO_ZOUT_L     0x20
#define MPU6500_FSYNC_SYNC_MODE_ACCEL_XOUT_L    0x28
#define MPU6500_FSYNC_SYNC_MODE_ACCEL_YOUT_L    0x30
#define MPU6500_FSYNC_SYNC_MODE_ACCEL_ZOUT_L    0x38

// INT_STATUS对照表
#define MPU6500_INT_STATUS_WOM_INT              0x40
#define MPU6500_INT_STATUS_FIFO_OVERFLOW_INT    0x10
#define MPU6500_INT_STATUS_FSYNC_INT            0x08
#define MPU6500_INT_STATUS_DMP_INT              0x02
#define MPU6500_INT_STATUS_RAW_DATA_RDY_INT     0x01

// MPU6500 registers
#define MPU6500_REG_SELF_TEST_X_GYRO            0x00
#define MPU6500_REG_SELF_TEST_Y_GYRO            0x01
#define MPU6500_REG_SELF_TEST_Z_GYRO            0x02
#define MPU6500_REG_SELF_TEST_X_ACCEL           0x0D
#define MPU6500_REG_SELF_TEST_Y_ACCEL           0x0E
#define MPU6500_REG_SELF_TEST_Z_ACCEL           0x0F
#define MPU6500_REG_XG_OFFSET_H                 0x13
#define MPU6500_REG_XG_OFFSET_L                 0x14
#define MPU6500_REG_YG_OFFSET_H                 0x15
#define MPU6500_REG_YG_OFFSET_L                 0x16
#define MPU6500_REG_ZG_OFFSET_H                 0x17
#define MPU6500_REG_ZG_OFFSET_L                 0x18
#define MPU6500_REG_SMPLRT_DIV                  0x19
#define MPU6500_REG_CONFIG                      0x1A
#define MPU6500_REG_GYRO_CONFIG                 0x1B
#define MPU6500_REG_ACCEL_CONFIG                0x1C
#define MPU6500_REG_ACCEL_CONFIG_2              0x1D
#define MPU6500_REG_LP_ACCEL_ODR                0x1E
#define MPU6500_REG_WOM_THR                     0x1F
#define MPU6500_REG_FIFO_EN                     0x23
#define MPU6500_REG_I2C_MST_CTRL                0x24
#define MPU6500_REG_I2C_SLV0_ADDR               0x25
#define MPU6500_REG_I2C_SLV0_REG                0x26
#define MPU6500_REG_I2C_SLV0_CTRL               0x27
#define MPU6500_REG_I2C_SLV1_ADDR               0x28
#define MPU6500_REG_I2C_SLV1_REG                0x29
#define MPU6500_REG_I2C_SLV1_CTRL               0x2A
#define MPU6500_REG_I2C_SLV2_ADDR               0x2B
#define MPU6500_REG_I2C_SLV2_REG                0x2C
#define MPU6500_REG_I2C_SLV2_CTRL               0x2D
#define MPU6500_REG_I2C_SLV3_ADDR               0x2E
#define MPU6500_REG_I2C_SLV3_REG                0x2F
#define MPU6500_REG_I2C_SLV3_CTRL               0x30
#define MPU6500_REG_I2C_SLV4_ADDR               0x31
#define MPU6500_REG_I2C_SLV4_REG                0x32
#define MPU6500_REG_I2C_SLV4_DO                 0x33
#define MPU6500_REG_I2C_SLV4_CTRL               0x34
#define MPU6500_REG_I2C_SLV4_DI                 0x35
#define MPU6500_REG_I2C_MST_STATUS              0x36
#define MPU6500_REG_INT_PIN_CFG                 0x37
#define MPU6500_REG_INT_ENABLE                  0x38
#define MPU6500_REG_INT_STATUS                  0x3A
#define MPU6500_REG_ACCEL_XOUT_H                0x3B
#define MPU6500_REG_ACCEL_XOUT_L                0x3C
#define MPU6500_REG_ACCEL_YOUT_H                0x3D
#define MPU6500_REG_ACCEL_YOUT_L                0x3E
#define MPU6500_REG_ACCEL_ZOUT_H                0x3F
#define MPU6500_REG_ACCEL_ZOUT_L                0x40
#define MPU6500_REG_TEMP_OUT_H                  0x41
#define MPU6500_REG_TEMP_OUT_L                  0x42
#define MPU6500_REG_GYRO_XOUT_H                 0x43
#define MPU6500_REG_GYRO_XOUT_L                 0x44
#define MPU6500_REG_GYRO_YOUT_H                 0x45
#define MPU6500_REG_GYRO_YOUT_L                 0x46
#define MPU6500_REG_GYRO_ZOUT_H                 0x47
#define MPU6500_REG_GYRO_ZOUT_L                 0x48
#define MPU6500_REG_SIGNAL_PATH_RESET           0x68
#define MPU6500_REG_ACCEL_INTEL_CTRL            0x69
#define MPU6500_REG_USER_CTRL                   0x6A
#define MPU6500_REG_PWR_MGMT_1                  0x6B
#define MPU6500_REG_PWR_MGMT_2                  0x6C
#define MPU6500_REG_FIFO_COUNTH                 0x72
#define MPU6500_REG_FIFO_COUNTL                 0x73
#define MPU6500_REG_FIFO_R_W                    0x74
#define MPU6500_REG_WHO_AM_I                    0x75
#define MPU6500_REG_XA_OFFSET_H                 0x77
#define MPU6500_REG_XA_OFFSET_L                 0x78
#define MPU6500_REG_YA_OFFSET_H                 0x7A
#define MPU6500_REG_YA_OFFSET_L                 0x7B
#define MPU6500_REG_ZA_OFFSET_H                 0x7D
#define MPU6500_REG_ZA_OFFSET_L                 0x7E


// MUPU6500 data structure
extern struct MPU6500_Data
{
    I2C_HandleTypeDef *hi2c;
    TIM_HandleTypeDef *htim;
    uint8_t mpu6500_addr;

    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;

    float accel_north;
    float accel_east;
    float accel_down;
    float rate_x;
    float rate_y;
    float rate_z;
    float distination_x;
    float distination_y;
    float distination_z;

    float pitch;
    float roll;
    float yaw;

    uint8_t int_status;
    uint16_t fifo_count;
} mpu6500_data;

// Other structures
extern struct MPU6500_Offset
{
    float offset_x;
    float offset_y;
    float offset_z;
} mpu6500_offset;

// FSYNC配置结构体
extern struct MPU6500_FSYNC_Config
{
    uint8_t config_ext_sync_set;
    uint8_t int_pin_cfg_actl_fsync;
    uint8_t int_pin_cfg_fsync_int_mode_en;
} mpu6500_fsync_config;

// 错误枚举
extern enum MPU6500_Error
{
    MPU6500_OK,
    MPU6500_ERROR,
    MPU6500_ERROR_ID
} mpu6500_error;

/*==============================================================================================*/
/*===========================================相关函数===========================================*/ 
/*==============================================================================================*/

/*==辅助函数==*/
static HAL_StatusTypeDef MPU6500_Transmit(struct MPU6500_Data *mpu6500_data, uint8_t reg, uint8_t *data, uint16_t size);
static HAL_StatusTypeDef MPU6500_Receive(struct MPU6500_Data *mpu6500_data, uint8_t reg, uint8_t *data, uint16_t size);
static HAL_StatusTypeDef MPU6500_GyroOffset_Config(struct MPU6500_Data *mpu6500_data);
static uint8_t MPU6500_Get_SimpleDiv(uint16_t odr);
static void MPU6500_FSYNC_Config(struct MPU6500_FSYNC_Config *mpu6500_fsync_config);
static HAL_StatusTypeDef MPU6500_FIFO_RST(struct MPU6500_Data *mpu6500_data);
static uint8_t MPU6500_Read_FIFO_Count(struct MPU6500_Data *mpu6500_data);

static float invSqrt(float x);
static void computeAngles(float angle[3]);
static void Norm_Data(float *x,float *y,float *z);
static void Norm_Data_Q(float *w,float *x,float *y,float *z);
static void MadgwickAHRSupdateIMU(struct MPU6500_Data *mpu6500_data, float acc[3],float gyro[3]);
static void MadgwickAHRSupdate(struct MPU6500_Data *mpu6500_data, float acc[3],float gyro[3],float mag[3]);
static void lowPassFilter_accel(struct MPU6500_Data *mpu6500_data);
static void lowPassFilter_gyro(struct MPU6500_Data *mpu6500_data);

/*==可调用函数==*/
uint8_t MPU6500_Init(struct MPU6500_Data *mpu6500_data);                                    // 初始化MPU6500
uint8_t MPU6500_Read_Accel(struct MPU6500_Data *mpu6500_data);                              // 读取加速度数据
uint8_t MPU6500_Read_Gyro(struct MPU6500_Data *mpu6500_data);                               // 读取陀螺仪数据
uint8_t MPU6500_Read_Temp(struct MPU6500_Data *mpu6500_data);                               // 读取温度数据
uint8_t MPU6500_Read_INT_Status(struct MPU6500_Data *mpu6500_data);                         // 读取中断状态寄存器
uint8_t MPU6500_Read_FIFO(struct MPU6500_Data *mpu6500_data);                               // 读取FIFO数据
uint8_t MPU6500_Full_Power_Mode(struct MPU6500_Data *mpu6500_data);                         // 全功耗模式
uint8_t MPU6500_Low_Power_Mode(struct MPU6500_Data *mpu6500_data);                          // 低功耗模式
uint8_t MPU6500_Enter_Weak_en_Motion(struct MPU6500_Data *mpu6500_data);                    // 进入运动唤醒低功耗模式
uint8_t MPU6500_Exit_Weak_en_Motion(struct MPU6500_Data *mpu6500_data);                     // 退出运动唤醒低功耗模式
void MPU6500_GetWorldAccel(struct MPU6500_Data *mpu6500_data);                              // 获取世界坐标系下的加速度
void MPU6500_UpdataRate(struct MPU6500_Data *mpu6500_data);                                 // 更新速度数据
void MPU6500_UpdataDistination(struct MPU6500_Data *mpu6500_data);                          // 更新位移数据

void Angle_Update(struct MPU6500_Data *mpu6500_data, struct QMC5883P_Data *qmc5883P_data);  // 计算姿态角

#endif /* __MPU6500_H__ */