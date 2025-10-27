### MPU6500代码使用方法

Logs：
[25/10/27]此版本仅适用于STM32 HAL库使用，后续如有拓展会再次提起。当前版本正在写低功耗部分,但是实现流程不是很理想,基本的实现函数已经写出来了,希望有了解的大佬可以指出我的不足和问题.

## 简介

大多数操作都在.h文件中,根据注释配置即可.

这个文件的.h中的#define写的很乱,一部分原因是因为我之前从来没有尝试过预编译处理,另一部分原因是因为MPU6500的寄存器功能十分复杂,很多不同功能交错在一起,且分布广.但是从这个文件开始基本上就确定了我自己的#define的风格

仓库中应该是有我在开发学习的时候边写的日志,看一看会对理解我的代码思路有很大帮助

解算是使用的Madgwick解算

由于欧拉角的运算涉及到加速度,所以在高加速度的运动中欧拉角即使没有变还是会有一点变化,这是通病.

## 参数配置

### yaw轴角速度和磁力计与加速度互补系数,与yaw轴灵敏度有关系
~~~
#define betaDef					                    0.98f			// 角速度震荡系数(p)	Hz越大该值越小,1.02左右适用于50Hz左右       可以调整该值来改变互补滤波中加速度和角速度的权重,减缓yaw延迟
~~~

### 加速度和角速度的低通滤波系数
~~~
#define lowPassFilterFactorACC                      3               // 低通滤波系数,值越大滤波效果越明显
#define lowPassFilterFactorGYRO                     3               // 低通滤波系数,值越大滤波效果越明显
~~~

### 加速度和角速度的输出速率以及芯片内部滤波配置
~~~
// 输出配置
#define MPU6500_ACCEL_ODR                           300             //Hz
#define MPU6500_GYRO_ODR                            300             //Hz
#define _MPU6500_REG_GYRO_CONFIG_FS_SEL             0               // 0:±250dps, 1:±500dps, 2:±1000dps, 3:±2000dps
#define _MPU6500_REG_ACCEL_CONFIG_AFS_SEL           0               // 0:±2g, 1:±4g, 2:±8g, 3:±16g
// #define _MPU6500_GYRO_USE_HIGH_SPEED_MODE        1               //可填写值为0~1,数字越大速率越大,Fs=32KHz
#define _MPU6500_GYRO_USE_HIGH_LOW_MODE             1               //可填写值为1~7,数字越大速率越小(7为特殊,比1更大的速率更大)(除7和1为8KHz外,其余为1KHz)
// #define _MPU6500_ACCEL_USE_HIGH_SPEED_MODE       1               //不可改写数值,Rate=4KHz
#define _MPU6500_ACCEL_USE_HIGH_LOW_MODE            0               //可填写值为0~7,数字越大速率越小(7为特殊,与0相同),Rate=1KHz
~~~

### FSYNC配置
~~~
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
~~~

### 中断配置
~~~
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
~~~

### FIFO配置
~~~
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
~~~

## 前期准备

你至少需要开启一个STM32的I2C端口,以及开启一个基本定时器用于实时计算dt.

引入#include "MPU6500.h"在你需要使用的地方.

## 前期调用

~~~
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
~~~

在一开始,你需要在你需要的地方定义一个配置并存储陀螺仪数据的地方.

你需要输入你陀螺仪所使用的I2C信号端口I2C_HandleTypeDef *hi2c;方便多陀螺仪使用
你需要输入你陀螺仪所使用的定时器端口号TIM_HandleTypeDef *htim;方便计算dt
你需要输入你陀螺仪的地址uint8_t mpu6500_addr;方便同一I2C总线下不同期间使用

~~~
#include "QMC5883P.h"
~~~

由于需要使用到磁力计,所以请保证你的文件夹里面有磁力计的代码

~~~
uint8_t MPU6500_Init(struct MPU6500_Data *mpu6500_data);                                    // 初始化MPU6500
~~~

调用初始化函数,在你初始化I2C后

确保进行初始化后,就可以着手开始使用MPU6500获取所需要的值了.

## 函数使用

~~~
uint8_t MPU6500_Read_Accel(struct MPU6500_Data *mpu6500_data);                              // 读取加速度数据
uint8_t MPU6500_Read_Gyro(struct MPU6500_Data *mpu6500_data);                               // 读取陀螺仪数据
uint8_t MPU6500_Read_Temp(struct MPU6500_Data *mpu6500_data);                               // 读取温度数据
void Angle_Update(struct MPU6500_Data *mpu6500_data, struct QMC5883P_Data *qmc5883P_data);  // 计算姿态角
~~~

这些函数是用于获取基本的陀螺仪输出值,也就是加速度,角速度,温度,姿态角,并存储在输入的结构体指针中

~~~
uint8_t MPU6500_Read_INT_Status(struct MPU6500_Data *mpu6500_data);                         // 读取中断状态寄存器
~~~

该函数可以更新陀螺仪的中断状态,并保存在输入的结构体指针中

该函数十分重要,你可以在INT引脚触发后调用该函数,但是前提是你要在INT配置部分开启中断

~~~
// INT_STATUS对照表
#define MPU6500_INT_STATUS_WOM_INT              0x40
#define MPU6500_INT_STATUS_FIFO_OVERFLOW_INT    0x10
#define MPU6500_INT_STATUS_FSYNC_INT            0x08
#define MPU6500_INT_STATUS_DMP_INT              0x02
#define MPU6500_INT_STATUS_RAW_DATA_RDY_INT     0x01
~~~

以上是中断的对照表,函数MPU6500_Read_INT_Status的回传值可能可以是这些,在你的中断回调函数里面处理就可以了

~~~
uint8_t MPU6500_Read_FIFO(struct MPU6500_Data *mpu6500_data);                               // 读取FIFO数据
~~~

在开启FIFO后就可以读取FIFO了.但是实测有BUG,求解

~~~
uint8_t MPU6500_Full_Power_Mode(struct MPU6500_Data *mpu6500_data);                         // 全功耗模式
uint8_t MPU6500_Low_Power_Mode(struct MPU6500_Data *mpu6500_data);                          // 低功耗模式
uint8_t MPU6500_Enter_Weak_en_Motion(struct MPU6500_Data *mpu6500_data);                    // 进入运动唤醒低功耗模式
uint8_t MPU6500_Exit_Weak_en_Motion(struct MPU6500_Data *mpu6500_data);                     // 退出运动唤醒低功耗模式
~~~

低功耗部分函数,[25/10/27]正在完善

~~~
void MPU6500_GetWorldAccel(struct MPU6500_Data *mpu6500_data);                              // 获取世界坐标系下的加速度
void MPU6500_UpdataRate(struct MPU6500_Data *mpu6500_data);                                 // 更新速度数据
void MPU6500_UpdataDistination(struct MPU6500_Data *mpu6500_data);                          // 更新位移数据
~~~

后期处理函数


