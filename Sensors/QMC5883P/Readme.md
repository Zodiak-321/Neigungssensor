### QMC5883P代码使用方法

Logs：
[25/10/25]此版本仅适用于STM32 HAL库使用，后续如有拓展会再次提起。

## 前期准备

你至少需要开启一个STM32的SPI端口.

引入#include "QMC5883P.h"在你需要使用的地方.

## 前期函数调用

~~~
extern struct QMC5883P_Data{

    I2C_HandleTypeDef *hi2c;

    float mag_x;
    float mag_y;
    float mag_z;

    uint8_t int_status;

} qmc5883P_data;
~~~

在一开始,你需要在你需要的地方定义一个配置并存储磁力计数据的地方.

你需要输入你磁力计所使用的SPI信号端口I2C_HandleTypeDef *hi2c;

~~~
uint8_t QMC5883P_Init(struct QMC5883P_Data *QMC5883P_Data);             /* 初始化QMC5883P */
~~~

调用初始化函数,在你初始化SPI后

~~~
void QMC5883P_Calibration(struct QMC5883P_Data *QMC5883P_Data);         /* 磁力计校准 */
~~~

如果你是第一次在你的板子上使用磁力计或者在一个周围含铁制环境的的地方固定你的磁力计,请务必调用该函数

~~~
// 磁力计校准参数
#define MAG_X_OFFSET            0.068928f
#define MAG_Y_OFFSET            0.003696f
#define MAG_Z_OFFSET            0.024412f
#define MAG_X_SCALE             0.977537f
#define MAG_Y_SCALE             1.026078f
#define MAG_Z_SCALE             0.997570f
~~~

将输出的数据填入.h文件中的数据,进行硬软铁校准

确保进行硬软铁校准后,就可以着手开始使用QMC5883P获取磁力计值了.

## 函数使用



