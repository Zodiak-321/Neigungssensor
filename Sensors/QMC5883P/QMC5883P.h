#ifndef __QMC5883P_H__
#define __QMC5883P_H__

#include "main.h"
#include <stdio.h>

/*==============================================================================================*/
/*==========================================用户配置区==========================================*/ 
/*==============================================================================================*/

// QMC5883P I2C address
#define QMC5883P_ADDR 0x2C  //0101100_0

// QMC5883P Chip ID
#define QMC5883P_CHIP_ID 0x80

// 磁力计校准参数
#define MAG_X_OFFSET            0.068928f
#define MAG_Y_OFFSET            0.003696f
#define MAG_Z_OFFSET            0.024412f
#define MAG_X_SCALE             0.977537f
#define MAG_Y_SCALE             1.026078f
#define MAG_Z_SCALE             0.997570f

#define QMC5883P_CONTROL_1_ODR  QMC5883P_CONTROL_1_ODR_200HZ     //10,50,100,200Hz      输出速率

/*==============================================================================================*/
/*=============================================定义=============================================*/ 
/*==============================================================================================*/

// QMC5883P Registers
#define QMC5883P_REG_CHIP_ID       0x00  // Chip ID register
#define QMC5883P_REG_XOUT_L        0x01  // X-axis data LSB
#define QMC5883P_REG_XOUT_H        0x02  // X-axis data MSB
#define QMC5883P_REG_YOUT_L        0x03  // Y-axis data LSB
#define QMC5883P_REG_YOUT_H        0x04  // Y-axis data MSB
#define QMC5883P_REG_ZOUT_L        0x05  // Z-axis data LSB
#define QMC5883P_REG_ZOUT_H        0x06  // Z-axis data MSB
#define QMC5883P_REG_STATUS        0x09  // Status register (OVFL, DRDY)
#define QMC5883P_REG_CONTROL_1     0x0A  // Control register 1 (OSR2, OSR1, ODR, MODE)
#define QMC5883P_REG_CONTROL_2     0x0B  // Control register 2 (SOFT_RST, SELF_TEST, RNG, SET/RESET MODE)


// QMC5883P Status Register Bits
#define QMC5883P_STATUS_DRDY       0x01  // Data Ready
#define QMC5883P_STATUS_OVL        0x02  // Overflow

// QMC5883P Control Register 1 Bits
#define QMC5883P_CONTROL_1_MODE_SUSPEND  0x00  // Suspend mode
#define QMC5883P_CONTROL_1_MODE_NORMAL   0x01  // Normal mode
#define QMC5883P_CONTROL_1_MODE_SINGLE   0x02  // Single mode
#define QMC5883P_CONTROL_1_MODE_CONT     0x03  // Continuous mode
#define QMC5883P_CONTROL_1_ODR_10HZ      0x00  // Output Data Rate: 10Hz
#define QMC5883P_CONTROL_1_ODR_50HZ      0x04  // Output Data Rate: 50Hz
#define QMC5883P_CONTROL_1_ODR_100HZ     0x08  // Output Data Rate: 100Hz
#define QMC5883P_CONTROL_1_ODR_200HZ     0x0C  // Output Data Rate: 200Hz
#define QMC5883P_CONTROL_1_OSR1_8        0x00  // Over Sample Rate: 8
#define QMC5883P_CONTROL_1_OSR1_4        0x10  // Over Sample Rate: 4
#define QMC5883P_CONTROL_1_OSR1_2        0x20  // Over Sample Rate: 2
#define QMC5883P_CONTROL_1_OSR1_1        0x30  // Over Sample Rate: 1
#define QMC5883P_CONTROL_1_OSR2_1        0x00  // Down Sample Rate: 1
#define QMC5883P_CONTROL_1_OSR2_2        0x40  // Down Sample Rate: 2
#define QMC5883P_CONTROL_1_OSR2_4        0x80  // Down Sample Rate: 4
#define QMC5883P_CONTROL_1_OSR2_8        0xC0  // Down Sample Rate: 8

// QMC5883P Control Register 2 Bits
#define QMC5883P_CONTROL_2_SOFT_RESET    0x80  // Soft reset
#define QMC5883P_CONTROL_2_SELF_TEST     0x40  // Self test
#define QMC5883P_CONTROL_2_RNG_2G        0x0C  // Range: ±2G
#define QMC5883P_CONTROL_2_RNG_8G        0x08  // Range: ±8G
#define QMC5883P_CONTROL_2_RNG_12G       0x04  // Range: ±12G
#define QMC5883P_CONTROL_2_RNG_30G       0x00  // Range: ±30G
#define QMC5883P_CONTROL_2_MODE          0x00  // SET/RESET MODE

#define QMC5883_RNG_SENSITIVITY_2G   15000 // LSB/Gauss
#define QMC5883_RNG_SENSITIVITY_8G   3750  // LSB/Gauss
#define QMC5883_RNG_SENSITIVITY_12G  2500  // LSB/Gauss
#define QMC5883_RNG_SENSITIVITY_30G  1000  // LSB/Gauss

extern struct QMC5883P_Data{

    I2C_HandleTypeDef *hi2c;

    float mag_x;
    float mag_y;
    float mag_z;

    uint8_t int_status;

} qmc5883P_data;

extern enum QMC5883P_Error
{
    QMC5883P_OK,
    QMC5883P_ERROR,
    QMC5883P_ERROR_ID
} qmc5883p_error;

/*==============================================================================================*/
/*===========================================相关函数===========================================*/ 
/*==============================================================================================*/

/*==辅助函数==*/
static HAL_StatusTypeDef QMC5883P_Transmit(struct QMC5883P_Data *QMC5883P_Data, uint8_t reg, uint8_t *data, uint16_t size);
static HAL_StatusTypeDef QMC5883P_Receive(struct QMC5883P_Data *QMC5883P_Data, uint8_t reg, uint8_t *data, uint16_t size);

/*==可调用函数==*/
uint8_t QMC5883P_Init(struct QMC5883P_Data *QMC5883P_Data);             /* 初始化QMC5883P */
uint8_t QMC5883P_Read_Mag(struct QMC5883P_Data *QMC5883P_Data);         /* 读取磁力计数据 */
uint8_t QMC5883P_Read_INT_Status(struct QMC5883P_Data *QMC5883P_Data);  /* 读取中断状态 */

void QMC5883P_Calibration(struct QMC5883P_Data *QMC5883P_Data);         /* 磁力计校准 */

#endif /* __QMC5883P_H__ */