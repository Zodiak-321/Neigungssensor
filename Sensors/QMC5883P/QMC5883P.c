#include "QMC5883P.h"

HAL_StatusTypeDef QMC5883P_Transmit(struct QMC5883P_Data *QMC5883P_Data, uint8_t reg, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef res;
    res = HAL_I2C_Mem_Write(QMC5883P_Data->hi2c, QMC5883P_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
    return res;
}

HAL_StatusTypeDef QMC5883P_Receive(struct QMC5883P_Data *QMC5883P_Data, uint8_t reg, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef res;
    res = HAL_I2C_Mem_Read(QMC5883P_Data->hi2c, QMC5883P_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
    return res;
}

/*brief  QMC5883P初始化
 * @param  hi2c: I2C句柄指针
 * @retval 0:成功, 1:失败
 */
uint8_t QMC5883P_Init(struct QMC5883P_Data *QMC5883P_Data)
{
    uint8_t data;

    HAL_Delay(100);
    // Reset the sensor
    data = QMC5883P_CONTROL_2_SOFT_RESET;
    if (QMC5883P_Transmit(QMC5883P_Data, QMC5883P_REG_CONTROL_2, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;

    HAL_Delay(100);

    data = 0x00;
    if (QMC5883P_Transmit(QMC5883P_Data, QMC5883P_REG_CONTROL_2, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;
    
     HAL_Delay(100);

    if (QMC5883P_Receive(QMC5883P_Data, QMC5883P_REG_CHIP_ID, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;
    if (data != QMC5883P_CHIP_ID)
        return QMC5883P_ERROR_ID;

    data = QMC5883P_CONTROL_2_RNG_2G;
    if (QMC5883P_Transmit(QMC5883P_Data, QMC5883P_REG_CONTROL_2, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;

    // Configure the sensor
    data = QMC5883P_CONTROL_1_MODE_CONT | QMC5883P_CONTROL_1_ODR | QMC5883P_CONTROL_1_OSR1_2 | QMC5883P_CONTROL_1_OSR2_2;
    if (QMC5883P_Transmit(QMC5883P_Data, QMC5883P_REG_CONTROL_1, &data, 1) != HAL_OK)
        return QMC5883P_ERROR;

    return QMC5883P_OK;
}

/*brief  读取磁力计数据
 * @param  hi2c: I2C句柄指针
 * @param  QMC5883P_Data: QMC5883P数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t QMC5883P_Read_Mag(struct QMC5883P_Data *QMC5883P_Data)
{
    uint8_t data[6] = {0};
    HAL_StatusTypeDef res;

    res = QMC5883P_Receive(QMC5883P_Data, QMC5883P_REG_XOUT_L, data, 6);
    if (res != HAL_OK)
        return QMC5883P_ERROR;

    // 将数据拼接为16位有符号整数
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    // 转换为高斯值
    QMC5883P_Data->mag_x = (float)raw_x / QMC5883_RNG_SENSITIVITY_2G;
    QMC5883P_Data->mag_y = (float)raw_y / QMC5883_RNG_SENSITIVITY_2G;
    QMC5883P_Data->mag_z = (float)raw_z / QMC5883_RNG_SENSITIVITY_2G;

    QMC5883P_Data->mag_x = (QMC5883P_Data->mag_x - MAG_X_OFFSET) * MAG_X_SCALE;
    QMC5883P_Data->mag_y = (QMC5883P_Data->mag_y - MAG_Y_OFFSET) * MAG_Y_SCALE;
    QMC5883P_Data->mag_z = (QMC5883P_Data->mag_z - MAG_Z_OFFSET) * MAG_Z_SCALE;

    return QMC5883P_OK;
}

/*brief  读取中断状态
 * @param  hi2c: I2C句柄指针
 * @param  QMC5883P_Data: QMC5883P数据结构体指针
 * @retval 0:成功, 1:失败
 */
uint8_t QMC5883P_Read_INT_Status(struct QMC5883P_Data *QMC5883P_Data)
{
    HAL_StatusTypeDef res;

    res = QMC5883P_Receive(QMC5883P_Data, QMC5883P_REG_STATUS, &QMC5883P_Data->int_status, 1);
    if (res != HAL_OK)
        return QMC5883P_ERROR;

    return QMC5883P_OK;
}

/*brief  磁力计校准函数
 * @param  hi2c: I2C句柄指针
 * @param  QMC5883P_Data: QMC5883P数据结构体指针
 * @retval void
 */
void QMC5883P_Calibration(struct QMC5883P_Data *QMC5883P_Data)
{
    float max[3] = {0};
    float min[3] = {0};

    float mag_offset[3] = {0};
    float mag_scale[3] = {0};
    float avg_delta = 0;

    while(1)
    {
        QMC5883P_Read_Mag(QMC5883P_Data);

        if(QMC5883P_Data->mag_x >= max[0]) max[0] = QMC5883P_Data->mag_x;
        else if(QMC5883P_Data->mag_x <= min[0]) min[0] = QMC5883P_Data->mag_x;

        if(QMC5883P_Data->mag_y >= max[1]) max[1] = QMC5883P_Data->mag_y;
        else if(QMC5883P_Data->mag_y <= min[1]) min[1] = QMC5883P_Data->mag_y;

        if(QMC5883P_Data->mag_z >= max[2]) max[2] = QMC5883P_Data->mag_z;
        else if(QMC5883P_Data->mag_z <= min[2]) min[2] = QMC5883P_Data->mag_z;

        for(int i = 0; i < 3; i++)
        {
            mag_offset[i] = (max[i] + min[i]) / 2;
            mag_scale[i] = (max[i] - min[i]) / 2;
        }
        avg_delta = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3;
        for(int i = 0; i < 3; i++)
            mag_scale[i] = avg_delta / mag_scale[i];

        printf("%f, %f, %f, %f, %f, %f\n", mag_offset[0], mag_offset[1], mag_offset[2], mag_scale[0], mag_scale[1], mag_scale[2]);
    }
}

