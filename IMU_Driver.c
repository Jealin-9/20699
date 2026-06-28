/********************************************************************************
* header file
********************************************************************************/
#include "IMU_Driver.h"


/********************************************************************************
* macro definiton
********************************************************************************/
#define LOCK_TYPE_BLM        0U    /* BLM_LOCK */
#define LOCK_TYPE_CLM        1U    /* CLM_LOCK */
#define LOCK_TYPE_TLM        2U    /* TLM_LOCK */
#define BANK0                0U    /* BANK0 */
#define BANK1                1U    /* BANK1 */
#define BANK2                2U    /* BANK2 */
#define BANK3                3U    /* BANK3 */


/********************************************************************************
* global variable
********************************************************************************/

uint8_t (*imu_spi_Interface)(uint8_t CS, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);    /* SPI通信接口 */
void (*imu_delay_ms)(uint16_t count);    /* 延时函数接口 */

typedef enum {
    IMU_GYRO_FS_SET_640dps = 0,    /* gyro sensitivity = 640 lsb/dps */
    IMU_GYRO_FS_SET_320dps = 1,    /* gyro sensitivity = 320 lsb/dps */
    IMU_GYRO_FS_SET_240dps = 2,    /* gyro sensitivity = 240 lsb/dps */
    IMU_GYRO_FS_SET_160dps = 3,    /* gyro sensitivity = 160 lsb/dps */
    IMU_GYRO_FS_SET_120dps = 4,    /* gyro sensitivity = 120 lsb/dps */
    IMU_GYRO_FS_SET_60dps  = 5,    /* gyro sensitivity = 60  lsb/dps */
    IMU_GYRO_FS_SET_30dps  = 6,    /* gyro sensitivity = 30  lsb/dps */
    IMU_GYRO_FS_SET_15dps  = 7,    /* gyro sensitivity = 15  lsb/dps */
    IMU_GYRO_FS_SET_ALL
} IMU_GYRO_FS_SET_t

typedef enum {
    IMU_ACCEL_FS_SET_65g = 0,    /* accel sensitivity = 65 lsb/g */
    IMU_ACCEL_FS_SET_32g = 1,    /* accel sensitivity = 32 lsb/g */
    IMU_ACCEL_FS_SET_16g = 2,    /* accel sensitivity = 16 lsb/g */
    IMU_ACCEL_FS_SET_8g  = 3,    /* accel sensitivity = 8  lsb/g */
    IMU_ACCEL_FS_SET_4g  = 4,    /* accel sensitivity = 4  lsb/g */
    IMU_ACCEL_FS_SET_2g  = 5,    /* accel sensitivity = 2  lsb/g */
    IMU_ACCEL_FS_SET_1g  = 6,    /* accel sensitivity = 1  lsb/g */
    IMU_ACCEL_FS_SET_05g = 7,    /* accel sensitivity = 0.5 lsb/g */
    IMU_ACCEL_FS_SET_ALL
} IMU_ACCEL_FS_SET_t

typedef enum {
    IMU_DLPF_SET_Bypass = 0,    /* Control of DLPF = Bypass */
    IMU_DLPF_SET_10Hz   = 1,    /* Control of DLPF = 10Hz */
    IMU_DLPF_SET_13Hz   = 2,    /* Control of DLPF = 13Hz */
    IMU_DLPF_SET_30Hz   = 3,    /* Control of DLPF = 30Hz */
    IMU_DLPF_SET_47Hz   = 4,    /* Control of DLPF = 47Hz */
    IMU_DLPF_SET_60Hz   = 5,    /* Control of DLPF = 60Hz */
    IMU_DLPF_SET_250Hz  = 6,    /* Control of DLPF = 250Hz */
    IMU_DLPF_SET_400Hz  = 7,    /* Control of DLPF = 400Hz */
    IMU_DLPF_SET_ALL
} IMU_DLPF_SET_t

typedef enum {
    IMU_ODR_SET_8kHz  = 0,    /* ODR selection = 8kHz */
    IMU_ODR_SET_4kHz  = 1,    /* ODR selection = 4kHz */
    IMU_ODR_SET_2kHz  = 2,    /* ODR selection = 2kHz */
    IMU_ODR_SET_1kHz  = 3,    /* ODR selection = 1kHz */
    IMU_ODR_SET_800Hz = 4,    /* ODR selection = 800Hz */
    IMU_ODR_SET_400Hz = 5,    /* ODR selection = 400Hz */
    IMU_ODR_SET_200Hz = 6,    /* ODR selection = 200Hz */
    IMU_ODR_SET_100Hz = 7,    /* ODR selection = 100Hz */
    IMU_ODR_SET_50Hz  = 8,    /* ODR selection = 50Hz */
    IMU_ODR_SET_10Hz  = 9,    /* ODR selection = 10Hz */
    IMU_ODR_SET_256kHz = 0xF,    /* ODR selection = 256kHz */
    IMU_ODR_SET_ALL
} IMU_ODR_SET_t


/********************************************************************************
* function declaration
********************************************************************************/
static uint8_t gen_lido_crc(uint32_t data);    /* 计算CRC校验值 */
static uint8_t imu_read_reg(uint8_t address, uint16_t *read_data);    /* 读寄存器的值 */
static uint8_t imu_write_reg(uint8_t address, uint16_t write_data);    /* 写入寄存器 */
static uint8_t imu_read_fsm_com_stat(uint16_t *stat_value);    /* 读FSM状态 */
static uint8_t imu_write_fsm_com_stat(uint16_t cmd_value);    /* 发送FSM命令，转换状态 */
static uint8_t imu_lock(uint8_t lock_type);    /* 锁定寄存器 */
static uint8_t imu_unlock(uint8_t lock_type);    /* 解锁寄存器 */
static uint8_t imu_select_bank(uint8_t bank);    /* 切换BANK */
static uint8_t imu_read_whoami(uint16_t *value);    /* 读取whoami寄存器 */
static uint8_t imu_check_test_ro(uint16_t *ro_val);    /* 检查固定值 */
static uint8_t imu_device_reset(void);    /* 进入RESET状态 */
static uint8_t imu_device_lowpower(void);    /* 进入LowPower状态 */
static uint8_t imu_device_waitforspi(void);    /* 进入Wait For Spi状态 */
static uint8_t imu_device_run(void);    /* 进入RUN状态 */
static uint8_t imu_set_gyro_accel_fs(IMU_GYRO_FS_SET_t gyro_fs, IMU_ACCEL_FS_SET_t accel_fs);    /* 设置陀螺仪量程和加速度计量程 */
static uint8_t imu_set_odr(IMU_ODR_SET_t odr);    /* 设置ODR */
static uint8_t imu_set_dlpf(IMU_DLPF_SET_t dlpf);    /* 设置低通滤波 */



/********************************************************************************
* function definition
********************************************************************************/

/************************************************************
* @brief    IMU初始化配置(包括量程,ODR,DLPF)
* @param    none
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_init_config(void)
{
    uint8_t status = 1;
    uint16_t reg_rd_tmp = 0;

    imu_delay_ms(100);
    status &= imu_device_reset();    /* IMU复位(完成复位后自动进入LowPower状态) */
    status &= imu_device_waitforspi();    /* 进入Wari For Spi状态 */

    status &= imu_check_test_ro(&reg_rd_tmp);    /* 检查固定值 */
    status &= imu_read_whoami(&reg_rd_tmp);    /* 读取whoami寄存器 */
    status &= imu_set_dlpf(IMU_DLPF_SET_10Hz);    /* 设置低通滤波10Hz */
    status &= imu_set_odr(IMU_ODR_SET_8kHz);    /* 设置ODR(数据输出速率)8kHz */
    status &= imu_set_gyro_accel_fs(IMU_GYRO_FS_SET_320dps, IMU_ACCEL_FS_SET_4g);    /* 设置量程 */
    
    status &= imu_device_run();    /* 进入RUN状态 */
    imu_delay_ms(10);
    return status;
}


/************************************************************
* @brief    SPI通信读写测试
* @param    none
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_communication_test(uint16_t wdata, uint16_t *rdata)
{
    uint8_t status = 1;
    status &= imu_write_reg(0x25U, wdata);
    status &= imu_read_reg(0x25U, rdata);
    return status;
}


/************************************************************
* @brief    读取X、Y、Z轴的六轴数据及温度数据
* @param    none
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_R_data(uint16_t gyroData[], uint16_t accData[], uint16_t *tempData)
{
    /* 将读取寄存器的命令合并(temp、gyro_x、gyro_y、gyro_z、accel_x、accel_y、accel_z、dummy),节约时间 */
    static uint8_t R_data_cmd[32] = {0x81,0x00,0x00,0x03, 0x80,0x40,0x00,0x02, 0x80,0x80,0x00,0x04, 0x80,0xc0,0x00,0x06,
                                     0x81,0x40,0x00,0x01, 0x81,0x80,0x00,0x07, 0x81,0xc0,0x00,0x05, 0x9f,0xc0,0x00,0x00};
    uint8_t reg_rd_tmp[32] = 0;
    uint8_t status = 1;
    uint8_t i = 0;
    uint32_t spi_frame_data = 0;
    uint8_t crc_check = 0;

    /* 发送SPI命令并读取数据 */
    for(i = 0U; i < 8U; i++)
    {
        (void)imu_spi_Interface(IMU_SPI_CS, R_data_cmd+i*4U, reg_rd_tmp+i*4U, 4U);
    }
    /* 处理温度数据 */
    *tempData = (uint16_t)reg_rd_tmp[5]<<12U | (uint16_t)reg_rd_tmp[6]<<4U | (uint16_t)reg_rd_tmp[7]>>4U;
    /* 校验CRC */
    spi_frame_data = (uint32_t)reg_rd_tmp[4]<<24U | (uint32_t)reg_rd_tmp[5]<<16U | (uint32_t)reg_rd_tmp[6]<<8U | (uint32_t)reg_rd_tmp[7];
    crc_check = gen_lido_crc(spi_frame_data);
    if(crc_check != ((uint8_t)spi_frame_data&0x07U)) return 0U;
    /* 处理陀螺仪和加速度计数据 */
    for(i = 0U; i < 3U; i++)
    {
        gyroData[i] = (uint16_t)reg_rd_tmp[9U+4U*i]<<12U | (uint16_t)reg_rd_tmp[10U+4U*i]<<4U | (uint16_t)reg_rd_tmp[11U+4U*i]>>4U;
        accData[i] = (uint16_t)reg_rd_tmp[21U+4U*i]<<12U | (uint16_t)reg_rd_tmp[22U+4U*i]<<4U | (uint16_t)reg_rd_tmp[23U+4U*i]>>4U;
    }
    return 1U;
}


/************************************************************
* @brief    错误检测概览(SM安全机制)
* @param    fault_code:故障码
            Bit7    Bit6    Bit5    Bit4    Bit3    Bit2    Bit1    Bit0
            gyro_x  gyro_y  gyro_z  acc_x   acc_y   acc_z   spi     common
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_fault_overview(uint8_t *fault_code)
{
    uint8_t status = 1;
    uint16_t ssumok_reg = 0;
    uint8_t fault_gyro_x = 0, fault_gyro_y = 0, fault_gyro_z = 0;
    uint8_t fault_accel_x = 0, fault_accel_y = 0, fault_accel_z = 0;
    uint8_t fault_spi = 0, fault_common = 0;

    status &= imu_read_reg(0x0DU, &ssumok_reg);    /* 读取SUMMARY_STATUS_SSumOK寄存器(地址0x0D) */
    fault_gyro_x = (0xC000U != (ssumok_reg & 0xC000U))? 1U: 0U;
    fault_gyro_y = (0x3000U != (ssumok_reg & 0x3000U))? 1U: 0U;
    fault_gyro_z = (0x0C00U != (ssumok_reg & 0x0C00U))? 1U: 0U;
    fault_accel_x = (0x0300U != (ssumok_reg & 0x0300U))? 1U: 0U;
    fault_accel_y = (0x00C0U != (ssumok_reg & 0x00C0U))? 1U: 0U;
    fault_accel_z = (0x0030U != (ssumok_reg & 0x0030U))? 1U: 0U;
    fault_spi = (0x0002U != (ssumok_reg & 0x0002U))? 1U: 0U;
    fault_common = (0x000CU != (ssumok_reg & 0x000CU))? 1U: 0U;
    *fault_code = (fault_gyro_x<<7U)|(fault_gyro_y<<6U)|(fault_gyro_z<<5U)|(fault_accel_x<<4U)|(fault_accel_y<<3U)|(fault_accel_z<<2U)|(fault_spi<<1U)|fault_common;
    return status;
}


/************************************************************
* @brief    温度错误检测
* @param    fault_code:故障码
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_temp_detection(uint8_t *fault_code)
{
    uint8_t status = 1;
    uint8_t fault_temp = 0;
    uint16_t temp_err_reg = 0;

    status &= imu_read_reg(0x1FU, &temp_err_reg);    /* 读取COMMON_ST_STATUS_Cont_2寄存器(地址0x1F) */
    fault_temp = (0x0010U != (temp_err_reg & 0x0010U))? 1U: 0U;
    *fault_code = fault_temp;
    return status;
}


/************************************************************
* @brief    范围错误检测
* @param    fault_code:故障码
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_range_detection(uint8_t *fault_code)
{
    uint8_t status = 1;
    uint8_t fault = 0;
    uint16_t SSumOK_Reg = 0;
    
    status &= imu_read_reg(0x0EU, &SSumOK_Reg);    /* 读取SUMMARY_STATUS_RANGE_SSumRng寄存器(地址0x0E) */
    fault = ((SSumOK_Reg & 0x8000U) != 0x8000U)? (fault|0x80U): fault;    /* gyro_x_failure */
    fault = ((SSumOK_Reg & 0x4000U) != 0x4000U)? (fault|0x40U): fault;    /* gyro_y_failure */
    fault = ((SSumOK_Reg & 0x2000U) != 0x2000U)? (fault|0x20U): fault;    /* gyro_z_failure */
    fault = ((SSumOK_Reg & 0x1000U) != 0x1000U)? (fault|0x10U): fault;    /* accel_x_failure */
    fault = ((SSumOK_Reg & 0x0800U) != 0x0800U)? (fault|0x08U): fault;    /* accel_y_failure */
    fault = ((SSumOK_Reg & 0x0400U) != 0x0400U)? (fault|0x04U): fault;    /* accel_z_failure */
    fault = ((SSumOK_Reg & 0x0040U) != 0x0040U)? (fault|0x02U): fault;    /* temp_failure */
    *fault_code = fault;
    return status;
}


/************************************************************
* @brief    错误注入(用来测试)
* @param    none
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_fault_inject(uint8_t fault_code)
{
    uint8_t status = 1;
    return status;
}



uint8_t imu_functional_safety_diagnosis(SM_Alarms_Struct *SM_code);    /* 功能安全检测 */
uint8_t imu_temp_comp(uint16_t gyro_tc[], uint16_t gyro_qtc[], uint16_t *gyro_to, uint16_t acc_tc[], uint16_t acc_qtc[], uint16_t *acc_to);    /* IMU温度补偿 */
uint8_t imu_R_TempComp_Reg(uint16_t gyro_tc[], uint16_t gyro_qtc[], uint16_t *gyro_to, uint16_t acc_tc[], uint16_t acc_qtc[], uint16_t *acc_to);    /* 读取IMU温度补偿数据 */


static uint8_t gen_lido_crc(uint32_t data);    /* 计算CRC校验值 */
static uint8_t imu_read_reg(uint8_t address, uint16_t *read_data);    /* 读寄存器的值 */
static uint8_t imu_write_reg(uint8_t address, uint16_t write_data);    /* 写入寄存器 */
static uint8_t imu_read_fsm_com_stat(uint16_t *stat_value);    /* 读FSM状态 */
static uint8_t imu_write_fsm_com_stat(uint16_t cmd_value);    /* 发送FSM命令，转换状态 */
static uint8_t imu_lock(uint8_t lock_type);    /* 锁定寄存器 */
static uint8_t imu_unlock(uint8_t lock_type);    /* 解锁寄存器 */
static uint8_t imu_select_bank(uint8_t bank);    /* 切换BANK */
static uint8_t imu_read_whoami(uint16_t *value);    /* 读取whoami寄存器 */
static uint8_t imu_check_test_ro(uint16_t *ro_val);    /* 检查固定值 */
static uint8_t imu_device_reset(void);    /* 进入RESET状态 */
static uint8_t imu_device_lowpower(void);    /* 进入LowPower状态 */
static uint8_t imu_device_waitforspi(void);    /* 进入Wait For Spi状态 */
static uint8_t imu_device_run(void);    /* 进入RUN状态 */
static uint8_t imu_set_gyro_accel_fs(IMU_GYRO_FS_SET_t gyro_fs, IMU_ACCEL_FS_SET_t accel_fs);    /* 设置陀螺仪量程和加速度计量程 */
static uint8_t imu_set_odr(IMU_ODR_SET_t odr);    /* 设置ODR */
static uint8_t imu_set_dlpf(IMU_DLPF_SET_t dlpf);    /* 设置低通滤波 */
/************************************************************
* @brief    none
* @param    none
* @retval    none
************************************************************/
