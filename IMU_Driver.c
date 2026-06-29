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
} IMU_GYRO_FS_SET_t;

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
} IMU_ACCEL_FS_SET_t;

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
} IMU_DLPF_SET_t;

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
    IMU_ODR_SET_256kHz = 0xF    /* ODR selection = 256kHz */
} IMU_ODR_SET_t;


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
    uint8_t reg_rd_tmp[32] = {0};
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
    uint16_t SSumRng_Reg = 0;
    
    status &= imu_read_reg(0x0EU, &SSumRng_Reg);    /* 读取SUMMARY_STATUS_RANGE_SSumRng寄存器(地址0x0E) */
    fault = ((SSumRng_Reg & 0x8000U) != 0x8000U)? (fault|0x80U): fault;    /* gyro_x_failure */
    fault = ((SSumRng_Reg & 0x4000U) != 0x4000U)? (fault|0x40U): fault;    /* gyro_y_failure */
    fault = ((SSumRng_Reg & 0x2000U) != 0x2000U)? (fault|0x20U): fault;    /* gyro_z_failure */
    fault = ((SSumRng_Reg & 0x1000U) != 0x1000U)? (fault|0x10U): fault;    /* accel_x_failure */
    fault = ((SSumRng_Reg & 0x0800U) != 0x0800U)? (fault|0x08U): fault;    /* accel_y_failure */
    fault = ((SSumRng_Reg & 0x0400U) != 0x0400U)? (fault|0x04U): fault;    /* accel_z_failure */
    fault = ((SSumRng_Reg & 0x0040U) != 0x0040U)? (fault|0x02U): fault;    /* temp_failure */
    *fault_code = fault;
    return status;
}


/************************************************************
* @brief    IMU配置温度补偿系数
* @param    gyro_tc:温度补偿一次项系数(acc_tc同类)
* @param    gyro_qtc:温度补偿二次项系数(acc_qtc)
* @param    gyro_to:温度差补偿系数(acc_to)
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_temp_comp(uint16_t gyro_tc[], uint16_t gyro_qtc[], uint16_t *gyro_to, uint16_t acc_tc[], uint16_t acc_qtc[], uint16_t *acc_to)
{
    uint8_t status = 1;
    uint8_t i = 0;

    status &= imu_device_waitforspi();
    status &= imu_unlock(LOCK_TYPE_TLM);
    status &= imu_select_bank(BANK3);

    for(i = 0U; i < 5U; i++)
    {
        status &= imu_write_reg(i +0x4CU, gyro_tc[i]);    /* 陀螺仪一次项温度补偿系数寄存器首地址0x4C */
        status &= imu_write_reg(i +0x51U, gyro_qtc[i]);    /* 陀螺仪二次项温度补偿系数寄存器首地址0x51 */
        status &= imu_write_reg(i +0x3DU, acc_tc[i]);    /* 加速度一次项温度补偿系数寄存器首地址0x3D */
        status &= imu_write_reg(i +0x42U, acc_qtc[i]);    /* 加速度二次项温度补偿系数寄存器首地址0x42 */
    }
    status &= imu_write_reg(0x5DU, gyro_to);    /* 陀螺仪温度差补偿系数寄存器地址0x5D */
    status &= imu_write_reg(0x5EU, acc_to);    /* 加速度温度差补偿系数寄存器地址0x5E */

    status &= imu_select_bank(BANK0);
    status &= imu_lock(LOCK_TYPE_TLM);
    status &= imu_device_run();

    return status;
}


/************************************************************
* @brief    读取IMU温度补偿数据
* @param    gyro_tc:温度补偿一次项系数(acc_tc同类)
* @param    gyro_qtc:温度补偿二次项系数(acc_qtc)
* @param    gyro_to:温度差补偿系数(acc_to)
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_R_TempComp_Reg(uint16_t gyro_tc[], uint16_t gyro_qtc[], uint16_t *gyro_to, uint16_t acc_tc[], uint16_t acc_qtc[], uint16_t *acc_to)
{
    uint8_t status = 1;
    uint8_t i = 0;

    status &= imu_device_waitforspi();
    status &= imu_select_bank(BANK3);

    for(i = 0U; i < 5U; i++)
    {
        status &= imu_read_reg(i +0x4CU, gyro_tc[i]);    /* 陀螺仪一次项温度补偿系数寄存器首地址0x4C */
        status &= imu_read_reg(i +0x51U, gyro_qtc[i]);    /* 陀螺仪二次项温度补偿系数寄存器首地址0x51 */
        status &= imu_read_reg(i +0x3DU, acc_tc[i]);    /* 加速度一次项温度补偿系数寄存器首地址0x3D */
        status &= imu_read_reg(i +0x42U, acc_qtc[i]);    /* 加速度二次项温度补偿系数寄存器首地址0x42 */
    }
    status &= imu_read_reg(0x5DU, gyro_to);    /* 陀螺仪温度差补偿系数寄存器地址0x5D */
    status &= imu_read_reg(0x5EU, acc_to);    /* 加速度温度差补偿系数寄存器地址0x5E */

    status &= imu_select_bank(BANK0);
    status &= imu_device_run();

    return status;
}


/************************************************************
* @brief    错误注入(用来测试)
* @param    fault_code:错误码
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_fault_inject(uint8_t fault_code)
{
    uint8_t status = 1;
    uint16_t dummy_errinj_5_reg = 0;
    uint16_t reg_rd_tmp = 0;

    dummy_errinj_5_reg = ((fault_code & 0x40U) == 0x40U)? (dummy_errinj_5_reg | 0x0800U): dummy_errinj_5_reg;    /* gyro_x_failure */
    dummy_errinj_5_reg = ((fault_code & 0x20U) == 0x20U)? (dummy_errinj_5_reg | 0x0400U): dummy_errinj_5_reg;    /* gyro_y_failure */
    dummy_errinj_5_reg = ((fault_code & 0x10U) == 0x10U)? (dummy_errinj_5_reg | 0x0200U): dummy_errinj_5_reg;    /* gyro_z_failure */
    
    dummy_errinj_5_reg = ((fault_code & 0x40U) == 0x40U)? (dummy_errinj_5_reg | 0x0010U): dummy_errinj_5_reg;    /* gyro_x_failure */
    dummy_errinj_5_reg = ((fault_code & 0x20U) == 0x20U)? (dummy_errinj_5_reg | 0x0008U): dummy_errinj_5_reg;    /* gyro_y_failure */
    dummy_errinj_5_reg = ((fault_code & 0x10U) == 0x10U)? (dummy_errinj_5_reg | 0x0004U): dummy_errinj_5_reg;    /* gyro_z_failure */
    
    dummy_errinj_5_reg = ((fault_code & 0x08U) == 0x08U)? (dummy_errinj_5_reg | 0x4000U): dummy_errinj_5_reg;    /* accel_x_failure */
    dummy_errinj_5_reg = ((fault_code & 0x04U) == 0x04U)? (dummy_errinj_5_reg | 0x2000U): dummy_errinj_5_reg;    /* accel_y_failure */
    dummy_errinj_5_reg = ((fault_code & 0x02U) == 0x02U)? (dummy_errinj_5_reg | 0x1000U): dummy_errinj_5_reg;    /* accel_z_failure */
    dummy_errinj_5_reg = ((fault_code & 0x01U) == 0x01U)? (dummy_errinj_5_reg | 0x0020U): dummy_errinj_5_reg;    /* temp_failure */

    status &= imu_device_waitforspi();
    status &= imu_unlock(LOCK_TYPE_CLM);
    status &= imu_write_reg(0x2EU, dummy_errinj_5_reg);
    status &= imu_read_reg(0x2EU, &reg_rd_tmp);
    status &= (dummy_errinj_5_reg == reg_rd_tmp)? 1U: 0U;
    status &= imu_lock(LOCK_TYPE_CLM);
    status &= imu_device_run();
    
    return status;
}


/************************************************************
* @brief    功能安全诊断(全部)
* @param    SM_code:功能安全码
* @retval    1：成功, 0：失败
************************************************************/
uint8_t imu_functional_safety_diagnosis(SM_Alarms_Struct *SM_Code)
{
    uint8_t status = 1;
    uint8_t summary_fault = 0;
    uint16_t safety_reg = 0;

    status &= imu_fault_overview(&summary_fault);
    if(0U != (summary_fault & 0x80U))    /* X轴角速度报警 */
    {
        status &= imu_read_reg(0x10U, &safety_reg);    /* 读GYRO_ST_STATUS_X_Cont寄存器(地址0x10) */
        SM_Code->SM02_GYRO_X_Drvratio = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM03_GYRO_X_Drfreqmeas = (0U == (safety_reg & 0x4000U))? 0U: 1U;
        SM_Code->SM04_GYRO_X_Quadadc = (0U == (safety_reg & 0x2000U))? 0U: 1U;
        SM_Code->SM07_GYRO_X_Drclk = (0U == (safety_reg & 0x1000U))? 0U: 1U;
        SM_Code->SM20_GYRO_X_Alarm = (0U == (safety_reg & 0x0800U))? 0U: 1U;
        status &= imu_read_reg(0x13U, &safety_reg);    /* 读GYRO_ST_STATUS_X_SingleShot寄存器(地址0x13) */
        SM_Code->SM36_GYRO_X_St = (0U == (safety_reg & 0x8000U))? 0U: 1U;
    }
    if(0U != (summary_fault & 0x40U))    /* Y轴角速度报警 */
    {
        status &= imu_read_reg(0x11U, &safety_reg);    /* 读GYRO_ST_STATUS_Y_Cont寄存器(地址0x11) */
        SM_Code->SM02_GYRO_Y_Drvratio = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM03_GYRO_Y_Drfreqmeas = (0U == (safety_reg & 0x4000U))? 0U: 1U;
        SM_Code->SM04_GYRO_Y_Quadadc = (0U == (safety_reg & 0x2000U))? 0U: 1U;
        SM_Code->SM07_GYRO_Y_Drclk = (0U == (safety_reg & 0x1000U))? 0U: 1U;
        SM_Code->SM20_GYRO_Y_Alarm = (0U == (safety_reg & 0x0800U))? 0U: 1U;
        status &= imu_read_reg(0x14U, &safety_reg);    /* 读GYRO_ST_STATUS_Y_SingleShot寄存器(地址0x14) */
        SM_Code->SM36_GYRO_Y_St = (0U == (safety_reg & 0x8000U))? 0U: 1U;
    }
    if(0U != (summary_fault & 0x20U))    /* Z轴角速度报警 */
    {
        status &= imu_read_reg(0x12U, &safety_reg);    /* 读GYRO_ST_STATUS_Z_Cont寄存器(地址0x12) */
        SM_Code->SM02_GYRO_Z_Drvratio = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM03_GYRO_Z_Drfreqmeas = (0U == (safety_reg & 0x4000U))? 0U: 1U;
        SM_Code->SM04_GYRO_Z_Quadadc = (0U == (safety_reg & 0x2000U))? 0U: 1U;
        SM_Code->SM07_GYRO_Z_Drclk = (0U == (safety_reg & 0x1000U))? 0U: 1U;
        SM_Code->SM20_GYRO_Z_Alarm = (0U == (safety_reg & 0x0800U))? 0U: 1U;
        status &= imu_read_reg(0x15U, &safety_reg);    /* 读GYRO_ST_STATUS_Z_SingleShot寄存器(地址0x15) */
        SM_Code->SM36_GYRO_Z_St = (0U == (safety_reg & 0x8000U))? 0U: 1U;
    }
    if(0U != (summary_fault & 0x10U))    /* X轴加速度报警 */
    {
        status &= imu_read_reg(0x17U, &safety_reg);    /* 读ACCEL_ST_STATUS_X_Cont寄存器(地址0x17) */
        SM_Code->SM19_ACCEL_X_Alarm = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM25_ACCEL_X_C2V = (0U == (safety_reg & 0x2000U))? 0U: 1U;
        SM_Code->SM19_MGA_X_Alarm = (0U == (safety_reg & 0x0200U))? 0U: 1U;
        status &= imu_read_reg(0x1AU, &safety_reg);    /* 读ACCEL_ST_STATUS_X_SingleShot寄存器(地址0x1A) */
        SM_Code->SM16_ACCEL_X_St = (0U == (safety_reg & 0x8000U))? 0U: 1U;
    }
    if(0U != (summary_fault & 0x08U))    /* Y轴加速度报警 */
    {
        status &= imu_read_reg(0x18U, &safety_reg);    /* 读ACCEL_ST_STATUS_Y_Cont寄存器(地址0x18) */
        SM_Code->SM19_ACCEL_Y_Alarm = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM25_ACCEL_Y_C2V = (0U == (safety_reg & 0x2000U))? 0U: 1U;
        SM_Code->SM19_MGA_Y_Alarm = (0U == (safety_reg & 0x0200U))? 0U: 1U;
        status &= imu_read_reg(0x1BU, &safety_reg);    /* 读ACCEL_ST_STATUS_Y_SingleShot寄存器(地址0x1B) */
        SM_Code->SM16_ACCEL_Y_St = (0U == (safety_reg & 0x8000U))? 0U: 1U;
    }
    if(0U != (summary_fault & 0x04U))    /* Z轴加速度报警 */
    {
        status &= imu_read_reg(0x19U, &safety_reg);    /* 读ACCEL_ST_STATUS_Z_Cont寄存器(地址0x19) */
        SM_Code->SM17_Vrefshieldz = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM19_ACCEL_Z_Alarm = (0U == (safety_reg & 0x4000U))? 0U: 1U;
        SM_Code->SM25_ACCEL_Z_C2V = (0U == (safety_reg & 0x1000U))? 0U: 1U;
        SM_Code->SM19_MGA_Z_Alarm = (0U == (safety_reg & 0x0100U))? 0U: 1U;
        status &= imu_read_reg(0x1CU, &safety_reg);    /* 读ACCEL_ST_STATUS_Z_SingleShot寄存器(地址0x1C) */
        SM_Code->SM24_ACCEL_Z_Vrefsh_Charge = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM16_ACCEL_Z_St = (0U == (safety_reg & 0x0040U))? 0U: 1U;
    }
    if(0U != (summary_fault & 0x02U))    /* SPI通信报警 */
    {
        status &= imu_read_reg(0x0FU, &safety_reg);    /* 读SPI_COMMUNICATION_STATUS寄存器(地址0x0F) */
        SM_Code->SM30_Spi_Clkcnt = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM30_Spi_Crc = (0U == (safety_reg & 0x4000U))? 0U: 1U;
    }
    if(0U != (summary_fault & 0x01U))    /* COMMON报警 */
    {
        status &= imu_read_reg(0x1EU, &safety_reg);    /* 读COMMON_ST_STATUS_Cont_1寄存器(地址0x1E) */
        SM_Code->SM18_ACCEL_Cp = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM14_GYRO_Vref = (0U == (safety_reg & 0x4000U))? 0U: 1U;
        SM_Code->SM22_Ahb_Eccerr = (0U == (safety_reg & 0x2000U))? 0U: 1U;
        SM_Code->SM11_Avdd = (0U == (safety_reg & 0x1000U))? 0U: 1U;
        SM_Code->SM34_Bg = (0U == (safety_reg & 0x0800U))? 0U: 1U;
        SM_Code->SM14_Cp_Vref = (0U == (safety_reg & 0x0400U))? 0U: 1U;
        SM_Code->SM12_Dvdd = (0U == (safety_reg & 0x0200U))? 0U: 1U;
        SM_Code->SM06_GYRO_Cp25 = (0U == (safety_reg & 0x0100U))? 0U: 1U;
        SM_Code->SM06_GYRO_Cp5 = (0U == (safety_reg & 0x0080U))? 0U: 1U;
        SM_Code->SM14_ACCEL_Vref = (0U == (safety_reg & 0x0040U))? 0U: 1U;
        SM_Code->SM22_Otp_Eccerr = (0U == (safety_reg & 0x0002U))? 0U: 1U;
        SM_Code->SM03_Rcosc1_Freqmeas = (0U == (safety_reg & 0x0001U))? 0U: 1U;
        
        status &= imu_read_reg(0x1FU, &safety_reg);    /* 读COMMON_ST_STATUS_Cont_2寄存器(地址0x1F) */
        SM_Code->SM03_Rcosc2_Freqmeas = (0U == (safety_reg & 0x8000U))? 0U: 1U;
        SM_Code->SM52_GYRO_Cavity_Check = (0U == (safety_reg & 0x2000U))? 0U: 1U;
        SM_Code->SM30_Spi_Encdecod = (0U == (safety_reg & 0x1000U))? 0U: 1U;
        SM_Code->SM03_Sysclk_Freqmeas = (0U == (safety_reg & 0x0800U))? 0U: 1U;
        SM_Code->SM08_Temp_Dsp = (0U == (safety_reg & 0x0400U))? 0U: 1U;
        SM_Code->SM14_Temp_Vref = (0U == (safety_reg & 0x0200U))? 0U: 1U;
        SM_Code->SM08_Temp12 = (0U == (safety_reg & 0x0100U))? 0U: 1U;
        SM_Code->SM13_Vdd = (0U == (safety_reg & 0x0080U))? 0U: 1U;
        SM_Code->SM13_Vddio = (0U == (safety_reg & 0x0040U))? 0U: 1U;
        SM_Code->SM10_Vddmaster = (0U == (safety_reg & 0x0020U))? 0U: 1U;
        SM_Code->SM17_Vrefshieldxy = (0U == (safety_reg & 0x0010U))? 0U: 1U;
        SM_Code->SM33_Reg_Crc = (0U == (safety_reg & 0x0002U))? 0U: 1U;
        SM_Code->SM33_Reg_Parity = (0U == (safety_reg & 0x0001U))? 0U: 1U;
        
        status &= imu_read_reg(0x21U, &safety_reg);    /* 读COMMON_ST_STATUS_SingleShot_1寄存器(地址0x21) */
        SM_Code->SM26_Otp_Cpy = (0U == (safety_reg & 0x0800U))? 0U: 1U;
        SM_Code->SM26_Otp_Crc = (0U == (safety_reg & 0x0400U))? 0U: 1U;
        SM_Code->SM26_Otp_Reg = (0U == (safety_reg & 0x0200U))? 0U: 1U;
        SM_Code->SM29_Ahb_Bus = (0U == (safety_reg & 0x0100U))? 0U: 1U;
    }
    return status;
}





/************************************************************
* @brief    计算CRC校验值
* @param    data:fram data
* @retval    crc value
* @note    多项式为CRC-3/GSM(poly=0x03),初始值0x05,MSB-first
************************************************************/
static uint8_t gen_lido_crc(uint32_t data)
{
    uint8_t crc[3] = {1, 0, 1};
    uint32_t shift = 0x80000000U;
    uint32_t crc_data = data & 0xFFFFFFF8U;
    uint8_t crc_tmp = 0, i = 0;

    for(i = 0U; i < 32U; ++i)
    {
        crc_tmp = crc[2];
        crc[2] = crc[1];
        crc[1] = crc[0] ^ crc_tmp;
        crc[0] = ((uint8_t)((crc_data & (shift >> i)) > 0U)) ^ crc_tmp;
    }
    crc_tmp = (crc[2] << 2U) | (crc[1] << 1U) | crc[0];
    return crc_tmp;
}


/************************************************************
* @brief    读IMU寄存器的值
* @param    address:寄存器地址
* @param    read_data:寄存器值
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_read_reg(uint8_t address, uint16_t *read_data)
{
    uint32_t imu_cmd = 0, imu_reg = 0;
    uint8_t spi_txData[4] = {0};
    uint8_t spi_rxData[4] = {0};
    uint8_t dummy_cmd[4] = {0x9F, 0xC0, 0x00, 0x00};

    /* 根据SPI帧结构创建SPI命令 */
    imu_cmd = 0x80000000U | ((uint32_t)address <<22U);
    imu_cmd = (gen_lido_crc(imu_cmd) & 0x00000007U) | imu_cmd;

    /* 发送SPI命令并接收数据 */
    spi_txData[0] = (uint8_t)(imu_cmd >> 24U);
    spi_txData[1] = (uint8_t)(imu_cmd >> 16U);
    spi_txData[2] = (uint8_t)(imu_cmd >> 8U);
    spi_txData[3] = (uint8_t)imu_cmd
    (void)imu_spi_Interface(IMU_SPI_CS, spi_txData, spi_rxData, 4);
    (void)imu_spi_Interface(IMU_SPI_CS, dummy_cmd, spi_rxData, 4);

    imu_reg = (uint32_t)spi_rxData[0]<<24U | (uint32_t)spi_rxData[1]<<16U | (uint32_t)spi_rxData[2]<<8U | (uint32_t)spi_rxData[3];
    *read_data = (uint16_t)(imu_reg >> 4U);

    /* CRC校验 */
    if(gen_lido_crc(imu_reg) == ((uint8_t)imu_reg & 0x07U))
        return 1U;
    else
        return 0U;
}


/************************************************************
* @brief    向IMU寄存器写入
* @param    address:寄存器地址
* @param    write_data:要写入的寄存器值
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_write_reg(uint8_t address, uint16_t write_data)
{
    uint32_t imu_cmd = 0;
    uint16_t imu_rd = 0;
    uint8_t spi_txData[4] = {0};
    uint8_t spi_rxData[4] = {0};
    uint8_t dummy_cmd[4] = {0x9F, 0xC0, 0x00, 0x00};

    /* 根据SPI帧结构创建SPI命令 */
    imu_cmd = 0x80200000U | ((uint32_t)address <<22U) | ((uint32_t)write_data << 3U);
    imu_cmd = (gen_lido_crc(imu_cmd) & 0x00000007U) | imu_cmd;

    /* 发送SPI命令并接收数据 */
    spi_txData[0] = (uint8_t)(imu_cmd >> 24U);
    spi_txData[1] = (uint8_t)(imu_cmd >> 16U);
    spi_txData[2] = (uint8_t)(imu_cmd >> 8U);
    spi_txData[3] = (uint8_t)imu_cmd;
    (void)imu_spi_Interface(IMU_SPI_CS, spi_txData, spi_rxData, 4);
    (void)imu_spi_Interface(IMU_SPI_CS, dummy_cmd, spi_rxData, 4);

    return 1U;
}


/************************************************************
* @brief    读取IMU当前的FSM状态
* @param    stat_value:IMU此时的FSM状态
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_read_fsm_com_stat(uint16_t *stat_value)
{
    uint8_t ret = imu_read_reg(0x7EU, stat_value);    /* FSM_COM_STAT寄存器地址0x7E */
    *stat_value = (*stat_value & 0xFF00U) >> 8U;
    return ret;
}


/************************************************************
* @brief    发送FSM命令，转换IMU状态
* @param    cmd_value:FSM_COM命令
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_write_fsm_com_stat(uint16_t cmd_value)
{
    uint8_t status = 1;
    uint16_t fsm_com_stat = 0;
    status &= imu_read_reg(0x7EU, &fsm_com_stat);    /* FSM_COM_STAT寄存器地址0x7E */
    fsm_com_stat &= ~(0x00FFU);
    fsm_com_stat |= cmd_value;
    status &= imu_write_reg(0x7EU, fsm_com_stat);    /* FSM_COM_STAT寄存器地址0x7E */
    return status;
}


/************************************************************
* @brief    锁定寄存器
* @param    lock_type: 0:BLM, 1:CLM, 2:TLM
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_lock(uint8_t lock_type)
{
    uint8_t status = 1;
    uint16_t unlock_psw_val = 0;
    uint16_t mask = 0, shift = 0, lock_code = 0;

    switch(lock_type)
    {
        case 0U: mask = 0x07U; lock_code = 5U;
            break;
        case 1U: mask = 0x380U; shift = 7U; lock_code = 4U;
            break;
        case 2U: mask = 0x1C00U; shift = 10U; lock_code = 6U;
            break;
        default: break;
    }
    status &= imu_read_reg(0x6EU, &unlock_psw_val);    /* MODO_REGISTER寄存器地址0x6E */
    if(0U != ((unlock_psw_val & mask) >> shift))    /* 非零值表示当前寄存器并非锁定状态 */
    {
        lock_code = (lock_code << shift) & mask;
        status &= imu_write_reg(0x6EU, lock_code);
    }
    status &= imu_read_reg(0x6EU, &unlock_psw_val);
    status &= (0U == (unlock_psw_val & mask))? 1U: 0U;
    return status;
}


/************************************************************
* @brief    解锁寄存器
* @param    lock_type: 0:BLM, 1:CLM, 2:TLM
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_unlock(uint8_t lock_type)
{
    uint8_t status = 1, i = 0, j = 0;
    uint16_t mask = 0, shift = 0;
    uint16_t unlock_code[4] = {0};
    uint16_t unlock_psw_val = 0;
    static uint16_t unlock_status[4] = {0, 2, 3, 7};
    
    switch(lock_type)
    {
        case 0U:    /* BLM UNLOCK */
            mask = 0x0007U; unlock_code[0] = 2U; unlock_code[1] = 1U; unlock_code[2] = 4U;
            break;
        case 1U:     /* CLM UNLOCK */
            mask = 0x0380U; shift = 7U; unlock_code[0] = 6U; unlock_code[1] = 3U; unlock_code[2] = 5U;
            break;
        case 2U:     /* TLM UNLOCK */
            mask = 0x1C00U; shift = 10U; unlock_code[0] = 7U; unlock_code[1] = 4U; unlock_code[2] = 3U;
            break;
        default: break;
    }
    status &= imu_lock(lock_type);
    status &= imu_read_reg(0x6EU, &unlock_psw_val);    /* MODO_REGISTER寄存器地址0x6E */
    while(((unlock_psw_val & mask) >> shift) != unlock_status[3])    /* 完成解锁时状态因为0x07 */
    {
        for(i = 0U; i < 4U; i++)
        {
            /* 按照顺序写入110->011->101, 对应状态未010->011->111 */
            if(unlock_status[i] == ((unlock_psw_val & mask) >> shift))
            {
                unlock_psw_val = (unlock_code[i] << shift) & mask;
                status &= imu_write_reg(0x6EU, unlock_psw_val);
                break;
            }
        }
        status &= imu_read_reg(0x6EU, &unlock_psw_val);
        j++; if(j > 90) break;
    }
    status &= imu_read_reg(0x6EU, &unlock_psw_val);
    status &= (((unlock_psw_val & mask) >> shift) == unlock_status[3])? 1U: 0U;
    return status;
}


/************************************************************
* @brief    设置所处BANK区
* @param    bank: 0:bank0, 1:bank1, 2:bank2, 3:bank3
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_select_bank(uint8_t bank)
{
    uint8_t status = 1;
    uint16_t bank_val = 0;

    status &= imu_read_reg(0x7FU, &bank_val);    /* bank寄存器地址0x7F */
    bank_val &= 0x03U;
    if((bank_val != bank) && status)
    {
        if(0U == bank_val)
            status &= imu_unlock(LOCK_TYPE_BLM);
        status &= imu_write_reg(0x7FU, bank);
        status &= imu_read_reg(0x7FU, &bank_val);
        if(bank != bank_val) status = 0U;
        if(0U == bank_val)
            status &= imu_lock(LOCK_TYPE_BLM);
    }
    return status;
}


/************************************************************
* @brief    进入RESET状态(完成复位后会自动进入LowPower状态)
* @param    none
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_device_reset(void)
{
    uint8_t status = 1, i = 0;
    uint16_t fsm_status_reg = 0;

    status &= imu_write_fsm_com_stat(0x00F9U);    /* 发送GO_RESET指令(0x00F9) */

    for(i = 0U; i < 30U; i++)
    {
        imu_delay_ms(10U);    /* 延时10ms */
        status &= imu_read_fsm_com_stat(&fsm_status_reg);    /* 读此时的FSM状态是否是LowPower状态(0x01) */
        if(0x01U == fsm_status_reg)
            break;
    }
    if((1U==fsm_status_reg) && (1U==status))
        return 1U;
    else
        return 0U;
}


/************************************************************
* @brief    进入LowPower状态
* @param    none
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_device_lowpower(void)
{
    uint8_t status = 1, i = 0;
    uint16_t fsm_status_reg = 0;

    status &= imu_read_fsm_com_stat(&fsm_status_reg);     /* 读此时的FSM状态 */

    if(1U == fsm_status_reg) {
        return status;    /* 若处于LowPower状态,直接结束 */
    }
    else if((3U==fsm_status_reg) || (5U==fsm_status_reg)) {
        status &= imu_write_fsm_com_stat(0x0004U);    /* 若处于WaitForSpi状态或RUN状态,发送GO_LowPower指令(0x0004) */
    }
    else {
        return 0U;
    }

    for(i = 0U; i < 30U; i++)
    {
        imu_delay_ms(10U);    /* 延时10ms */
        status &= imu_read_fsm_com_stat(&fsm_status_reg);    /* 读此时的FSM状态是否是LowPower状态(0x01) */
        if(0x01U == fsm_status_reg)
            break;
    }
    if((1U==fsm_status_reg) && (1U==status))
        return 1U;
    else
        return 0U;
}


/************************************************************
* @brief    进入Wait For Spi状态
* @param    none
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_device_waitforspi(void)
{
    uint8_t status = 1, i = 0;
    uint16_t fsm_status_reg = 0;

    status &= imu_read_fsm_com_stat(&fsm_status_reg);     /* 读此时的FSM状态 */

    if(3U == fsm_status_reg) {
        return status;    /* 若处于WaitForSpi状态,直接结束 */
    }
    else if(1U==fsm_status_reg) {
        status &= imu_write_fsm_com_stat(0x0001U);    /* 若处于LowPower状态,发送GO_WakeUp指令(0x0001) */
    }
    else if((5U==fsm_status_reg) || (7U==fsm_status_reg)) {
        status &= imu_write_fsm_com_stat(0x0003U);    /* 若处于RUN状态或Wake_On_Motion状态,发送GO_Wait指令(0x0003) */
    }
    else {
        return 0U;
    }

    for(i = 0U; i < 30U; i++)
    {
        imu_delay_ms(10U);    /* 延时10ms */
        status &= imu_read_fsm_com_stat(&fsm_status_reg);    /* 读此时的FSM状态是否是WaitForSpi状态(0x03) */
        if(0x03U == fsm_status_reg)
            break;
    }
    if((3U==fsm_status_reg) && (1U==status))
        return 1U;
    else
        return 0U;
}


/************************************************************
* @brief    进入RUN状态
* @param    none
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_device_run(void)
{
    uint8_t status = 1, i = 0;
    uint16_t fsm_status_reg = 0;

    status &= imu_read_fsm_com_stat(&fsm_status_reg);     /* 读此时的FSM状态 */

    if(5U == fsm_status_reg) {
        return status;    /* 若处于Run状态,直接结束 */
    }
    else if(3U==fsm_status_reg) {
        status &= imu_write_fsm_com_stat(0x0002U);    /* 若处于WaitForSpi状态,发送EOI指令(0x0002) */
    }
    else {
        return 0U;
    }

    for(i = 0U; i < 30U; i++)
    {
        imu_delay_ms(10U);    /* 延时10ms */
        status &= imu_read_fsm_com_stat(&fsm_status_reg);    /* 读此时的FSM状态是否是Run状态(0x05) */
        if(0x05U == fsm_status_reg)
            break;
    }
    if((5U==fsm_status_reg) && (1U==status))
        return 1U;
    else
        return 0U;
}


/************************************************************
* @brief    读取whoami寄存器
* @param    value:WHO_AM_I寄存器值
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_read_whoami(uint16_t *value)
{
    uint8_t status = 1;
    status &= imu_select_bank(BANK0);
    status &= imu_unlock(LOCK_TYPE_TLM);
    status &= imu_read_reg(0x39U, value);    /* WHO_AM_I寄存器地址0x39 */
    status &= imu_lock(LOCK_TYPE_TLM);
    return status;
}


/************************************************************
* @brief    检查固定值
* @param    ro_val:TEST_RO寄存器值
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_check_test_ro(uint16_t *ro_val)
{
    uint8_t status = 1;
    status &= imu_select_bank(BANK0);
    status &= imu_read_reg(0x0CU, ro_val);
    if(0xAA55U != *ro_val || 0U == status)
        return 0U;
    else
        return 1U;
}


/************************************************************
* @brief    设置陀螺仪量程和加速度计量程
* @param    gyro_fs:陀螺仪量程
* @param    accel_fs:加速度计量程
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_set_gyro_accel_fs(IMU_GYRO_FS_SET_t gyro_fs, IMU_ACCEL_FS_SET_t accel_fs)
{
    uint8_t status = 1;
    uint16_t gyro_fs_d = 0, accel_fs_d = 0, reg_rd_tmp = 0;

    status &= imu_select_bank(BANK0);
    status &= imu_unlock(LOCK_TYPE_CLM);
    status &= imu_read_reg(0x5EU, &gyro_fs_d);    /* GYRO_FS寄存器地址0x5E */
    status &= imu_read_reg(0x5DU, &accel_fs_d);    /* ACCEL_FS寄存器地址0x5D */
    if(0U == status) {
        status &= imu_lock(LOCK_TYPE_CLM);
        return status;
    }
    gyro_fs_d &= ~(0xE000U | 0x1C00U | 0x380U);    /* 分别是XYZ陀螺仪量程bit在寄存器中的掩码(0xFF80) */
    accel_fs_d &=~(0x0380U | 0x0070U | 0x00EU);    /* 分别是XYZ加速度量程bit在寄存器中的掩码(0x03FE) */
    gyro_fs_d |= (( ((uint16_t)gyro_fs<<13U) | ((uint16_t)gyro_fs<<10U) | ((uint16_t)gyro_fs<<7U) ) & 0xFF80U);
    accel_fs_d |= (( ((uint16_t)accel_fs<<7U) | ((uint16_t)accel_fs<<4U) | ((uint16_t)accel_fs<<1U) ) & 0x03FEU);
    
    status &= imu_write_reg(0x5EU, gyro_fs_d);    /* 写入配置 */
    status &= imu_write_reg(0x5DU, accel_fs_d);
    
    status &= imu_read_reg(0x5EU, &reg_rd_tmp);    /* 检查值是否已经写入 */
    status &= (gyro_fs_d == reg_rd_tmp)? 1U: 0U;
    status &= imu_read_reg(0x5DU, &reg_rd_tmp);
    status &= (accel_fs_d == reg_rd_tmp)? 1U: 0U;

    status &= imu_lock(LOCK_TYPE_CLM);
    return status;
}


/************************************************************
* @brief    设置ODR(数据输出速率)
* @param    odr:数据输出速率
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_set_odr(IMU_ODR_SET_t odr)
{
    uint8_t status = 1;
    uint16_t odr_rd = 0, reg_rd_tmp = 0;

    status &= imu_select_bank(BANK0);
    status &= imu_read_reg(0x4FU, &odr_rd);    /* ODR_CFG寄存器地址0x4F */
    if(0U == status)
        return status;
    odr_rd &= ~0x000FU;    /* ODR对应bit在寄存器中的掩码(0x000F) */
    odr_rd |= ((uint16_t)odr & 0x000FU);
    
    status &= imu_write_reg(0x4FU, odr_rd);    /* 写入配置 */
    status &= imu_read_reg(0x4FU, &reg_rd_tmp);    /* 检查值是否已经写入 */
    status &= (odr_rd == reg_rd_tmp)? 1U: 0U;

    return status;
}


/************************************************************
* @brief    设置低通滤波
* @param    dlpf:低通滤波频率
* @retval    1：成功, 0：失败
************************************************************/
static uint8_t imu_set_dlpf(IMU_DLPF_SET_t dlpf)
{
    uint8_t status = 1;
    uint16_t gyro_dlpf_d = 0, accel_dlpf_d = 0, reg_rd_tmp = 0;

    status &= imu_select_bank(BANK0);
    status &= imu_unlock(LOCK_TYPE_CLM);
    status &= imu_read_reg(0x5CU, &gyro_dlpf_d);    /* DLPF_CFG_2寄存器(陀螺仪低通滤波设置)地址0x5C */
    status &= imu_read_reg(0x5BU, &accel_dlpf_d);    /* DLPF_CFG_1寄存器(加速度低通滤波设置)地址0x5B */
    if(0U == status) {
        status &= imu_lock(LOCK_TYPE_CLM);
        return status;
    }
    gyro_dlpf_d &= ~(0x01C0U | 0x0038U | 0x007U);    /* 分别是XYZ陀螺仪低通滤波bit在寄存器中的掩码(0xFF80) */
    accel_dlpf_d &=~(0x01C0U | 0x0038U | 0x007U);    /* 分别是XYZ加速度低通滤波bit在寄存器中的掩码(0x03FE) */
    gyro_dlpf_d |= (( ((uint16_t)dlpf<<6U) | ((uint16_t)dlpf<<3U) | ((uint16_t)dlpf<<0U) ) & 0x01FFU);
    accel_dlpf_d |= (( ((uint16_t)dlpf<<6U) | ((uint16_t)dlpf<<3U) | ((uint16_t)dlpf<<0U) ) & 0x01FFU);
    
    status &= imu_write_reg(0x5CU, gyro_dlpf_d);    /* 写入配置 */
    status &= imu_write_reg(0x5BU, accel_dlpf_d);
    
    status &= imu_read_reg(0x5CU, &reg_rd_tmp);    /* 检查值是否已经写入 */
    status &= (gyro_dlpf_d == reg_rd_tmp)? 1U: 0U;
    status &= imu_read_reg(0x5BU, &reg_rd_tmp);
    status &= (accel_dlpf_d == reg_rd_tmp)? 1U: 0U;

    status &= imu_lock(LOCK_TYPE_CLM);
    return status;
}


