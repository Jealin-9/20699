#ifndef    __IMU_DRIVER_H__
#define    __IMU_DRIVER_H__

/********************************************************************************
* header file
********************************************************************************/
#include "BIET7A1_Driver_HL.h"


/********************************************************************************
* macro definiton
********************************************************************************/
#define IMU_SPI_CS        CGQ_CS_IMU1    /* SPI通信接口的片选CS */


/********************************************************************************
* global variable
********************************************************************************/
extern uint8_t (*imu_spi_Interface)(uint8_t CS, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);    /* SPI通信接口 */
extern void (*imu_delay_ms)(uint16_t count);    /* 延时函数接口 */


/********************************************************************************
* function declaration
********************************************************************************/
uint8_t imu_init_config(voi);    /* IMU初始化配置 */
uint8_t imu_communication_test(uint16_t wdata, uint16_t *rdata);    /* IMU通信测试 */
uint8_t imu_R_data(uint16_t gyroData[], uint16_t accData[], uint16_t *tempData);    /* 读取IMU六轴数据及温度数据 */
uint8_t imu_fault_overview(uint8_t *fault_code);    /* 错误检测概览 */
uint8_t imu_temp_detection(uint8_t *fault_code);    /* 温度错误检测 */
uint8_t imu_range_detection(uint8_t *fault_code);    /* 范围错误检测 */
uint8_t imu_fault_inject(uint8_t fault_code);    /* 错误注入(用来测试) */
uint8_t imu_functional_safety_diagnosis(SM_Alarms_Struct *SM_code);    /* 功能安全检测 */

uint8_t imu_temp_comp(uint16_t gyro_tc[], uint16_t gyro_qtc[], uint16_t *gyro_to, uint16_t acc_tc[], uint16_t acc_qtc[], uint16_t *acc_to);    /* IMU温度补偿 */
uint8_t imu_R_TempComp_Reg(uint16_t gyro_tc[], uint16_t gyro_qtc[], uint16_t *gyro_to, uint16_t acc_tc[], uint16_t acc_qtc[], uint16_t *acc_to);    /* 读取IMU温度补偿数据 */




#endif
