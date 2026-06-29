#ifndef    __IMU_DRIVER_H__
#define    __IMU_DRIVER_H__

/********************************************************************************
* header file
********************************************************************************/



/********************************************************************************
* macro definiton
********************************************************************************/
#define IMU_SPI_CS        CGQ_CS_IMU1    /* SPI通信接口的片选CS */


/********************************************************************************
* global variable
********************************************************************************/
extern uint8_t (*imu_spi_Interface)(uint8_t CS, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);    /* SPI通信接口 */
extern void (*imu_delay_ms)(uint16_t count);    /* 延时函数接口 */

typedef struct {
  uint8_t SM02_GYRO_X_Drvratio;    /* X轴陀螺仪 */
  uint8_t SM03_GYRO_X_Drfreqmeas;    /* X轴陀螺仪 */
  uint8_t SM04_GYRO_X_Quadadc;    /* X轴陀螺仪 */
  uint8_t SM07_GYRO_X_Drclk;    /* X轴陀螺仪 */
  uint8_t SM20_GYRO_X_Alarm;    /* X轴陀螺仪 */
  uint8_t SM36_GYRO_X_St;    /* X轴陀螺仪 */

  uint8_t SM02_GYRO_Y_Drvratio;    /* Y轴陀螺仪 */
  uint8_t SM03_GYRO_Y_Drfreqmeas;    /* Y轴陀螺仪 */
  uint8_t SM04_GYRO_Y_Quadadc;    /* Y轴陀螺仪 */
  uint8_t SM07_GYRO_Y_Drclk;    /* Y轴陀螺仪 */
  uint8_t SM20_GYRO_Y_Alarm;    /* Y轴陀螺仪 */
  uint8_t SM36_GYRO_Y_St;    /* Y轴陀螺仪 */

  uint8_t SM02_GYRO_Z_Drvratio;    /* Z轴陀螺仪 */
  uint8_t SM03_GYRO_Z_Drfreqmeas;    /* Z轴陀螺仪 */
  uint8_t SM04_GYRO_Z_Quadadc;    /* Z轴陀螺仪 */
  uint8_t SM07_GYRO_Z_Drclk;    /* Z轴陀螺仪 */
  uint8_t SM20_GYRO_Z_Alarm;    /* Z轴陀螺仪 */
  uint8_t SM36_GYRO_Z_St;    /* Z轴陀螺仪 */

  uint8_t SM19_ACCEL_X_Alarm;    /* X轴加速度计 */
  uint8_t SM25_ACCEL_X_C2V;    /* X轴加速度计 */
  uint8_t SM19_MGA_X_Alarm;    /* X轴加速度计 */
  uint8_t SM16_ACCEL_X_St;    /* X轴加速度计 */

  uint8_t SM19_ACCEL_Y_Alarm;    /* Y轴加速度计 */
  uint8_t SM25_ACCEL_Y_C2V;    /* Y轴加速度计 */
  uint8_t SM19_MGA_Y_Alarm;    /* Y轴加速度计 */
  uint8_t SM16_ACCEL_Y_St;    /* Y轴加速度计 */

  uint8_t SM19_ACCEL_Z_Alarm;    /* Z轴加速度计 */
  uint8_t SM25_ACCEL_Z_C2V;    /* Z轴加速度计 */
  uint8_t SM19_MGA_Z_Alarm;    /* Z轴加速度计 */
  uint8_t SM16_ACCEL_Z_St;    /* Z轴加速度计 */
  uint8_t SM24_ACCEL_Z_Vrefsh_Charge;    /* Z轴加速度计 */
  uint8_t SM17_ACCEL_Z_Vrefshieldz;    /* Z轴加速度计 */

  uint8_t SM30_Spi_Clkcnt;    /* SPI通信 */
  uint8_t SM30_Spi_Crc;    /* SPI通信 */

  uint8_t SM18_ACCEL_Cp;    /* COMMON */
  uint8_t SM14_GYRO_Vref;    /* COMMON */
  uint8_t SM22_Ahb_Eccerr;    /* COMMON */
  uint8_t SM11_Avdd;    /* COMMON */
  uint8_t SM34_Bg;    /* COMMON */
  uint8_t SM14_Cp_Vref;    /* COMMON */
  uint8_t SM12_Dvdd;    /* COMMON */
  uint8_t SM06_GYRO_Cp25;    /* COMMON */
  uint8_t SM06_GYRO_Cp5;    /* COMMON */
  uint8_t SM14_ACCEL_Vref;    /* COMMON */
  uint8_t SM22_Otp_Eccerr;    /* COMMON */
  uint8_t SM03_Rcosc1_Freqmeas;    /* COMMON */

  uint8_t SM03_Rcosc2_Freqmeas;    /* COMMON */
  uint8_t SM52_GYRO_Cavity_Check;    /* COMMON */
  uint8_t SM30_Spi_Encdecod;    /* COMMON */
  uint8_t SM03_Sysclk_Freqmeas;    /* COMMON */
  uint8_t SM08_Temp_Dsp;    /* COMMON */
  uint8_t SM14_Temp_Vref;    /* COMMON */
  uint8_t SM08_Temp12;    /* COMMON */
  uint8_t SM13_Vdd;    /* COMMON */
  uint8_t SM13_Vddio;    /* COMMON */
  uint8_t SM10_Vddmaster;    /* COMMON */
  uint8_t SM17_Vrefshieldxy;    /* COMMON */
  uint8_t SM33_Reg_Crc;    /* COMMON */
  uint8_t SM33_Reg_Parity;    /* COMMON */

  uint8_t SM26_Otp_Cpy;    /* COMMON */
  uint8_t SM26_Otp_Crc;    /* COMMON */
  uint8_t SM26_Otp_Reg;    /* COMMON */
  uint8_t SM29_Ahb_Bus;    /* COMMON */
}

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
