/**
 * @author  Fouad El khiati
 * @website http://stm32f4-discovery.net
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   MPU9250 hardware adaptation layer for the attitude EKF on STM32Fxxx
 *
@verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen MAJERLE

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef IMU_PORT_H
#define IMU_PORT_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup IMU_PORT
 * @brief    Hardware adaptation layer turning MPU9250 register reads into calibrated SI quantities
 * @{
 *
 * This is the only file in this project that includes a tm_stm32 header.
 * It works on STM32F0xx, STM32F4xx and STM32F7xx devices.
 *
 * \par How it works
 *
 * Reads gyroscope, accelerometer and magnetometer from the MPU9250, converts them to
 * the units the \ref EKF expects and applies the \ref MAGCAL calibration to the magnetometer.
 * Porting to a different MCU or IMU means rewriting this file and nothing else.
 *
 * \par Initialization order
 *
 * The MPU9250 and the SSD1306 display share I2C1, and the display's init is what brings the bus up.
 * Call \ref imu_port_init after TM_DELAY_Init and after TM_SSD1306_Init.
 *
 * \par Magnetometer calibration lifecycle
 *
\verbatim
    imu_port_cal_begin();
    while (collecting) imu_port_cal_feed();   //Rotate the board in all axes
    imu_port_cal_finish();
\endverbatim
 *
 * \par Changelog
 *
\verbatim
 Version 1.0
  - First release
\endverbatim
 *
 * \par Dependencies
 *
\verbatim
 - TM MPU9250
 - TM DELAY
 - MAGCAL
\endverbatim
 */

/**
 * @defgroup IMU_PORT_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  One converted IMU sample
 */
typedef struct {
    float gyro[3];    /*!< Gyro data for X, Y and Z axis, in units of rad/s */
    float acc[3];     /*!< Accelerometer data for X, Y and Z axis, in units of gees */
    float mag[3];     /*!< Calibrated magnetic unit vector, or raw data if uncalibrated */
    float dt;         /*!< Seconds since the previous sample */
    int   mag_valid;  /*!< 0 until a calibration has been solved, then 1 */
} imu_sample_t;

/**
 * @}
 */

/**
 * @defgroup IMU_PORT_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Brings up the MPU9250.
 * @note   Call after TM_DELAY_Init and after TM_SSD1306_Init, both devices share I2C1.
 * @param  None
 * @retval Initialization status:
 *            - 1: Success
 *            - 0: MPU9250 did not respond
 */
int imu_port_init(void);

/**
 * @brief  Reads all three sensors and fills the sample structure.
 * @param  *s: Pointer to \ref imu_sample_t structure to fill
 * @retval Sample status:
 *            - 1: Sample is valid
 *            - 0: dt was implausible, the sample should be skipped rather than fed forward
 */
int imu_port_read(imu_sample_t *s);

/**
 * @brief  Clears the calibration accumulators and begins collecting.
 * @param  None
 * @retval None
 */
void imu_port_cal_begin(void);

/**
 * @brief  Reads one magnetometer sample and folds it into the calibration.
 * @note   Rotate the board through as many orientations as possible while this is being called.
 * @param  None
 * @retval Running sample count
 */
int imu_port_cal_feed(void);

/**
 * @brief  Solves the fit and starts applying it.
 * @param  None
 * @retval Calibration status:
 *            - 1: Success, calibration is applied by \ref imu_port_read
 *            - 0: Failure, calibration stays inactive and mag_valid remains 0
 */
int imu_port_cal_finish(void);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* IMU_PORT_H */
