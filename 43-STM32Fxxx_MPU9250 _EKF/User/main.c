/**
 * Attitude estimation on STM32F411 with MPU9250 + SSD1306 using Extended Kalman Filter.
 *
 *   I2C1: PB6 = SCL, PB7 = SDA (shared by both devices)
 *   LED:  PA5 (the only user LED on a Nucleo-64)
 *
 * Layering:
 *   main.c      board setup, timing, display  -- this file
 *   imu_port.c  the only file that knows tm_stm32 exists
 *   ekf.c       agnostic filter, compiles on a PC too
 *
 * @author    Fouad El khiati
 * @email     fouadelkh2@gmail.com
 * @ide       Keil uVision 5
 * @conf      PLL parameters are set in "Options for Target" -> "C/C++" -> "Defines"
 * @packs     STM32F4xx/STM32F7xx Keil packs are requred with HAL driver support And TM libraries
 * @stdperiph STM32F4xx/STM32F7xx HAL drivers required
 * blows the MDK-Lite 32 KB limit. Values are formatted by hand below.
 */
 
/* Include core modules */
#include "stm32fxxx_hal.h"

/* Include my libraries here */
#include "defines.h"
#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_ssd1306.h"
#include "tm_stm32_fonts.h"

/* Include my EKF application headers here */
#include "ekf.h"
#include "imu_port.h"

/* Set to 1 only after the magnetometer has been calibrated.
   With an uncalibrated mag the yaw correction fights the gyro and
   makes the estimate worse than leaving yaw to free-run. */
#define EKF_USE_MAG   0

#define RAD2DEG       57.29578f

static ekf_t        ekf;
static imu_sample_t sample;

/* ---------------------------------------------------------------- */
/* Fixed-point formatting, no stdio                                   */
/* ---------------------------------------------------------------- */

static char* putu(char *p, uint32_t v)
{
    char tmp[12];
    int8_t n = 0;

    if (v == 0) { *p++ = '0'; return p; }
    while (v) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n--) *p++ = tmp[n];
    return p;
}

/* Build "R:-12.34" */
static void fmt(char *dst, const char *label, float val)
{
    char *p = dst;
    uint32_t scaled, frac;

    while (*label) *p++ = *label++;

    if (val < 0.0f) { *p++ = '-'; val = -val; }
    if (!(val < 9999.0f)) val = 9999.0f;   /* also catches NaN and inf */

    scaled = (uint32_t)(val * 100.0f + 0.5f);

    p = putu(p, scaled / 100);
    *p++ = '.';
    frac = scaled % 100;
    *p++ = (char)('0' + (frac / 10));
    *p++ = (char)('0' + (frac % 10));
    *p   = '\0';
}

static void draw_line(int y, const char *label, float value)
{
    char buf[24];
    fmt(buf, label, value);
    TM_SSD1306_GotoXY(0, y);
    TM_SSD1306_Puts(buf, &TM_Font_7x10, SSD1306_COLOR_WHITE);
}

/* Fast blink forever -- unrecoverable init failure */
static void fail_blink(void)
{
    while (1) {
        TM_DISCO_LedToggle(LED_GREEN);
        Delayms(80);
    }
}

/* ---------------------------------------------------------------- */

int main(void)
{
    ekf_cfg_t cfg;
    float roll, pitch, yaw;

    /* --- Board bring-up. Order matters: TM_DELAY_Init must run before
       imu_port_init, since imu_port derives dt from TM_DELAY_Time. --- */
    TM_RCC_InitSystem();
    HAL_Init();
    TM_DELAY_Init();
    TM_DISCO_LedInit();

    Delayms(300);                  /* let both modules power up */

    /* SSD1306 first: its init brings up the shared I2C1 bus */
    if (!TM_SSD1306_Init()) {
        fail_blink();
    }

    TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
    TM_SSD1306_GotoXY(0, 20);
    TM_SSD1306_Puts("Starting...", &TM_Font_7x10, SSD1306_COLOR_WHITE);
    TM_SSD1306_UpdateScreen();

    if (!imu_port_init()) {
        TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
        TM_SSD1306_GotoXY(0, 20);
        TM_SSD1306_Puts("MPU9250 FAIL", &TM_Font_7x10, SSD1306_COLOR_WHITE);
        TM_SSD1306_UpdateScreen();
        while (1);
    }

    /* --- Filter tuning ---
       gyro_noise  raise if the estimate looks jittery
       bias_noise  raise if bias converges too slowly
       accel_noise raise if vibration pulls the attitude around */
    cfg.gyro_noise  = 0.01f;
    cfg.bias_noise  = 0.0005f;
    cfg.accel_noise = 0.05f;
    cfg.mag_noise   = 0.10f;
    cfg.mag_ref[0]  = 1.0f;        /* placeholder until calibrated */
    cfg.mag_ref[1]  = 0.0f;
    cfg.mag_ref[2]  = 0.0f;

    ekf_init(&ekf, &cfg);

    /* Steady LED = both devices up and the filter is running */
    TM_DISCO_LedOn(LED_GREEN);

    while (1) {

        if (imu_port_read(&sample)) {

            ekf_predict(&ekf, sample.gyro, sample.dt);

            /* Returns 0 and skips the update when the accelerometer is
               not dominated by gravity -- during linear acceleration
               the vector no longer points along g. */
            ekf_update_accel(&ekf, sample.acc);

#if EKF_USE_MAG
            ekf_update_mag(&ekf, sample.mag);
#endif

            ekf_euler(&ekf, &roll, &pitch, &yaw);

            TM_SSD1306_Fill(SSD1306_COLOR_BLACK);

            draw_line( 0, "R:", roll  * RAD2DEG);
            draw_line(10, "P:", pitch * RAD2DEG);
            draw_line(20, "Y:", yaw   * RAD2DEG);

            /* Gyro bias in deg/s -- should settle to small constants
               within a few seconds if the board is held still. */
            draw_line(30, "bx:", ekf.bias[0] * RAD2DEG);
            draw_line(40, "by:", ekf.bias[1] * RAD2DEG);
            draw_line(50, "bz:", ekf.bias[2] * RAD2DEG);

            TM_SSD1306_UpdateScreen();
        }

        Delayms(10);               /* roughly 100 Hz */
    }
}
