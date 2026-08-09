/**
 * MPU9250 + SSD1306 over I2C1 (PB6 = SCL, PB7 = SDA)
 *
 * Board:  STM32F411RE (Nucleo-64), LED on PA5
 * Target: must define STM32F411xE, USE_HAL_DRIVER, NUCLEO_F411,
 *         RCC_OSCILLATORTYPE=RCC_OSCILLATORTYPE_HSI,
 *         RCC_PLLM=16, RCC_PLLN=400, RCC_PLLP=4, RCC_PLLQ=7
 *
 * No stdio, no printf, no floating-point formatter -- keeps the image
 * under the 32 KB MDK-Lite linker limit.
 */

#include "stm32fxxx_hal.h"
#include "defines.h"
#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_mpu9250.h"
#include "tm_stm32_ssd1306.h"
#include "tm_stm32_fonts.h"

TM_MPU9250_t MPU9250;

/* Append an unsigned integer to a string, return the new end pointer */
static char* putu(char *p, uint32_t v) {
    char tmp[12];
    int8_t n = 0;

    if (v == 0) {
        *p++ = '0';
        return p;
    }
    while (v) {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (n--) {
        *p++ = tmp[n];
    }
    return p;
}

/* Build a string like "Ax:-1.23" without stdio */
static void fmt(char *dst, const char *label, float val) {
    char *p = dst;
    uint32_t scaled, frac;

    while (*label) {
        *p++ = *label++;
    }

    if (val < 0.0f) {
        *p++ = '-';
        val = -val;
    }

    scaled = (uint32_t)(val * 100.0f + 0.5f);

    p = putu(p, scaled / 100);
    *p++ = '.';

    frac = scaled % 100;
    *p++ = (char)('0' + (frac / 10));
    *p++ = (char)('0' + (frac % 10));
    *p   = '\0';
}

int main(void) {
    char buf[20];

    TM_RCC_InitSystem();
    HAL_Init();
    TM_DELAY_Init();
    TM_DISCO_LedInit();

    /* Let both modules finish powering up */
    Delayms(300);

    /* SSD1306 first -- its init brings up the shared I2C1 bus */
    if (!TM_SSD1306_Init()) {
        /* Fast blink on PA5 = display did not acknowledge on I2C */
        while (1) {
            TM_DISCO_LedToggle(LED_GREEN);
            Delayms(80);
        }
    }

    if (TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0) != TM_MPU9250_Result_Ok) {
        /* Display works at this point, so report the failure on screen */
        TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
        TM_SSD1306_GotoXY(0, 20);
        TM_SSD1306_Puts("MPU9250 FAIL", &TM_Font_7x10, SSD1306_COLOR_WHITE);
        TM_SSD1306_UpdateScreen();
        while (1);
    }

    /* Steady PA5 = both devices initialised */
    TM_DISCO_LedOn(LED_GREEN);

    while (1) {
        /* Read directly -- no dependency on the INT pin being wired */
        TM_MPU9250_ReadAcce(&MPU9250);
        TM_MPU9250_ReadGyro(&MPU9250);

        TM_SSD1306_Fill(SSD1306_COLOR_BLACK);

        fmt(buf, "Ax:", MPU9250.Ax);
        TM_SSD1306_GotoXY(0, 0);
        TM_SSD1306_Puts(buf, &TM_Font_7x10, SSD1306_COLOR_WHITE);

        fmt(buf, "Ay:", MPU9250.Ay);
        TM_SSD1306_GotoXY(0, 12);
        TM_SSD1306_Puts(buf, &TM_Font_7x10, SSD1306_COLOR_WHITE);

        fmt(buf, "Az:", MPU9250.Az);
        TM_SSD1306_GotoXY(0, 24);
        TM_SSD1306_Puts(buf, &TM_Font_7x10, SSD1306_COLOR_WHITE);

        fmt(buf, "Gx:", MPU9250.Gx);
        TM_SSD1306_GotoXY(0, 36);
        TM_SSD1306_Puts(buf, &TM_Font_7x10, SSD1306_COLOR_WHITE);

        fmt(buf, "Gy:", MPU9250.Gy);
        TM_SSD1306_GotoXY(0, 48);
        TM_SSD1306_Puts(buf, &TM_Font_7x10, SSD1306_COLOR_WHITE);

        TM_SSD1306_UpdateScreen();
        Delayms(100);
    }
}