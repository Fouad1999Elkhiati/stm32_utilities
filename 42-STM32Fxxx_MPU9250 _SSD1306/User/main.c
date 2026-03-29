/**
 * Keil project example for MPU9250
 *
 * Before you start, select your target, on the right of the "Load" button
 *
 * @author    Tilen MAJERLE
 * @email     tilen@majerle.eu
 * @website   http://stm32f4-discovery.net
 * @ide       Keil uVision 5
 * @conf      PLL parameters are set in "Options for Target" -> "C/C++" -> "Defines"
 * @packs     STM32F4xx/STM32F7xx Keil packs are requred with HAL driver support
 * @stdperiph STM32F4xx/STM32F7xx HAL drivers required
 */
/* Include core modules */
#include "stm32fxxx_hal.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_mpu9250.h"
#include "tm_stm32_usart.h"
#include "tm_stm32_exti.h"
#include "tm_stm32_ahrs_imu.h"
#include "tm_stm32_ssd1306.h"
#include "tm_stm32_fonts.h"
#include "stdio.h"

TM_MPU9250_t MPU9250;

TM_AHRSIMU_t IMU;

int main(void) {
	/* Initialize system */
	//SystemInit();
    
	/* Init system clock for maximum system speed */
	TM_RCC_InitSystem();
	
	HAL_Init();
	
	/* Delay init */
	TM_DELAY_Init();
    
    /* Init LEDs */
    TM_DISCO_LedInit();
    
    /* Init USART, TX = PA2, RX = PA3 */
    TM_USART_Init(USART2, TM_USART_PinsPack_1, 921600);
	
    /* Init MPU9250 */
    if (TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0) != TM_MPU9250_Result_Ok) {
        printf("Device error!\r\n");
        while (1);
    }
    
    printf("Device connected!\r\n");
    
    TM_EXTI_Attach(GPIOA, GPIO_PIN_0, TM_EXTI_Trigger_Rising);
    
    TM_AHRSIMU_Init(&IMU, 1000, 0.5, 0);
	
	/* Init SSD1306 LCD 128 x 64 px */
	if (TM_SSD1306_Init()) {
		/* SSD1306 is connected */
		TM_DISCO_LedOn(LED_GREEN);
	} else {
		/* SSD1306 is not connected */
		while (1) {
			/* Toggle all leds */
			TM_DISCO_LedToggle(LED_ALL);
			
			/* Delay a little */
			Delayms(50);
		}
	}
	
		char AcceX[50];
		char AcceY[50];
		char AcceZ[50];
	
		char GyrX[50];
		char GyrY[50];
		char GyrZ[50];
	
    while (1) {
        if (TM_MPU9250_DataReady(&MPU9250) == TM_MPU9250_Result_Ok) {
            TM_MPU9250_ReadAcce(&MPU9250);
            TM_MPU9250_ReadGyro(&MPU9250);
            TM_MPU9250_ReadMag(&MPU9250);
            
            TM_AHRSIMU_UpdateIMU(&IMU, MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, MPU9250.Ax, MPU9250.Ay, MPU9250.Az);
            
						TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
						
						// Using snprintf (safer - specifies buffer size)
						// Draw acceleration 
						snprintf(AcceX, sizeof(AcceX), "AcceX =%.2f", MPU9250.Ax);
						snprintf(AcceY, sizeof(AcceY), "AcceY =%.2f", MPU9250.Ay);
						snprintf(AcceZ, sizeof(AcceZ), "AcceZ =%.2f", MPU9250.Az);
						
						//Go to location X = 30, Y = 4 */
					  TM_SSD1306_GotoXY(10, 4);
						TM_SSD1306_Puts(AcceX, &TM_Font_7x10, SSD1306_COLOR_WHITE);
						
						//Go to location X = 20, Y = 25 */
						TM_SSD1306_GotoXY(8, 25);
						TM_SSD1306_Puts(AcceY, &TM_Font_7x10, SSD1306_COLOR_WHITE);
						
						//Go to location X = 15, Y = 45 */
						TM_SSD1306_GotoXY(15, 45);
						TM_SSD1306_Puts(AcceZ, &TM_Font_7x10, SSD1306_COLOR_WHITE);
						
						//Draw gyroscope data
						snprintf(GyrX, sizeof(GyrX), "GyrX =%.2f", MPU9250.Gx);
						snprintf(GyrY, sizeof(GyrY), "GyrY =%.2f", MPU9250.Gy);
						snprintf(GyrZ, sizeof(GyrZ), "GyrZ =%.2f", MPU9250.Gz);
						
						//Go to location X = 20, Y = 4 */
						TM_SSD1306_GotoXY(20, 4);
						TM_SSD1306_Puts(GyrX, &TM_Font_7x10, SSD1306_COLOR_WHITE);
						
						//Go to location X = 18, Y = 25 */
						TM_SSD1306_GotoXY(18, 25);
						TM_SSD1306_Puts(GyrY, &TM_Font_7x10, SSD1306_COLOR_WHITE);
						
						//Go to location X = 25, Y = 45 */
						TM_SSD1306_GotoXY(25, 45);
						TM_SSD1306_Puts(GyrZ, &TM_Font_7x10, SSD1306_COLOR_WHITE);
						
						TM_SSD1306_UpdateScreen();
						HAL_Delay(100);
            //printf("R: %f, P: %f, Y: %f\n", IMU.Roll, IMU.Pitch, IMU.Yaw);
            //
						
						TM_SSD1306_UpdateScreen();
						HAL_Delay(100);

 /*         //rintf("Ax: %f, Ay: %f, Az: %f, Gx: %f, Gy: %f, Gz: %f\n", 
                MPU9250.Ax, MPU9250.Ay, MPU9250.Az,
                MPU9250.Gx, MPU9250.Gy, MPU9250.Gz
            );
            printf("Mx: %d, My: %d, Mz: %d\r\n",
                MPU9250.Mx, MPU9250.My, MPU9250.Mz
            ); */
        }
	}
}

/* Printf handler */
int fputc(int ch, FILE* fil) {
    TM_USART_Putc(USART2, ch);
    
    return ch;
}

/* EXTI handler */

void TM_EXTI_Handler(uint16_t GPIO_Pin) {
    TM_DISCO_LedToggle(LED_ALL);
}