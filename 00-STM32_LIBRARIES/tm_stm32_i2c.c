/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen MAJERLE
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32_i2c.h"

/* I2C2 AF fix for F0xx */
#if !defined(GPIO_AF4_I2C2) 
#define GPIO_AF4_I2C2   GPIO_AF1_I2C2
#endif

/* Timeout value: TM_I2C_TIMEOUT, see tm_stm32_i2c.h (overridable in defines.h) */

/* Handle values for I2C */
#ifdef I2C1
static I2C_HandleTypeDef I2C1Handle = {I2C1};
#endif
#ifdef I2C2
static I2C_HandleTypeDef I2C2Handle = {I2C2};
#endif
#ifdef I2C3
static I2C_HandleTypeDef I2C3Handle = {I2C3};
#endif
#ifdef I2C4
static I2C_HandleTypeDef I2C4Handle = {I2C4};
#endif

/* Private functions */
#ifdef I2C1
static void TM_I2C1_INT_InitPins(TM_I2C_PinsPack_t pinspack);
#endif
#ifdef I2C2
static void TM_I2C2_INT_InitPins(TM_I2C_PinsPack_t pinspack);
#endif
#ifdef I2C3
static void TM_I2C3_INT_InitPins(TM_I2C_PinsPack_t pinspack);
#endif
#ifdef I2C4
static void TM_I2C4_INT_InitPins(TM_I2C_PinsPack_t pinspack);
#endif

I2C_HandleTypeDef* TM_I2C_GetHandle(I2C_TypeDef* I2Cx) {
#ifdef I2C1
	if (I2Cx == I2C1) {
		return &I2C1Handle;
	}
#endif
#ifdef I2C2
	if (I2Cx == I2C2) {
		return &I2C2Handle;
	}
#endif
#ifdef I2C3
	if (I2Cx == I2C3) {
		return &I2C3Handle;
	}
#endif
#ifdef I2C4
	if (I2Cx == I2C4) {
		return &I2C4Handle;
	}
#endif
	
	/* Return invalid */
	return 0;
}

static void TM_I2C_FillSettings(I2C_HandleTypeDef* Handle, uint32_t clockSpeed) {
#if defined(STM32F7xx)
	uint32_t I2C_Timing;
	
	/* Future */
	if (clockSpeed >= 3400000) {
		/* 100kHz @ 50MHz APB clock */
		I2C_Timing = 0x40912732;
	} else if (clockSpeed >= 1000000) {
		/* 100kHz @ 50MHz APB clock */
		I2C_Timing = 0x40912732;
	} else if (clockSpeed >= 400000) {
		/* 100kHz @ 50MHz APB clock */
		I2C_Timing = 0x40912732;
	} else {	
		/* 100kHz @ 50MHz APB clock */
		I2C_Timing = 0x40912732;
	}
#endif
#if defined(STM32F0xx)
	/* 100kHz @ 48MHz APB clock */
	uint32_t I2C_Timing = 0x10805E89;
#endif

	/* Fill settings */
	Handle->Init.OwnAddress2 = 0x00;
	Handle->Init.OwnAddress1 = 0x00;
	Handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	Handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	Handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	Handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; 
#if defined(STM32F7xx) || defined(STM32F0xx)
	Handle->Init.Timing = I2C_Timing;
#else
	Handle->Init.ClockSpeed = clockSpeed;
	Handle->Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif
}

TM_I2C_Result_t TM_I2C_Init(I2C_TypeDef* I2Cx, TM_I2C_PinsPack_t pinspack, uint32_t clockSpeed) {	
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	
	/* Fill instance value */
	Handle->Instance = I2Cx;
	
#ifdef I2C1
	if (I2Cx == I2C1) {
		/* Enable clock */
		__HAL_RCC_I2C1_CLK_ENABLE();
		
		/* Enable pins */
		TM_I2C1_INT_InitPins(pinspack);
	}
#endif
#ifdef I2C2	
	if (I2Cx == I2C2) {
		/* Enable clock */
		__HAL_RCC_I2C2_CLK_ENABLE();
		
		/* Enable pins */
		TM_I2C2_INT_InitPins(pinspack);
	} 
#endif
#ifdef I2C3
	if (I2Cx == I2C3) {
		/* Enable clock */
		__HAL_RCC_I2C3_CLK_ENABLE();
		
		/* Enable pins */
		TM_I2C3_INT_InitPins(pinspack);
	}
#endif
#ifdef I2C4
	if (I2Cx == I2C4) {
		/* Enable clock */
		__HAL_RCC_I2C4_CLK_ENABLE();
		
		/* Enable pins */
		TM_I2C4_INT_InitPins(pinspack);
	}
#endif
	
	/* Fill settings */
	TM_I2C_FillSettings(Handle, clockSpeed);
	
	/* Initialize I2C */
	HAL_I2C_Init(Handle);
		
	/* Enable analog filter */
#if defined(I2C_ANALOGFILTER_ENABLE)
	HAL_I2CEx_ConfigAnalogFilter(Handle, I2C_ANALOGFILTER_ENABLE);
#endif
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_Read(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t* data) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	
	/* Send address */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &register_address, 1, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(Handle, device_address, data, 1, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	
	/* Send register address */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &register_address, 1, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(Handle, device_address, data, count, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_ReadNoRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t* data) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);

	/* Receive single byte without specifying  */
	if (HAL_I2C_Master_Receive(Handle, (uint16_t)device_address, data, 1, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t* data, uint16_t count) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);

	/* Receive multi bytes without specifying  */
	if (HAL_I2C_Master_Receive(Handle, (uint16_t)device_address, data, count, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_Write(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t data) {
	uint8_t d[2];
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
		
	/* Format array to send */
	d[0] = register_address;
	d[1] = data;
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, (uint8_t *)d, 2, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	} 
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t device_address, uint16_t register_address, uint8_t* data, uint16_t count) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);

	/* Try to transmit via I2C */
	if (HAL_I2C_Mem_Write(Handle, device_address, register_address, register_address > 0xFF ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, data, count, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t data) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &data, 1, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	} 
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_WriteMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t* data, uint16_t count) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, data, count, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	} 
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_Write16(I2C_TypeDef* I2Cx, uint8_t device_address, uint16_t register_address, uint8_t data) {
	uint8_t d[3];
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
		
	/* Format array to send */
	d[0] = (register_address >> 8) & 0xFF; /* High byte */
	d[1] = (register_address) & 0xFF;      /* Low byte */
	d[2] = data;                           /* Data byte */
	
	/* Try to transmit via I2C */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, (uint8_t *)d, 3, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	} 
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_Read16(I2C_TypeDef* I2Cx, uint8_t device_address, uint16_t register_address, uint8_t* data) {
	uint8_t adr[2];
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	
	/* Format I2C address */
	adr[0] = (register_address >> 8) & 0xFF; /* High byte */
	adr[1] = (register_address) & 0xFF;      /* Low byte */
	
	/* Send address */
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, adr, 2, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive(Handle, device_address, data, 1, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
			
		}
		
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t device_address) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	
	/* Check if device is ready for communication */
	if (HAL_I2C_IsDeviceReady(Handle, device_address, 2, 5) != HAL_OK) {
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

TM_I2C_Result_t TM_I2C_WriteReadRepeatedStart(
	I2C_TypeDef* I2Cx,
	uint8_t device_address, 
	uint8_t write_register_address, 
	uint8_t* write_data,
	uint16_t write_count, 
	uint8_t read_register_address, 
	uint8_t* read_data,
	uint16_t read_count
) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	
	/* Write command to device */
	if (HAL_I2C_Mem_Write(Handle, device_address, write_register_address, I2C_MEMADD_SIZE_8BIT, write_data, write_count, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Read data from controller */
	if (HAL_I2C_Mem_Read(Handle, device_address, read_register_address, I2C_MEMADD_SIZE_8BIT, read_data, read_count, TM_I2C_TIMEOUT) != HAL_OK) {
		/* Return error */
		return TM_I2C_Result_Error;
	}
	
	/* Return OK */
	return TM_I2C_Result_Ok;
}

/* ---------------------------------------------------------------- */
/* Bus recovery                                                       */
/* ---------------------------------------------------------------- */

/* Roughly 5 us busy wait. Kept local so this library does not depend
   on TM DELAY; precision does not matter, I2C slaves have no minimum
   clock frequency. */
static void TM_I2C_INT_HalfClock(void) {
	volatile uint32_t n = SystemCoreClock / 800000U + 1U;
	while (n--);
}

/* SCL and SDA pins for each pinspack, matching TM_I2Cx_INT_InitPins below. */
static uint8_t TM_I2C_INT_GetPins(I2C_TypeDef* I2Cx, TM_I2C_PinsPack_t pinspack,
                                  GPIO_TypeDef** sclPort, uint16_t* sclPin,
                                  GPIO_TypeDef** sdaPort, uint16_t* sdaPin) {
#ifdef I2C1
	if (I2Cx == I2C1) {
#if defined(GPIOB)
		if (pinspack == TM_I2C_PinsPack_1) { *sclPort = GPIOB; *sclPin = GPIO_PIN_6;  *sdaPort = GPIOB; *sdaPin = GPIO_PIN_7;  return 1; }
		if (pinspack == TM_I2C_PinsPack_2) { *sclPort = GPIOB; *sclPin = GPIO_PIN_8;  *sdaPort = GPIOB; *sdaPin = GPIO_PIN_9;  return 1; }
		if (pinspack == TM_I2C_PinsPack_3) { *sclPort = GPIOB; *sclPin = GPIO_PIN_6;  *sdaPort = GPIOB; *sdaPin = GPIO_PIN_9;  return 1; }
#endif
	}
#endif
#ifdef I2C2
	if (I2Cx == I2C2) {
#if defined(GPIOB)
		if (pinspack == TM_I2C_PinsPack_1) { *sclPort = GPIOB; *sclPin = GPIO_PIN_10; *sdaPort = GPIOB; *sdaPin = GPIO_PIN_11; return 1; }
#endif
#if defined(GPIOF)
		if (pinspack == TM_I2C_PinsPack_2) { *sclPort = GPIOF; *sclPin = GPIO_PIN_1;  *sdaPort = GPIOF; *sdaPin = GPIO_PIN_0;  return 1; }
#endif
#if defined(GPIOH)
		if (pinspack == TM_I2C_PinsPack_3) { *sclPort = GPIOH; *sclPin = GPIO_PIN_4;  *sdaPort = GPIOH; *sdaPin = GPIO_PIN_5;  return 1; }
#endif
	}
#endif
#ifdef I2C3
	if (I2Cx == I2C3) {
#if defined(GPIOA) && defined(GPIOC)
		if (pinspack == TM_I2C_PinsPack_1) { *sclPort = GPIOA; *sclPin = GPIO_PIN_8;  *sdaPort = GPIOC; *sdaPin = GPIO_PIN_9;  return 1; }
#endif
#if defined(GPIOH)
		if (pinspack == TM_I2C_PinsPack_2) { *sclPort = GPIOH; *sclPin = GPIO_PIN_7;  *sdaPort = GPIOH; *sdaPin = GPIO_PIN_8;  return 1; }
#endif
	}
#endif
#ifdef I2C4
	if (I2Cx == I2C4) {
#if defined(GPIOD)
		if (pinspack == TM_I2C_PinsPack_1) { *sclPort = GPIOD; *sclPin = GPIO_PIN_12; *sdaPort = GPIOD; *sdaPin = GPIO_PIN_13; return 1; }
#endif
#if defined(GPIOF)
		if (pinspack == TM_I2C_PinsPack_2) { *sclPort = GPIOF; *sclPin = GPIO_PIN_1;  *sdaPort = GPIOF; *sdaPin = GPIO_PIN_0;  return 1; }
		if (pinspack == TM_I2C_PinsPack_3) { *sclPort = GPIOF; *sclPin = GPIO_PIN_14; *sdaPort = GPIOF; *sdaPin = GPIO_PIN_15; return 1; }
#endif
#if defined(GPIOH)
		if (pinspack == TM_I2C_PinsPack_4) { *sclPort = GPIOH; *sclPin = GPIO_PIN_11; *sdaPort = GPIOH; *sdaPin = GPIO_PIN_12; return 1; }
#endif
#if defined(GPIOB) && defined(GPIOD)
		if (pinspack == TM_I2C_PinsPack_5) { *sclPort = GPIOD; *sclPin = GPIO_PIN_12; *sdaPort = GPIOB; *sdaPin = GPIO_PIN_7;  return 1; }
#endif
	}
#endif
	return 0;
}

/* Pulse the peripheral's RCC reset line. Clears a BUSY flag that a
   glitch on the lines can leave latched (STM32F4 errata 2.9.7). */
static void TM_I2C_INT_ResetPeripheral(I2C_TypeDef* I2Cx) {
#ifdef I2C1
	if (I2Cx == I2C1) { __HAL_RCC_I2C1_FORCE_RESET(); __HAL_RCC_I2C1_RELEASE_RESET(); }
#endif
#ifdef I2C2
	if (I2Cx == I2C2) { __HAL_RCC_I2C2_FORCE_RESET(); __HAL_RCC_I2C2_RELEASE_RESET(); }
#endif
#ifdef I2C3
	if (I2Cx == I2C3) { __HAL_RCC_I2C3_FORCE_RESET(); __HAL_RCC_I2C3_RELEASE_RESET(); }
#endif
#ifdef I2C4
	if (I2Cx == I2C4) { __HAL_RCC_I2C4_FORCE_RESET(); __HAL_RCC_I2C4_RELEASE_RESET(); }
#endif
}

TM_I2C_Result_t TM_I2C_BusRecover(I2C_TypeDef* I2Cx, TM_I2C_PinsPack_t pinspack, uint32_t clockSpeed) {
	I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);
	GPIO_TypeDef *sclPort, *sdaPort;
	uint16_t sclPin, sdaPin;
	uint8_t i, released;
	
	if (Handle == 0 || !TM_I2C_INT_GetPins(I2Cx, pinspack, &sclPort, &sclPin, &sdaPort, &sdaPin)) {
		return TM_I2C_Result_Error;
	}
	
	/* Stop the peripheral driving the lines */
	HAL_I2C_DeInit(Handle);
	
	/* Take both lines as open-drain GPIO outputs, released (high) */
	HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);
	TM_GPIO_Init(sclPort, sclPin, TM_GPIO_Mode_OUT, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(sdaPort, sdaPin, TM_GPIO_Mode_OUT, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_I2C_INT_HalfClock();
	
	/* A slave stuck mid-byte holds SDA low and waits for clocks.
	   Nine clocks are enough to finish any byte plus its ACK. */
	for (i = 0; i < 9 && HAL_GPIO_ReadPin(sdaPort, sdaPin) == GPIO_PIN_RESET; i++) {
		HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_RESET);
		TM_I2C_INT_HalfClock();
		HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
		TM_I2C_INT_HalfClock();
	}
	
	/* STOP condition: SDA rises while SCL is high */
	HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_RESET);
	TM_I2C_INT_HalfClock();
	HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_RESET);
	TM_I2C_INT_HalfClock();
	HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);
	TM_I2C_INT_HalfClock();
	HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);
	TM_I2C_INT_HalfClock();
	
	released = HAL_GPIO_ReadPin(sclPort, sclPin) == GPIO_PIN_SET &&
	           HAL_GPIO_ReadPin(sdaPort, sdaPin) == GPIO_PIN_SET;
	
	/* Reset the peripheral and give the pins back to it */
	TM_I2C_INT_ResetPeripheral(I2Cx);
	TM_I2C_Init(I2Cx, pinspack, clockSpeed);
	
	return released ? TM_I2C_Result_Ok : TM_I2C_Result_Error;
}

__weak void TM_I2C_InitCustomPinsCallback(I2C_TypeDef* I2Cx, uint16_t AlternateFunction) {
	/* Custom user function. */
	/* In case user needs functionality for custom pins, this function should be declared outside this library */
}

/* Private functions */
#ifdef I2C1
static void TM_I2C1_INT_InitPins(TM_I2C_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOB)
	if (pinspack == TM_I2C_PinsPack_1) {
		TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C1);
	}
#endif
#if defined(GPIOB)
	if (pinspack == TM_I2C_PinsPack_2) {
		TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C1);
	}
#endif
#if defined(GPIOB)
	if (pinspack == TM_I2C_PinsPack_3) {
		TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_6 | GPIO_PIN_9, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C1);
	}
#endif
	if (pinspack == TM_I2C_PinsPack_Custom) {
		/* Init custom pins, callback function */
		TM_I2C_InitCustomPinsCallback(I2C1, GPIO_AF4_I2C1);
	}
}
#endif
#ifdef I2C2
static void TM_I2C2_INT_InitPins(TM_I2C_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOB)
	if (pinspack == TM_I2C_PinsPack_1) {
		TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C2);
	}
#endif
#if defined(GPIOF)
	if (pinspack == TM_I2C_PinsPack_2) {
		TM_GPIO_InitAlternate(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C2);
	}
#endif
#if defined(GPIOH)
	if (pinspack == TM_I2C_PinsPack_3) {
		TM_GPIO_InitAlternate(GPIOH, GPIO_PIN_4 | GPIO_PIN_5, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C2);
	}
#endif
	if (pinspack == TM_I2C_PinsPack_Custom) {
		/* Init custom pins, callback function */
		TM_I2C_InitCustomPinsCallback(I2C2, GPIO_AF4_I2C2);
	}
}
#endif
#ifdef I2C3
static void TM_I2C3_INT_InitPins(TM_I2C_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOA) && defined(GPIOC)
	if (pinspack == TM_I2C_PinsPack_1) {
		TM_GPIO_InitAlternate(GPIOA, GPIO_PIN_8, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C3);
		TM_GPIO_InitAlternate(GPIOC, GPIO_PIN_9, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C3);
	}
#endif
#if defined(GPIOH)
	if (pinspack == TM_I2C_PinsPack_2) {
		TM_GPIO_InitAlternate(GPIOH, GPIO_PIN_7 | GPIO_PIN_8, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C3);
	}
#endif
	if (pinspack == TM_I2C_PinsPack_Custom) {
		/* Init custom pins, callback function */
		TM_I2C_InitCustomPinsCallback(I2C3, GPIO_AF4_I2C3);
	}
}
#endif
#ifdef I2C4
static void TM_I2C4_INT_InitPins(TM_I2C_PinsPack_t pinspack) {
	/* Init pins */
#if defined(GPIOD)
	if (pinspack == TM_I2C_PinsPack_1) {
		TM_GPIO_InitAlternate(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C4);
	}
#endif
#if defined(GPIOF)
	if (pinspack == TM_I2C_PinsPack_2) {
		TM_GPIO_InitAlternate(GPIOF, GPIO_PIN_1 | GPIO_PIN_0, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C4);
	}
	if (pinspack == TM_I2C_PinsPack_3) {
		TM_GPIO_InitAlternate(GPIOF, GPIO_PIN_14 | GPIO_PIN_15, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C4);
	}
#endif
#if defined(GPIOH)
	if (pinspack == TM_I2C_PinsPack_4) {
		TM_GPIO_InitAlternate(GPIOH, GPIO_PIN_11 | GPIO_PIN_12, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C4);
	}
#endif
#if defined(GPIOB) && defined(GPIOD)
	if (pinspack == TM_I2C_PinsPack_5) {
#if defined(GPIO_AF11_I2C4)
		TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_7, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF11_I2C4);
#else
		TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_7, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C4);
#endif
        TM_GPIO_InitAlternate(GPIOD, GPIO_PIN_12, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF4_I2C4);
	}    
#endif
	/* Init pins */
	if (pinspack == TM_I2C_PinsPack_Custom) {
		/* Init custom pins, callback function */
		TM_I2C_InitCustomPinsCallback(I2C3, GPIO_AF4_I2C4);
	}
}
#endif
