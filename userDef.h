/*******************************************************************************
*
* @file userDef.h   evsoc
*
* @brief Header file defines user definition on specific designs. 
*
*******************************************************************************/
#include "soc.h"

#pragma once


//I2C 0
#ifdef SYSTEM_I2C_0_IO_CTRL
    #define I2C_CTRL SYSTEM_I2C_0_IO_CTRL
    #define I2C_CTRL_PLIC_INTERRUPT SYSTEM_PLIC_SYSTEM_I2C_0_IO_INTERRUPT
#endif

//I2C 1
#ifdef SYSTEM_I2C_1_IO_CTRL
    #define I2C_CTRL_1 SYSTEM_I2C_1_IO_CTRL
    #define I2C_CTRL_PLIC_INTERRUPT_1 SYSTEM_PLIC_SYSTEM_I2C_1_IO_INTERRUPT
#endif

//I2C 2
#ifdef SYSTEM_I2C_2_IO_CTRL
    #define I2C_CTRL_2 SYSTEM_I2C_2_IO_CTRL
    #define I2C_CTRL_PLIC_INTERRUPT_2 SYSTEM_PLIC_SYSTEM_I2C_2_IO_INTERRUPT
#endif

//GPIO 1
#ifdef SYSTEM_GPIO_1_IO_CTRL
    #define GPIO1 SYSTEM_GPIO_1_IO_CTRL
#endif

#define I2C_CTRL_HZ SYSTEM_CLINT_HZ
#define BSP_MACHINE_TIMER_HZ BSP_CLINT_HZ
#define SPI SYSTEM_SPI_0_IO_CTRL
