/**
******************************************************************************
* @file           : main.c
* @brief          : Button controlled LED blink with variable speed
******************************************************************************
* @attention
*
* Copyright (c) 2025 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/

#include <stdint.h>

#include "stm32f411xe.h"

/* GPIO Definitions */
#define GPIOAEN     (1U<<0)  // Enable GPIOA clock
#define GPIOCEN     (1U<<2)  // Enable GPIOC clock
#define LED_PIN     (1U<<5)  // PA5 - Onboard LED
#define BTN_PIN     (1U<<13) // PC13 - User button

/* Delay speeds */
#define MAX_SPEEDS  4        // Number of different speeds
#define BASE_DELAY  500000   // Slowest speed delay value

/* Function prototypes */
static void gpio_init(void);
static uint8_t is_button_pressed(void);
static void delay(uint32_t count);

int main(void)
{
    uint32_t current_delay = BASE_DELAY;
    uint8_t speed_level = 0;
    uint8_t button_was_pressed = 0;

    /* Initialize GPIO peripherals */
    gpio_init();

    while(1)
    {
        /* Check button state */
        if(is_button_pressed())
        {
            if(!button_was_pressed)  // Button just pressed
            {
                button_was_pressed = 1;
                /* Update speed level and delay */
                speed_level = (speed_level + 1) % MAX_SPEEDS;//Modulus acts as inhibitor
                current_delay = BASE_DELAY >> speed_level;  // Divide by 2^speed_level
            }
        }
        else
        {
            button_was_pressed = 0;  // Button released
        }

        /* Toggle LED */
        GPIOA->ODR ^= LED_PIN;
        delay(current_delay);
    }
}

/**
 * @brief Initialize GPIO peripherals
 * @retval None
 */
static void gpio_init(void)
{
    /* Enable GPIO Clocks */
    RCC->AHB1ENR |= GPIOAEN;  // Enable GPIOA clock
    RCC->AHB1ENR |= GPIOCEN;  // Enable GPIOC clock

    /* Configure PA5 (LED) as output */
    GPIOA->MODER |= (1U<<10);  // Set bit 10
    GPIOA->MODER &= ~(1U<<11); // Clear bit 11

    /* Configure PC13 (Button) as input */
    GPIOC->MODER &= ~(3U<<26); // Clear both bits for PC13
}

/**
 * @brief Check if button is pressed (active low)
 * @retval 1 if pressed, 0 if not pressed
 */
static uint8_t is_button_pressed(void)
{
    return !(GPIOC->IDR & BTN_PIN);  // Button is active low
}

/**
 * @brief Simple delay function
 * @param count: Number of cycles to delay
 * @type: Static, makes function only visible in this file
 * @type: Static, this allows us to use these function names in other files.
 */
static void delay(uint32_t count)
{
    for(uint32_t i = 0; i < count; i++)
    {
        __NOP();  // No operation - prevents optimization
    }
}
