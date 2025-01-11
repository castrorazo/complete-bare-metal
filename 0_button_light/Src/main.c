/**
******************************************************************************
* @file           : main.c
* @brief          : Advanced button controlled LED patterns using Timer
******************************************************************************
*/

#include <stdint.h>
#include "stm32f411xe.h"

/* GPIO and Peripheral Definitions */
#define GPIOAEN         (1U<<0)     // Enable GPIOA clock
#define GPIOCEN         (1U<<2)     // Enable GPIOC clock
#define TIM2EN          (1U<<0)     // Enable TIM2 clock
#define USART2EN        (1U<<17)    // Enable USART2 clock
#define LED_PIN         (1U<<5)     // PA5 - Onboard LED
#define BTN_PIN         (1U<<13)    // PC13 - User button

/* UART Settings */
#define UART_BAUDRATE   115200
#define SYS_FREQ        16000000
#define APB1_CLK        SYS_FREQ
#define UART_BRR_VAL    ((APB1_CLK + (UART_BAUDRATE/2U))/UART_BAUDRATE)

/* Timer Settings
 * System clock = 16MHz
 * Prescaler = 16000 - 1 gives 1kHz timer clock (1ms period)
 * Auto-reload = desired period in ms
 */
#define TIM2_PSC        15999       // Prescaler for 1ms ticks
#define LED_PERIOD_MS   500         // Base LED blink period (500ms)
#define LONG_PRESS_MS   1000        // Long press detection time (1s)

/* Pattern Types */
typedef enum {
    PATTERN_STEADY,     // Regular on/off blinking
    PATTERN_BREATH,     // Gradual brightness change
    PATTERN_MORSE_SOS,  // SOS morse code pattern
    PATTERN_DOUBLE,     // Double-blink pattern
    MAX_PATTERNS
} BlinkPattern;

/* Button States */
typedef enum {
    BUTTON_IDLE,
    BUTTON_PRESSED,
    BUTTON_LONG_PRESS,
    BUTTON_RELEASED
} ButtonState;

/* Global variables */
static volatile uint32_t g_ms_ticks = 0;
static volatile uint32_t g_press_start = 0;
static BlinkPattern g_current_pattern = PATTERN_STEADY;
static uint8_t g_speed_level = 0;

/* Function prototypes */
static void gpio_init(void);
static void timer_init(void);
static void uart_init(void);
static void uart_write(const char* str);
static void uart_write_num(uint32_t num);
static ButtonState check_button(void);
static void pattern_steady(uint32_t period_ms);
static void pattern_breath(uint32_t period_ms);
static void pattern_morse_sos(uint32_t period_ms);
static void pattern_double(uint32_t period_ms);
static void error_indication(void);

/* Debug strings */
static const char* pattern_names[] = {
    "STEADY\r\n",
    "BREATHING\r\n",
    "MORSE SOS\r\n",
    "DOUBLE BLINK\r\n"
};

/**
 * @brief TIM2 Interrupt Handler
 */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) // If update interrupt flag
    {
        g_ms_ticks++;          // Increment tick counter
        TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
    }
}

int main(void)
{
    uint32_t current_period = LED_PERIOD_MS;
    ButtonState btn_state;

    /* Initialize peripherals */
    gpio_init();
    timer_init();
    uart_init();

    /* Send initial message */
    uart_write("LED Control Program Started\r\n");
    uart_write("Current Pattern: STEADY\r\n");
    uart_write("Current Speed Level: 0\r\n");

    /* Enable TIM2 interrupt in NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Start timer */
    TIM2->CR1 |= TIM_CR1_CEN;

    while(1)
    {
        btn_state = check_button();

        switch(btn_state)
        {
            case BUTTON_PRESSED:
                /* Increment speed on short press */
                g_speed_level = (g_speed_level + 1) % 4;
                current_period = LED_PERIOD_MS >> g_speed_level;
                uart_write("Speed Level Changed to: ");
                uart_write_num(g_speed_level);
                uart_write("\r\n");
                break;

            case BUTTON_LONG_PRESS:
                /* Change pattern on long press */
                g_current_pattern = (g_current_pattern + 1) % MAX_PATTERNS;
                uart_write("Pattern Changed to: ");
                uart_write(pattern_names[g_current_pattern]);
                break;

            default:
                break;
        }

        /* Execute current pattern */
        switch(g_current_pattern)
        {
            case PATTERN_STEADY:
                pattern_steady(current_period);
                break;

            case PATTERN_BREATH:
                pattern_breath(current_period);
                break;

            case PATTERN_MORSE_SOS:
                pattern_morse_sos(current_period);
                break;

            case PATTERN_DOUBLE:
                pattern_double(current_period);
                break;

            default:
                error_indication();
                break;
        }
    }
}

/**
 * @brief Initialize GPIO peripherals
 */
static void gpio_init(void)
{
    /* Enable GPIO Clocks */
    RCC->AHB1ENR |= GPIOAEN | GPIOCEN;

    /* Configure PA5 (LED) as output */
    GPIOA->MODER |= (1U<<10);
    GPIOA->MODER &= ~(1U<<11);

    /* Configure PC13 (Button) as input */
    GPIOC->MODER &= ~(3U<<26);

    /* Configure UART pins (PA2 = TX, PA3 = RX) */
    // Configure PA2 (TX)
    GPIOA->MODER &= ~(3U<<4);    // Clear bits
    GPIOA->MODER |= (2U<<4);     // Set to alternate function
    GPIOA->AFR[0] |= (7<<8);     // AF7 for USART2

    // Configure PA3 (RX)
    GPIOA->MODER &= ~(3U<<6);    // Clear bits
    GPIOA->MODER |= (2U<<6);     // Set to alternate function
    GPIOA->AFR[0] |= (7<<12);    // AF7 for USART2
}

/**
 * @brief Initialize Timer2 for 1ms ticks
 */
static void timer_init(void)
{
    /* Enable Timer2 clock */
    RCC->APB1ENR |= TIM2EN;

    /* Configure Timer2 */
    TIM2->PSC = TIM2_PSC;    // Prescaler for 1ms ticks
    TIM2->ARR = 1;           // Auto-reload value (1ms period)

    /* Enable update interrupt */
    TIM2->DIER |= TIM_DIER_UIE;
}

/**
 * @brief Check button state with debouncing
 */
static ButtonState check_button(void)
{
    static uint8_t prev_pressed = 0;
    static uint8_t long_press_triggered = 0;
    uint8_t currently_pressed = !(GPIOC->IDR & BTN_PIN);
    ButtonState result = BUTTON_IDLE;

    if(currently_pressed)
    {
        if(!prev_pressed)  // Button just pressed
        {
            g_press_start = g_ms_ticks;
            long_press_triggered = 0;
            result = BUTTON_PRESSED;
        }
        else  // Button being held
        {
            uint32_t press_duration = g_ms_ticks - g_press_start;
            if(press_duration >= LONG_PRESS_MS && !long_press_triggered)
            {
                result = BUTTON_LONG_PRESS;
                long_press_triggered = 1;
            }
        }
    }
    else if(!currently_pressed && prev_pressed)  // Button released
    {
        result = BUTTON_RELEASED;
        long_press_triggered = 0;
    }

    prev_pressed = currently_pressed;
    return result;
}

/**
 * @brief Regular on/off blinking pattern
 */
static void pattern_steady(uint32_t period_ms)
{
    static uint32_t last_toggle = 0;

    if(g_ms_ticks - last_toggle >= period_ms)
    {
        GPIOA->ODR ^= LED_PIN;
        last_toggle = g_ms_ticks;
    }
}

/**
 * @brief Breathing effect pattern using PWM
 */
static void pattern_breath(uint32_t period_ms)
{
    static uint32_t last_update = 0;
    static uint8_t brightness = 0;
    static int8_t direction = 1;
    static uint32_t pwm_counter = 0;

    /* Update brightness every period */
    if(g_ms_ticks - last_update >= period_ms/100)
    {
        brightness += direction;
        if(brightness >= 100 || brightness == 0)
        {
            direction *= -1;
        }
        last_update = g_ms_ticks;
    }

    /* PWM implementation */
    pwm_counter = (pwm_counter + 1) % 100;
    if(pwm_counter < brightness)
    {
        GPIOA->ODR |= LED_PIN;
    }
    else
    {
        GPIOA->ODR &= ~LED_PIN;
    }
}

/**
 * @brief SOS morse code pattern
 */
static void pattern_morse_sos(uint32_t period_ms)
{
    static const uint8_t sos_pattern[] = {
        1,0,1,0,1,0,  // S (...)
        2,0,2,0,2,0,  // O (---)
        1,0,1,0,1     // S (...)
    };
    static uint8_t pattern_pos = 0;
    static uint32_t last_change = 0;

    if(g_ms_ticks - last_change >= period_ms)
    {
        if(sos_pattern[pattern_pos] == 1)
        {
            GPIOA->ODR |= LED_PIN;
            last_change = g_ms_ticks;
        }
        else if(sos_pattern[pattern_pos] == 2)
        {
            GPIOA->ODR |= LED_PIN;
            last_change = g_ms_ticks;
        }
        else
        {
            GPIOA->ODR &= ~LED_PIN;
            last_change = g_ms_ticks;
        }

        pattern_pos = (pattern_pos + 1) % sizeof(sos_pattern);
    }
}

/**
 * @brief Double-blink pattern
 */
static void pattern_double(uint32_t period_ms)
{
    static uint32_t last_change = 0;
    static uint8_t blink_state = 0;

    if(g_ms_ticks - last_change >= period_ms/4)
    {
        switch(blink_state)
        {
            case 0:
                GPIOA->ODR |= LED_PIN;
                blink_state = 1;
                break;
            case 1:
                GPIOA->ODR &= ~LED_PIN;
                blink_state = 2;
                break;
            case 2:
                GPIOA->ODR |= LED_PIN;
                blink_state = 3;
                break;
            case 3:
                GPIOA->ODR &= ~LED_PIN;
                blink_state = 0;
                break;
        }
        last_change = g_ms_ticks;
    }
}

/**
 * @brief Error indication pattern
 */
/**
 * @brief Initialize UART for debug output
 */
static void uart_init(void)
{
    /* Enable UART clock */
    RCC->APB1ENR |= USART2EN;

    /* Configure UART */
    USART2->BRR = UART_BRR_VAL;  // Set baudrate
    USART2->CR1 = 0;             // Clear control register
    USART2->CR1 |= (1U<<13);     // Enable UART
    USART2->CR1 |= (1U<<3);      // Enable Transmitter
    USART2->CR1 |= (1U<<2);      // Enable Receiver
}

/**
 * @brief Send string over UART
 */
static void uart_write(const char* str)
{
    while(*str != '\0')
    {
        // Wait until transmit buffer is empty
        while(!(USART2->SR & (1U<<7)));
        USART2->DR = (*str & 0xFF);
        str++;
    }
}

/**
 * @brief Send number over UART
 */
static void uart_write_num(uint32_t num)
{
    char str[11];  // Maximum 32-bit number is 10 digits
    int i = 0;

    // Handle zero case
    if(num == 0)
    {
        uart_write("0");
        return;
    }

    // Convert number to string (in reverse)
    while(num > 0)
    {
        str[i++] = '0' + (num % 10);
        num /= 10;
    }

    // Send digits in correct order
    while(i > 0)
    {
        char c[2] = {str[--i], '\0'};
        uart_write(c);
    }
}

static void error_indication(void)
{
    static uint32_t last_change = 0;
    static uint8_t blinks = 0;

    if(g_ms_ticks - last_change >= 100)  // Fast 100ms blinks
    {
        GPIOA->ODR ^= LED_PIN;
        last_change = g_ms_ticks;
        blinks++;

        if(blinks >= 10)  // 5 complete blink cycles
        {
            blinks = 0;
            g_current_pattern = PATTERN_STEADY;  // Return to steady pattern
        }
    }
}
