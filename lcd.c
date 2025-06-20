
#include <STM32F405xx.h>

#include "lcd.h"

// GPIO initialization for LCD

void lcd_gpio_init(void) {

    RCC->AHB1ENR = (0x0F << 0);

    // PA0, PA1 as output

    GPIOA->MODER |= (5 << 0);

    // PB0, PB1 as output

    GPIOB->MODER |= (5 << 0);

    GPIOB->MODER |= (5 << 24); // PB12, PB13 as output

    GPIOB->MODER |= (5 << 28); // PB14, PB15 as output

    // PC4, PC5 as output

    GPIOC->MODER |= (5 << 8);

}

void lcd_init() {

    lcd(0x01, 0); // Clear screen

    lcd(0x38, 0); // 2-line mode

    lcd(0x06, 0); // Increment cursor

    lcd(0x0C, 0); // Display on, cursor off

}

void lcd(uint8_t val, uint8_t cmd) {

    uint8_t data;

    // PC4

    data = val & 0x01;

    if (data)

        GPIOC->ODR |= (1 << 4);

    else

        GPIOC->ODR &= ~(1 << 4);

    // PC5

    data = (val >> 1) & 0x01;

    if (data)

        GPIOC->ODR |= (1 << 5);

    else

        GPIOC->ODR &= ~(1 << 5);

    // PB0

    data = (val >> 2) & 0x01;

    if (data)

        GPIOB->ODR |= (1 << 0);

    else

        GPIOB->ODR &= ~(1 << 0);

    // PB1

    data = (val >> 3) & 0x01;

    if (data)

        GPIOB->ODR |= (1 << 1);

    else

        GPIOB->ODR &= ~(1 << 1);

    // PB12

    data = ((val >> 4) & 0x01);

    if (data)

        GPIOB->ODR |= (1 << 12);

    else

        GPIOB->ODR &= ~(1 << 12);

    // PB13

    data = (val >> 5) & 0x01;

    if (data)

        GPIOB->ODR |= (1 << 13);

    else

        GPIOB->ODR &= ~(1 << 13);

    // PB14

    data = (val >> 6) & 0x01;

    if (data)

        GPIOB->ODR |= (1 << 14);

    else

        GPIOB->ODR &= ~(1 << 14);

    // PB15

    data = (val >> 7) & 0x01;

    if (data)

        GPIOB->ODR |= (1 << 15);

    else

        GPIOB->ODR &= ~(1 << 15);

    // PA0 Command RS

    if (cmd)

        GPIOA->ODR |= (1 << 0); // RS = 1 (data)

    else

        GPIOA->ODR &= ~(1 << 0); // RS = 0 (command)

    // PA1 Enable (EN)

    GPIOA->ODR |= (1 << 1); // High

    for (unsigned int i = 0; i < 16800; i++);

    GPIOA->ODR &= ~(1 << 1); // Low

    for (unsigned int i = 0; i < 16800; i++);

}

void lcd_string(char *str) {

    uint8_t i = 0;

    while (str[i] != '\0') {

        lcd(str[i], 1);

        i++;

    }

}

void single_print(uint32_t val) {

    long int var = val;

    unsigned char d1, d2, d3, d4 = 0;

    d1 = var % 10;

    var = var / 10;

    d2 = var % 10;

    var = var / 10;

    d3 = var % 10;

    d4 = var / 10;

    lcd(d4 | 0x30, 1);

    lcd(d3 | 0x30, 1);

    lcd(d2 | 0x30, 1);

    lcd(d1 | 0x30, 1);

}

 
