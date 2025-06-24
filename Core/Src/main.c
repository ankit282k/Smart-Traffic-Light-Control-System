/*
 * Smart Traffic Light System with Keypad Security and Ultrasonic Detection
 */

#include <stdint.h>
#include <string.h>

#include "STM32F4xx.h"
#include "lcd.h"

// === TM1637 Pins ===
#define TM1_DIO_PIN 7  // PC7
#define TM1_CLK_PIN 6  // PC6
#define TM1_PORT GPIOC

#define TM2_DIO_PIN 10  // PC10
#define TM2_CLK_PIN 8   // PC8
#define TM2_PORT GPIOC

// === Traffic Light Pins ===
#define RED1_PIN 2
#define YELLOW1_PIN 3
#define GREEN1_PIN 4

#define RED2_PIN 5
#define YELLOW2_PIN 6
#define GREEN2_PIN 7

// === Ultrasonic Pins ===
#define TRIG1_PIN 8   // PB8
#define ECHO1_PIN 9   // PB9
#define TRIG2_PIN 12  // PC12
#define ECHO2_PIN 13  // PC13

// === LED Indicators ===
#define LED1_PIN 14
#define LED2_PIN 15

// === Timing Settings ===
#define BASE_GREEN_TIME 5
#define EXTRA_TIME 3
#define DETECTION_THRESHOLD_CM 10
#define TIMEOUT 60000

// === Pin Definitions ===
#define GREEN_LED_PIN 8  // PA8
#define RED_LED_PIN 9    // PA9

#define US1_TRIG_PIN 8   // PB8
#define US1_ECHO_PIN 9   // PB9
#define US2_TRIG_PIN 12  // PC12
#define US2_ECHO_PIN 13  // PC13

#define PASSKEY_LENGTH 4
char passkey[PASSKEY_LENGTH + 1] = "1234";
char input[PASSKEY_LENGTH + 1];

// Keypad definitions
char keypad_map[4][4] = {{'*', '0', '#', 'D'},
                         {'7', '8', '9', 'C'},
                         {'4', '5', '6', 'B'},
                         {'1', '2', '3', 'A'}};

#define ROW_PORT GPIOB
#define COL_PORTB GPIOB
#define COL_PORTC GPIOC

#define R1_PIN 3
#define R2_PIN 4
#define R3_PIN 5
#define R4_PIN 6
#define C1_PIN 7  // PB7
#define C2_PIN 0  // PC0
#define C3_PIN 1  // PC1
#define C4_PIN 2  // PC2

// SMART TRAFFIC LIGHT START

// === Clock Setup ===
void SystemClock_Config(void) {
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY));
  RCC->PLLCFGR = (8U) | (336U << 6) | (7U << 24) | RCC_PLLCFGR_PLLSRC_HSE;
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY));
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
  SystemCoreClockUpdate();
}

// === Delay ===
void dwt_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks);
}

void delay_ms(uint32_t ms) {
  while (ms--) delay_us(1000);
}

// === Segment Map ===
uint8_t segment_map[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66,
                           0x6D, 0x7D, 0x07, 0x7F, 0x6F};

// === TM1637 Display ===
void TM_init(GPIO_TypeDef* port, uint8_t clk, uint8_t dio) {
  port->MODER &= ~((3 << (clk * 2)) | (3 << (dio * 2)));
  port->MODER |= ((1 << (clk * 2)) | (1 << (dio * 2)));
}

void TM_all_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  TM_init(TM1_PORT, TM1_CLK_PIN, TM1_DIO_PIN);
  TM_init(TM2_PORT, TM2_CLK_PIN, TM2_DIO_PIN);
}

void TM_start(GPIO_TypeDef* port, uint8_t clk, uint8_t dio) {
  port->ODR |= (1 << clk) | (1 << dio);
  delay_us(2);
  port->ODR &= ~(1 << dio);
  delay_us(2);
}

void TM_stop(GPIO_TypeDef* port, uint8_t clk, uint8_t dio) {
  port->ODR &= ~(1 << clk);
  port->ODR &= ~(1 << dio);
  delay_us(2);
  port->ODR |= (1 << clk);
  delay_us(2);
  port->ODR |= (1 << dio);
}

void TM_write_byte(GPIO_TypeDef* port, uint8_t clk, uint8_t dio, uint8_t b) {
  for (int i = 0; i < 8; i++) {
    port->ODR &= ~(1 << clk);
    if (b & 0x01)
      port->ODR |= (1 << dio);
    else
      port->ODR &= ~(1 << dio);
    delay_us(2);
    port->ODR |= (1 << clk);
    delay_us(2);
    b >>= 1;
  }
  port->ODR &= ~(1 << clk);
  port->ODR |= (1 << dio);
  delay_us(2);
  port->ODR |= (1 << clk);
  delay_us(2);
  port->ODR &= ~(1 << clk);
}

void TM_display_number(GPIO_TypeDef* port, uint8_t clk, uint8_t dio,
                       uint16_t num) {
  uint8_t d[4] = {segment_map[(num / 1000) % 10], segment_map[(num / 100) % 10],
                  segment_map[(num / 10) % 10], segment_map[num % 10]};

  TM_start(port, clk, dio);
  TM_write_byte(port, clk, dio, 0x40);
  TM_stop(port, clk, dio);

  TM_start(port, clk, dio);
  TM_write_byte(port, clk, dio, 0xC0);
  for (int i = 0; i < 4; i++) TM_write_byte(port, clk, dio, d[i]);
  TM_stop(port, clk, dio);

  TM_start(port, clk, dio);
  TM_write_byte(port, clk, dio, 0x88 | 0x07);
  TM_stop(port, clk, dio);
}

// === Traffic Light ===
void traffic_light_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER &= ~(0x3FFF << 4);
  GPIOA->MODER |= (0x5555 << 4);
}

void set_light(uint8_t r, uint8_t y, uint8_t g, uint8_t red_pin,
               uint8_t yellow_pin, uint8_t green_pin) {
  GPIOA->ODR &= ~((1 << red_pin) | (1 << yellow_pin) | (1 << green_pin));
  if (r) GPIOA->ODR |= (1 << red_pin);
  if (y) GPIOA->ODR |= (1 << yellow_pin);
  if (g) GPIOA->ODR |= (1 << green_pin);
}

// === Ultrasonic Setup ===
void gpio_ultrasonic_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

  // TRIG1 (PB8), ECHO1 (PB9)
  GPIOB->MODER &= ~((3 << (TRIG1_PIN * 2)) | (3 << (ECHO1_PIN * 2)));
  GPIOB->MODER |= (1 << (TRIG1_PIN * 2));
  GPIOB->PUPDR &= ~(3 << (ECHO1_PIN * 2));
  GPIOB->PUPDR |= (2 << (ECHO1_PIN * 2));

  // TRIG2 (PC12), ECHO2 (PC13)
  GPIOC->MODER &= ~((3 << (TRIG2_PIN * 2)) | (3 << (ECHO2_PIN * 2)));
  GPIOC->MODER |= (1 << (TRIG2_PIN * 2));
  GPIOC->PUPDR &= ~(3 << (ECHO2_PIN * 2));
  GPIOC->PUPDR |= (2 << (ECHO2_PIN * 2));

  // LED Indicators
  GPIOB->MODER &= ~((3 << (LED1_PIN * 2)) | (3 << (LED2_PIN * 2)));
  GPIOB->MODER |= ((1 << (LED1_PIN * 2)) | (1 << (LED2_PIN * 2)));
}

uint32_t read_distance_cm(GPIO_TypeDef* trig_port, uint8_t trig_pin,
                          GPIO_TypeDef* echo_port, uint8_t echo_pin) {
  uint32_t time = 0, timeout = 0;

  trig_port->ODR &= ~(1 << trig_pin);
  delay_us(2);
  trig_port->ODR |= (1 << trig_pin);
  delay_us(10);
  trig_port->ODR &= ~(1 << trig_pin);

  timeout = 0;
  while (!(echo_port->IDR & (1 << echo_pin))) {
    if (timeout++ > TIMEOUT) return 0;
    delay_us(1);
  }

  time = 0;
  timeout = 0;
  while (echo_port->IDR & (1 << echo_pin)) {
    if (timeout++ > TIMEOUT) break;
    time++;
    delay_us(1);
  }

  return time / 58;
}

// SMART TRAFFIC LIGHT END

// === Delay Function ===
void delay(uint32_t t) {
  for (uint32_t i = 0; i < t * 4000; i++) __NOP();
}


// === LED Control ===
void led_init() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= (1 << (GREEN_LED_PIN * 2)) | (1 << (RED_LED_PIN * 2));
  GPIOA->ODR &= ~((1 << GREEN_LED_PIN) | (1 << RED_LED_PIN));
}

void set_led(uint8_t green, uint8_t red) {
  if (green)
    GPIOA->ODR |= (1 << GREEN_LED_PIN);
  else
    GPIOA->ODR &= ~(1 << GREEN_LED_PIN);

  if (red)
    GPIOA->ODR |= (1 << RED_LED_PIN);
  else
    GPIOA->ODR &= ~(1 << RED_LED_PIN);
}

// === Keypad Init and Scan ===
void keypad_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

  for (int pin = R1_PIN; pin <= R4_PIN; pin++) {
    ROW_PORT->MODER &= ~(0x3 << (pin * 2));
    ROW_PORT->PUPDR |= (0x1 << (pin * 2));  // Pull-up
  }

  COL_PORTB->MODER |= (0x1 << (C1_PIN * 2));
  COL_PORTC->MODER |=
      (0x1 << (C2_PIN * 2)) | (0x1 << (C3_PIN * 2)) | (0x1 << (C4_PIN * 2));
}

char keypad_scan(void) {
  uint8_t col_pins[4] = {C1_PIN, C2_PIN, C3_PIN, C4_PIN};
  GPIO_TypeDef* col_ports[4] = {COL_PORTB, COL_PORTC, COL_PORTC, COL_PORTC};
  uint8_t row_pins[4] = {R4_PIN, R3_PIN, R2_PIN, R1_PIN};

  for (int c = 0; c < 4; c++) {
    COL_PORTB->ODR |= (1 << C1_PIN);
    COL_PORTC->ODR |= (1 << C2_PIN) | (1 << C3_PIN) | (1 << C4_PIN);
    col_ports[c]->ODR &= ~(1 << col_pins[c]);
    delay(1);

    for (int r = 0; r < 4; r++) {
      if (!(ROW_PORT->IDR & (1 << row_pins[r]))) {
        while (!(ROW_PORT->IDR & (1 << row_pins[r])));
        return keypad_map[r][c];
      }
    }
  }
  return '\0';
}

// === Ultrasonic Init and Read ===
void us_sensor_init() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
  GPIOB->MODER |= (1 << (US1_TRIG_PIN * 2));
  GPIOB->MODER &= ~(0x3 << (US1_ECHO_PIN * 2));
  GPIOC->MODER |= (1 << (US2_TRIG_PIN * 2));
  GPIOC->MODER &= ~(0x3 << (US2_ECHO_PIN * 2));
}

uint32_t read_ultrasonic(GPIO_TypeDef* TRIG_PORT, int TRIG_PIN,
                         GPIO_TypeDef* ECHO_PORT, int ECHO_PIN) {
  uint32_t start, end;
  TRIG_PORT->ODR &= ~(1 << TRIG_PIN);
  delay(1);
  TRIG_PORT->ODR |= (1 << TRIG_PIN);
  delay(1);
  TRIG_PORT->ODR &= ~(1 << TRIG_PIN);

  while (!(ECHO_PORT->IDR & (1 << ECHO_PIN)));
  start = SysTick->VAL;
  while ((ECHO_PORT->IDR & (1 << ECHO_PIN)));
  end = SysTick->VAL;

  return (start - end);
}

// === Main ===
int main(void) {
  int idx;

  SysTick->LOAD = 0xFFFFFF;
  SysTick->CTRL = 5;

  SystemClock_Config();
  dwt_init();
  TM_all_init();
  traffic_light_init();
  gpio_ultrasonic_init();
  lcd_gpio_init();
  lcd_init();
  led_init();
  keypad_init();
  us_sensor_init();

  lcd_string("Welcome");
  delay(3000);
  lcd(0x01, 0);

  while (1) {
    lcd_string("Press * to Start");
    while (keypad_scan() != '*');

    lcd(0x01, 0);
    lcd_string("Enter Passkey:");
    // delay(6000);
    lcd(0xC0, 0);

    idx = 0;
    memset(input, 0, sizeof(input));

    while (1) {
      char key = keypad_scan();

      if (key >= '0' && key <= '9' && idx < PASSKEY_LENGTH) {
        input[idx++] = key;
        lcd(key, 1);
      } else if (key == '#') {
        input[idx] = '\0';
        lcd(0x01, 0);

        if (strcmp(input, passkey) == 0) {
          lcd_string("Access Granted");
          set_light(0, 0, 1, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
          set_light(0, 0, 1, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
          delay(500);
          set_light(0, 0, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
          set_light(0, 0, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
          delay(1000);
          lcd(0x01, 0);
          char mode = '\0';
          while (mode != '1' && mode != '2') {
            lcd(0x01, 0);
            lcd_string("1:Normal 2:Smart");
            // lcd(0xC0, 0);
            delay(200);
            //                        lcd_string("2:Smart");
            //                        delay(200);
            lcd(0xC0, 0);

            mode = keypad_scan();
          }

          if (mode == '1') {
            // Normal Mode
            while (1) {
              lcd(0x01, 0);
              lcd_string("Normal Mode On");
              delay(500);

              // Phase A
              for (int t = BASE_GREEN_TIME; t >= 0; t--) {
                //                                set_light(0, 0, 1, RED1_PIN,
                //                                YELLOW1_PIN, GREEN1_PIN);
                //                                set_light(1, 0, 0, RED2_PIN,
                //                                YELLOW2_PIN, GREEN2_PIN);
                if (t > 2) {
                  // Green ON
                  set_light(0, 0, 1, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
                  set_light(1, 0, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
                } else {
                  // Yellow ON (last 2 seconds)
                  set_light(0, 1, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
                  set_light(1, 0, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
                }
                TM_display_number(TM1_PORT, TM1_CLK_PIN, TM1_DIO_PIN, t);
                TM_display_number(TM2_PORT, TM2_CLK_PIN, TM2_DIO_PIN, t);
                delay_ms(100);
              }

              set_light(0, 1, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
              delay_ms(200);

              // Phase B
              for (int t = BASE_GREEN_TIME; t >= 0; t--) {
                //                                set_light(1, 0, 0, RED1_PIN,
                //                                YELLOW1_PIN, GREEN1_PIN);
                //                                set_light(0, 0, 1, RED2_PIN,
                //                                YELLOW2_PIN, GREEN2_PIN);
                if (t > 2) {
                  // Green ON
                  set_light(1, 0, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
                  set_light(0, 0, 1, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
                } else {
                  // Yellow ON (last 2 seconds)
                  set_light(1, 0, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
                  set_light(0, 1, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
                }
                TM_display_number(TM1_PORT, TM1_CLK_PIN, TM1_DIO_PIN, t);
                TM_display_number(TM2_PORT, TM2_CLK_PIN, TM2_DIO_PIN, t);
                delay_ms(100);
              }

              set_light(0, 1, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
              delay_ms(200);
            }

          } else {
            // Smart Mode (your existing logic goes here)
            lcd(0x01, 0);
            lcd_string("Smart Mode On");
            delay(500);
            // [Insert your existing smart mode loop here...]
          }

          break;
        } else {
          lcd_string("Wrong Passkey");
          set_light(1, 0, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
          set_light(1, 0, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
          delay(500);
          set_light(0, 0, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
          set_light(0, 0, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
          set_led(0, 1);
          delay(1500);
          set_led(0, 0);
          lcd(0x01, 0);
          idx = 0;
          lcd_string("Retry Passkey:");
          lcd(0xC0, 0);
        }
      }
    }

    while (1) {
      lcd(0x01, 0);
      lcd_string("System Live ...");
      // === Phase A: Green1, Red2 ===
      uint32_t d1 = read_distance_cm(GPIOB, TRIG1_PIN, GPIOB, ECHO1_PIN);
      uint32_t d2 = read_distance_cm(GPIOC, TRIG2_PIN, GPIOC, ECHO2_PIN);
      uint8_t extraA = (d1 > 0 && d1 < DETECTION_THRESHOLD_CM) ? EXTRA_TIME : 0;
      uint8_t t_total = BASE_GREEN_TIME + extraA;

      GPIOB->ODR = (d1 < DETECTION_THRESHOLD_CM)
                       ? (GPIOB->ODR | (1 << LED1_PIN))
                       : (GPIOB->ODR & ~(1 << LED1_PIN));
      GPIOB->ODR = (d2 < DETECTION_THRESHOLD_CM)
                       ? (GPIOB->ODR | (1 << LED2_PIN))
                       : (GPIOB->ODR & ~(1 << LED2_PIN));

      for (int t = t_total; t >= 0; t--) {
        set_light(0, 0, 1, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
        set_light(1, 0, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
        TM_display_number(TM1_PORT, TM1_CLK_PIN, TM1_DIO_PIN, t);
        TM_display_number(TM2_PORT, TM2_CLK_PIN, TM2_DIO_PIN, t);
        delay_ms(100);
      }

      set_light(0, 1, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
      delay_ms(200);

      // === Phase B: Green2, Red1 ===
      d1 = read_distance_cm(GPIOB, TRIG1_PIN, GPIOB, ECHO1_PIN);
      d2 = read_distance_cm(GPIOC, TRIG2_PIN, GPIOC, ECHO2_PIN);
      uint8_t extraB = (d2 > 0 && d2 < DETECTION_THRESHOLD_CM) ? EXTRA_TIME : 0;
      t_total = BASE_GREEN_TIME + extraB;

      GPIOB->ODR = (d1 < DETECTION_THRESHOLD_CM)
                       ? (GPIOB->ODR | (1 << LED1_PIN))
                       : (GPIOB->ODR & ~(1 << LED1_PIN));
      GPIOB->ODR = (d2 < DETECTION_THRESHOLD_CM)
                       ? (GPIOB->ODR | (1 << LED2_PIN))
                       : (GPIOB->ODR & ~(1 << LED2_PIN));

      for (int t = t_total; t >= 0; t--) {
        set_light(1, 0, 0, RED1_PIN, YELLOW1_PIN, GREEN1_PIN);
        set_light(0, 0, 1, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
        TM_display_number(TM1_PORT, TM1_CLK_PIN, TM1_DIO_PIN, t);
        TM_display_number(TM2_PORT, TM2_CLK_PIN, TM2_DIO_PIN, t);
        delay_ms(100);
      }

      set_light(0, 1, 0, RED2_PIN, YELLOW2_PIN, GREEN2_PIN);
      delay_ms(200);
    }
  }
}
 
