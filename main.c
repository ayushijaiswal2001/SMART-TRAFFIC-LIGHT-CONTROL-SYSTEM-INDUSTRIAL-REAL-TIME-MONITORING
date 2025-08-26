#include "stm32f405xx.h"
#include "lcd.h"

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define ledCount 10
#define THRESHOLD 819  // 20% of 4096

uint8_t fireState = 0;
uint8_t smokeState;
char fire, smoke, gas;

volatile uint16_t thermistorValue = 0;
volatile float voltage = 0.0;
volatile int temperature = 0;
volatile float t1 = 0.0;



//===============================KEYPAD===============================
const char keymap[4][4] = { { '1', '2', '3', 'A' }, { '4', '5', '6', 'B' },
                            { '7', '8', '9', 'C' }, { '*', '0', '#', 'D' } };

const uint8_t row_pins[4] = { 0, 1, 3, 8 };  // PC0, PC1, PC3, PC8
const uint8_t col_pins[4] = { 4, 5, 6, 7 };  // PC4, PC5, PC6, PC7

void delayms(uint32_t dly) {
    for (uint32_t i = 0; i < dly; i++)
        for (volatile uint32_t j = 0; j < 16000; j++);
}

void gpio_keypad_init(void) {
    RCC->AHB1ENR |= (1 << 2);
    for (int i = 0; i < 4; i++) {
        GPIOC->MODER &= ~(3 << (row_pins[i] * 2));
        GPIOC->MODER |= (1 << (row_pins[i] * 2));
    }
    for (int i = 0; i < 4; i++) {
        GPIOC->MODER &= ~(3 << (col_pins[i] * 2));
        GPIOC->PUPDR &= ~(3 << (col_pins[i] * 2));
        GPIOC->PUPDR |= (1 << (col_pins[i] * 2));
    }
}

char scan_keypad(void) {
    for (int row = 0; row < 4; row++) {
        for (int r = 0; r < 4; r++)
            GPIOC->ODR |= (1 << row_pins[r]);
        GPIOC->ODR &= ~(1 << row_pins[row]);
        for (volatile int d = 0; d < 1000; d++);
        for (int col = 0; col < 4; col++) {
            if ((GPIOC->IDR & (1 << col_pins[col])) == 0) {
                return keymap[row][col];
            }
        }
    }
    return 0;
}

//=================== LED BAR GRAPH ================================
const struct {
    GPIO_TypeDef *port;
    uint8_t pin;
} ledPins[ledCount] = {
    { GPIOB, 4 }, { GPIOB, 5 }, { GPIOB, 6 }, { GPIOB, 7 },
    { GPIOB, 8 }, { GPIOB, 9 }, { GPIOB, 10 }, { GPIOB, 11 },
    { GPIOC, 13 }, { GPIOC, 12 }
};

void gpio_init_graph(void) {
    RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);
    for (int i = 0; i < ledCount; i++) {
        GPIO_TypeDef *port = ledPins[i].port;
        uint8_t pin = ledPins[i].pin;
        port->MODER &= ~(3 << (pin * 2));
        port->MODER |= (1 << (pin * 2));
        port->OTYPER &= ~(1 << pin);
        port->PUPDR &= ~(3 << (pin * 2));
    }
}

void adc_init_graph(void) {
    RCC->AHB1ENR |= (1 << 2);
    GPIOC->MODER |= (3 << (2 * 2)); // PC2 analog
    RCC->APB2ENR |= (1 << 8); // ADC1
    ADC1->SQR3 = 12;
    ADC1->CR1 = (1 << 8);
    ADC1->CR2 |= (1 << 1) | (1 << 0);
}

uint16_t adc_read_graph(void) {
    ADC1->SQR3 = 12;
    ADC1->CR2 |= (1 << 30);
    while (!(ADC1->SR & (1 << 1)));
    return ADC1->DR;
}

//================= General GPIO ================================
void delayMs(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 16000; i++);
}

void gpio_init(void) {
    RCC->AHB1ENR |= (1 << 0);

    GPIOA->MODER &= ~(3 << (4 * 2));
    GPIOA->PUPDR &= ~(3 << (4 * 2));
    GPIOA->PUPDR |= (1 << (4 * 2));

    GPIOA->MODER &= ~(3 << (6 * 2));
    GPIOA->PUPDR &= ~(3 << (6 * 2));
    GPIOA->PUPDR |= (1 << (6 * 2));

    GPIOA->MODER &= ~(3 << (7 * 2));
    GPIOA->MODER |= (1 << (7 * 2));

    GPIOA->MODER &= ~((3 << 4) | (3 << 6) | (3 << 16));
    GPIOA->MODER |= ((1 << 4) | (1 << 6) | (1 << 16));

    GPIOA->MODER |= (3 << (5 * 2));
    GPIOA->PUPDR &= ~(3 << (5 * 2));
}

void adc_init(void) {
    RCC->APB2ENR |= (1 << 9);
    ADC2->CR2 = 0;
    ADC2->SMPR2 |= (7 << 15);
    ADC2->CR2 |= (1 << 0);
}

uint16_t read_thermistor_adc(void) {
    ADC2->SQR3 = 5;
    ADC2->CR2 |= (1 << 30);
    while (!(ADC2->SR & (1 << 1)));
    return ADC2->DR;
}

void toggle_led(uint8_t pin) {
    GPIOA->ODR ^= (1 << pin);
    delayMs(10);
}

void display_temp(void) {
    thermistorValue = read_thermistor_adc();
    voltage = (thermistorValue / 4095.0) * 3.3;
    t1 = -50.0 * voltage;
    temperature = t1 + 107.5;
    char temp_str[17];
    sprintf(temp_str, "Temp: %d C", temperature );
    lprint(0xC0, temp_str);
}

//================ FAN & PUMP =============================
void gpio_init_fan_pump(void) {
    RCC->AHB1ENR |= (1 << 1);
    GPIOB->MODER &= ~((3 << 0) | (3 << 2) | (3 << 4) | (3 << 6));
    GPIOB->MODER |= ((1 << 0) | (1 << 2) | (1 << 4) | (1 << 6));
}

void fan_on(void) {
    GPIOB->ODR |= (1 << 0);
    GPIOB->ODR &= ~(1 << 1);
}

void fan_off(void) {
    GPIOB->ODR &= ~((1 << 0) | (1 << 1));
}

void pump_on(void) {
    GPIOB->ODR |= (1 << 2);
    GPIOB->ODR &= ~(1 << 3);
}

void pump_off(void) {
    GPIOB->ODR &= ~((1 << 2) | (1 << 3));
}

// ... [All includes and definitions are unchanged] ...

// ========================= MAIN ==========================
int main(void) {
    gpio_init();
    adc_init();
    gpio_init_fan_pump();
    gpio_keypad_init();
    gpio_init_graph();
    adc_init_graph();
    LcdInit();
    DelayLcd();

    char passkey[5] = { 0 };
    const char password[] = "127C";

    LcdFxn(0, 0x01);
    lprint(0x80, "Welcome");

    while (1) {
        char key = scan_keypad();
        if (key == '*') {
            LcdFxn(0, 0x01);
            lprint(0x80, "Enter Passkey:");
            memset(passkey, 0, sizeof(passkey));
            int i = 0;
            while (1) {
                key = scan_keypad();
                if (key) {
                    if (key == '#') {
                        passkey[i] = '\0';
                        LcdFxn(0, 0x01);
                        if (strcmp(passkey, password) == 0)
                            lprint(0x80, "Please come in");
                        else
                            lprint(0x80, "Wrong");
                        delayms(300);
                        LcdFxn(0, 0x01);
                        lprint(0x80, "Welcome");
                        break;
                    }
                    if (i < 4) {
                        passkey[i++] = key;
                        lprint(0xC0 + i - 1, "*");
                    }
                    while (scan_keypad());
                    delayms(50);
                }
            }
            if (strcmp(passkey, password) == 0) {
                LcdFxn(0, 0x01);
                break;
            }
        }
    }

    while (1) {
        fireState = (GPIOA->IDR & (1 << 6)) == 0;
        smokeState = !(GPIOA->IDR & (1 << 4));
        uint16_t GassensorReading = adc_read_graph();

        if (fireState && smokeState) {
            LcdFxn(0, 0x01);
            lprint(0x80, "FIRE & SMOKE!!!");
            GPIOA->ODR |= (1 << 7);
            fan_on();
            pump_on();
            for (int i = 0; i < ledCount; i++)
                ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);
            display_temp();
            fire = 'y';
            smoke = 'y';
            gas = 'n';
            for (int i = 0; i < 10; i++) {
                toggle_led(2); // RED
                toggle_led(3); // YELLOW
            }
            GPIOA->ODR &= ~(1 << 8); // GREEN OFF
        }

        else if (fireState) {
            LcdFxn(0, 0x01);
            lprint(0x80, "FIRE detected!!!");
            GPIOA->ODR |= (1 << 7);
            pump_on();
            fan_off();
            for (int i = 0; i < ledCount; i++)
                ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);
            display_temp();
            fire = 'y';
            smoke = 'n';
            gas = 'n';
            GPIOA->ODR &= ~((1 << 3) | (1 << 8)); // RED and GREEN OFF
            for (int i = 0; i < 15; i++)
                toggle_led(2);
        }

        else if (smokeState) {
            LcdFxn(0, 0x01);
            lprint(0x80, "SMOKE detected!!!");
            GPIOA->ODR |= (1 << 7);
            fan_on();
            pump_off();
            for (int i = 0; i < ledCount; i++)
                ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);
            display_temp();
            fire = 'n';
            smoke = 'y';
            gas = 'n';
            GPIOA->ODR &= ~((1 << 2) | (1 << 8)); // YELLOW and GREEN OFF
            for (int i = 0; i < 15; i++)
                toggle_led(3);
        }

        else if (GassensorReading > THRESHOLD) {
            uint8_t ledLevel = (GassensorReading * ledCount) / 4096;
            uint8_t gasPercent = (GassensorReading * 100) / 4096;
            if (gasPercent == 0) gasPercent = 1;

            // LED bar graph
            for (int i = 0; i < ledCount; i++) {
                if (i < ledLevel)
                    ledPins[i].port->ODR |= (1 << ledPins[i].pin);
                else
                    ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);
            }

            // Fan and pump off
            fan_off();
            pump_off();

            // Turn OFF buzzer
            GPIOA->ODR &= ~(1 << 7);

            // NEW LOGIC: Turn off green LED if gas > 50%
            if (gasPercent > 50)
                GPIOA->ODR &= ~(1 << 8); // GREEN OFF
            else
                GPIOA->ODR |= (1 << 8);  // GREEN ON

            // Show gas level
            LcdFxn(0, 0x01);
            char buffer[20];
            sprintf(buffer, " Gas level: %3d %%", gasPercent);
            lprint(0x80, buffer);
            display_temp();

            fire = 'n';
            smoke = 'n';
            gas = 'y';
        }

        else {
            LcdFxn(0, 0x01);
            GPIOA->ODR &= ~(1 << 7);        // Buzzer OFF
            GPIOA->ODR |= (1 << 8);         // GREEN ON
            GPIOA->ODR &= ~((1 << 2) | (1 << 3)); // YELLOW, RED OFF

            // Turn off LED bar
            for (int i = 0; i < ledCount; i++)
                ledPins[i].port->ODR &= ~(1 << ledPins[i].pin);

            fan_off();
            pump_off();
            lprint(0x80, "SAFE");
            display_temp();
            fire = 'n';
            smoke = 'n';
            gas = 'n';
        }

        delayMs(200);
    }
}
