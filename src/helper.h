#include "stdint.h"
#include "freertos/FreeRTOS.h"

#define NOP() asm volatile("nop")
#define noInterrupts() portDISABLE_INTERRUPTS()
#define interrupts() portENABLE_INTERRUPTS()

typedef uint8_t byte;

// void pinMode(uint8_t pin, uint8_t mode)
// {

//     gpio_config_t cfg = {
//         .pin_bit_mask = static_cast<uint64_t>(1) << sckPin,
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_ENABLE,
//         .intr_type = GPIO_INTR_DISABLE};
//     gpio_config(&cfg);
//     cfg.pin_bit_mask = static_cast<uint64_t>(1) << doutPin;
//     cfg.mode = GPIO_MODE_INPUT;
//     gpio_config(&cfg);
// }

void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = esp_timer_get_time();
    if (us)
    {
        uint32_t e = (m + us);
        if (m > e)
        { //overflow
            while (esp_timer_get_time() > e)
            {
                NOP();
            }
        }
        while (esp_timer_get_time() < e)
        {
            NOP();
        }
    }
}

extern int IRAM_ATTR __digitalRead(uint8_t pin)
{
    if (pin < 32)
    {
        return (GPIO.in >> pin) & 0x1;
    }
    else if (pin < 40)
    {
        return (GPIO.in1.val >> (pin - 32)) & 0x1;
    }
    return 0;
}

long millis()
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

long micros()
{
    return (unsigned long)esp_timer_get_time();
}

void yield()
{
    return vPortYield();
}