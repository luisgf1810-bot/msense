#include <Arduino.h>
#include <Wire.h>

#define LED_DEFAULT_BRIGHTNESS 7U
#define LED_SLEEP_BRIGHTNESS 1U
#define LED_PIN 19U
#define LED_ON_PIN 20U
#define LED_COLOR_RED 0XFF0000U
#define LED_COLOR_ORANGE 0XC04000U
#define LED_COLOR_YELLOW 0XA06000U
#define LED_COLOR_GREEN 0X00FF00U
#define LED_COLOR_AQUA 0X00A030U
#define LED_COLOR_PINK 0XC00020U
#define LED_COLOR_BLUE 0X0000FFU
#define LED_COLOR_WHITE 0XFFFFFFU
#define LED_OFF 0U


class LedStrip {
    public:
        uint16_t _LED_Breathing_counter = 0U;
        uint16_t _LED_level = LED_DEFAULT_BRIGHTNESS;
        uint32_t _charge_color = 0;

        LedStrip();
        void Init();

        void LED(uint8_t r, uint8_t g, uint8_t b);
        void LED_SetBrightness(uint16_t level);

};


