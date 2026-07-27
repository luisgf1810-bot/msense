#include <Arduino.h>
#include <Wire.h>


#define POWER_BAT_RUN 0U
#define POWER_USB 1U
#define POWER_INIT 2U
#define POWER_BAT_LOW 3U
#define POWER_BAT_FULL 4U
#define POWER_BAT_CHRG 5U
#define USB_VOLTAGE 4220
#define MIN_BATTERY_VOLTAGE 3350

#define LED_COLOR_RED 0XFF0000U
#define LED_COLOR_ORANGE 0XC04000U
#define LED_COLOR_YELLOW 0XA06000U
#define LED_COLOR_GREEN 0X00FF00U
#define LED_COLOR_AQUA 0X00A030U
#define LED_COLOR_PINK 0XC00020U
#define LED_COLOR_BLUE 0X0000FFU
#define LED_COLOR_WHITE 0XFFFFFFU
#define LED_OFF 0U
#define PWM_RES 8
#define AVRG_FILTER_SIZE 8U

class Battery {
    public:
        uint8_t _chrg_counter = 0U;
        uint8_t _lowvoltage_counter = 0;
        uint8_t _run_frequency_last = 0;
        uint8_t _power_counter = 250;
        uint8_t _charge_state = POWER_INIT;
        uint8_t _voltage_index = 0;
        uint16_t _voltage_last = 0xFFU;
        uint16_t _voltage_avrg[AVRG_FILTER_SIZE] = { 0 };

        Battery();
        void Init();
        uint16_t BatteryVoltageRead();
        uint8_t BatteryLevelRead();
        uint8_t PowerStateRead();
};