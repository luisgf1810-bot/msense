#include "LedStrip.hpp"
#include "esp_log.h"


LedStrip::LedStrip() {

}

void LedStrip::Init() {

    pinMode(LED_PIN, OUTPUT);   /*Set LED pin as output*/
    digitalWrite(LED_PIN, LOW); /*Init Set up to output low*/
    delay(1);
    rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000); /*Configure RMT to run the onboard addressable LED*/

    LED(0, 0, 0);
    delay(1);
    LED(0, 0, 7);
    delay(80);
    LED(7, 0, 0);
    delay(80);
    LED(0, 7, 0);
    delay(80);
    LED(0, 0, 0);
    delay(1000);
}

void LedStrip::LED(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(LED_PIN, r, g, b);
}

void LedStrip::LED_SetBrightness(uint16_t level) {
    //Brightness level between 1-10
    if (level == 0) {
        _LED_level = 0;
    } else {
        if (level > 10) {
        level = 10;
        }
        _LED_level = 11 - level;
    }
}
