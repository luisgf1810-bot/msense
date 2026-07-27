#include "Battery.hpp"
#include "esp_log.h"


Battery::Battery() {

}

void Battery::Init(){
    pinMode(0, INPUT); /*Set up Cable indication pin*/
    pinMode(4, INPUT); /*Set up Battery Voltage Read pin*/
    analogSetAttenuation(ADC_11db);

    uint16_t battery_voltage = (((uint16_t)analogReadMilliVolts(4)) * 2U)+65U;

    if (battery_voltage < USB_VOLTAGE) {
        //_charge_color = LED_COLOR_GREEN;
    } else if (battery_voltage < MIN_BATTERY_VOLTAGE) {
        _chrg_counter = 0;
        
    } else {
        //_charge_color = LED_COLOR_BLUE;
    }
    for (uint8_t vv = 0; vv < AVRG_FILTER_SIZE; vv++) {
        _voltage_avrg[vv] = battery_voltage;
    }
}

uint8_t Battery::BatteryLevelRead() {
  uint16_t voltage = 0;
  voltage = ((BatteryVoltageRead() / 8U) - 408U);
  if (_charge_state == POWER_BAT_CHRG) {
    _voltage_last = 101;  //Show Charging Icon in MicroLink App
  } else if (_charge_state == POWER_USB) {
    _voltage_last = 102;  //Show USB Icon in MicroLink App
  } else if (_charge_state == POWER_BAT_FULL) {
    _voltage_last = 100;
  } else if (_voltage_last <= 1) {
    _voltage_last = 1;
  } else if ((voltage <= _voltage_last) && (voltage <= 100)) {
    _voltage_last = voltage;
  } else {
    //Spike Voltage Filter
  }

  voltage = _voltage_last;

  return (uint8_t)_voltage_last;
}

uint16_t Battery::BatteryVoltageRead() {
  uint32_t voltage_avrg_total = 0;
  _voltage_avrg[_voltage_index] = (((uint16_t)analogReadMilliVolts(4)) * 2U) +65U;
  
  _voltage_index++;
  if (_voltage_index >= AVRG_FILTER_SIZE) {
    _voltage_index = 0;
  }
  for (uint8_t vv = 0; vv < AVRG_FILTER_SIZE; vv++) {
    voltage_avrg_total += _voltage_avrg[vv];
  }
  voltage_avrg_total = voltage_avrg_total / AVRG_FILTER_SIZE;

  return ((uint16_t)voltage_avrg_total);
}


uint8_t Battery::PowerStateRead() {
  return _charge_state;
}