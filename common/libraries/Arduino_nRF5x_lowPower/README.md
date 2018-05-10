# Arduino_nRF5x_lowPower
Arduino Power Management Library for nRF5x

## Usage

* PowerMode
  * ```nRF5x_lowPower.powerMode(POWER_MODE_OFF);``` to completely shutdown the nRF5x
  * ```nRF5x_lowPower.powerMode(POWER_MODE_LOW_POWER);``` to switch to low power mode
  * ```nRF5x_lowPower.powerMode(POWER_MODE_CONSTANT_LATENCY);``` to switch to constant latency mode (default)

* DCDC
  * ```nRF5x_lowPower.enableDCDC();``` to enable the DC/DC converter
  * ```nRF5x_lowPower.disableDCDC();``` to disable the DC/DC converter

* Interrupt for WakeUp from PowerOff
  * ```nRF5x_lowPower.enableWakeupByInterrupt(uint32_t pin, uint32_t mode);``` to enable the Sense Interrupt
    * pin is the nRF5x Pin
    * mode can be any of the following (HIGH, RISING, LOW, FALLING)
  * ```nRF5x_lowPower.disableWakeupByInterrupt(uint32_t pin);``` to disable the Sense Interrupt