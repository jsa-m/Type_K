# Type_K
Thermometer based on a type-k thermocouple able to measure up to 450 °C. The device it’s intended to be used alongside a mobile phone, where the user can read the temperature with a rate of 2 samples per second. 
The user can also send a set point temperature. If the actual temperature is near to the set point a led will start fading; if the actual temperature surpasses the set point an acoustic alarm will sound.
![My Image1](../images/type_k.jpg)
## BLE
The device uses “standard” BLE services and characteristics, so it will be recognized as an environmental sensing device with the characteristic, Temperature.
![My Image2](../images/type_k_ble1.jpg)
![My Image3](../images/type_k_ble2.jpg)

