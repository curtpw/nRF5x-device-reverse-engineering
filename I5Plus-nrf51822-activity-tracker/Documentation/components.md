# i5 Plus components

![X9_activity_tracker](/I5Plus-nrf51822-activity-tracker/Documentation/PCB_color_coded.jpg "X9 Smartwatch commercial image")

| color code | chip label 1st line | chip label 2nd line | chip label 3rd line |purpose of component|name of component|additional information|datasheet|
|------------|---------------------|---------------------|---------------------|---------------------|-|-|-|
| dark green   | T160 | MhLJ || High speed oscillator for nRF51| unknown||unknown|
| light green  | 16A1Y | 1532 ||unknown|unknown|in the linked forum posts below believed to be an Atmel ATmega16U2 (I don't see how that should workout given the label and the different pinout)|unknown|
| mint colored | 622K |||level shifting? 5V to 3.3V|unknown||unknown|
| cyan         | N51822 | QFAAG0 | 1404AR |Bluetooth/main chip|NRF51822||[here](/I5Plus-nrf51822-activity-tracker/Documentation/datasheets/nRF51822_PS_v3.1.pdf)
| yellow       | A524 ||| 32kHz oscillator| unknown||unknown|
| pink         | 232C | UG11 ||touch controller|Azoteq IQS263||[here](/I5Plus-nrf51822-activity-tracker/Documentation/datasheets/iqs263_datasheet-357247.pdf)
| purple       | UN8HX |||battery charge controller|UN8HX||[here](/I5Plus-nrf51822-activity-tracker/Documentation/datasheets/iqs263_datasheet-357247.pdf)
| red          | 8517 | C3H | D8YKN | accelerometer|ST LIS3DH||[here](/I5Plus-nrf51822-activity-tracker/Documentation/datasheets/en.CD00274221.pdf)
| orange       | T24C256A | 15dt3 ||EPROM with 256Kbit||directly wired with light green|	unknown

###additional information:
- the 4 little teeth on the very left directly connect to the USB connector on the housing

### more images
- [this image](/I5Plus-nrf51822-activity-tracker/raw_images/inside/DSC_2338.JPG) and [this image]# i5 Plus components

![X9_activity_tracker](/I5Plus-nrf51822-activity-tracker/Documentation/PCB_color_coded.jpg "X9 Smartwatch commercial image")

| color code | chip label 1st line | chip label 2nd line | chip label 3rd line |purpose of component|name of component|additional information|datasheet|
|------------|---------------------|---------------------|---------------------|---------------------|-|-|-|
| dark green   | T160 | MhLJ || High speed oscillator for nRF51| unknown||unknown|
| light green  | 16A1Y | 1532 ||unknown|unknown|in the linked forum posts below believed to be an Atmel ATmega16U2 (I don't see how that should workout given the label and the different pinout)|unknown|
| mint colored | 622K |||level shifting? 5V to 3.3V|unknown||unknown|
| cyan         | N51822 | QFAAG0 | 1404AR |Bluetooth/main chip|NRF51822||[here](/I5Plus-nrf51822-activity-tracker/Documentation/datasheets/nRF51822_PS_v3.1.pdf)
| yellow       | A524 ||| 32kHz oscillator| unknown||unknown|
| pink         | 232C | UG11 ||touch controller|Azoteq IQS263||[here](/I5Plus-nrf51822-activity-tracker/Documentation/datasheets/iqs263_datasheet-357247.pdf)
| purple       | UN8HX |||battery charge controller|UN8HX||[here](/I5Plus-nrf51822-activity-tracker/Documentation/datasheets/iqs263_datasheet-357247.pdf)
| red          | 8517 | C3H | D8YKN | accelerometer|ST LIS3DH|directly wired to light green as seen [here](/I5Plus-nrf51822-activity-tracker/raw_images/inside/DSC_2345.JPG)|[here](/I5Plus-nrf51822-activity-tracker/Documentation/datasheets/en.CD00274221.pdf)
| orange       | T24C256A | 15dt3 ||EPROM with 256Kbit||directly wired with light green|	unknown

###additional information:
- the 4 little teeth on the very left directly connect to the USB connector on the housing

### more images
- [this image](/I5Plus-nrf51822-activity-tracker/raw_images/inside/DSC_2338.JPG) and [this image](/I5Plus-nrf51822-activity-tracker/raw_images/inside/DSC_2345.JPG) are better to figure out the PCB layout
### front PCB:

![front view 2](/I5Plus-nrf51822-activity-tracker/Documentation/DSC_2318_cropped.JPG "PCB front view")
![front view](/I5Plus-nrf51822-activity-tracker/Documentation/DSC_2350_cropped.jpg "PCB front view")


###other teardowns of this device:

- [on xda-developers forum](https://forum.xda-developers.com/general/accessories/iwown-i5-ble-bracelet-teardown-debug-t3388230)
- [in a htc forum](https://www.htcmania.com/showthread.php?t=1085110)

### credits:
Special thanks to:
- [curtpw](https://github.com/curtpw/)
- [rogerclarkmelbourne](https://github.com/rogerclarkmelbourne)
- [Goran Mahovlic](https://github.com/goran-mahovlic)
- [dr. tune from xda-developers forum](https://forum.xda-developers.com/general/accessories/iwown-i5-ble-bracelet-teardown-debug-t3388230)
- [roninzgz from htcmania forum](https://www.htcmania.com/showthread.php?t=1085110)



this page was created by [Tim](https://github.com/54696d21), this same page is also avaible [here](https://github.com/54696d21/reverse_engineering_nrf5x-wearables/blob/master/I5Plus-nrf51822-activity-tracker/Documentation/components.md)(/I5Plus-nrf51822-activity-tracker/raw_images/inside/DSC_2345.JPG) are better to figure out the PCB layout
### front PCB:

![front view 2](/I5Plus-nrf51822-activity-tracker/Documentation/DSC_2318_cropped.JPG "PCB front view")
![front view](/I5Plus-nrf51822-activity-tracker/Documentation/DSC_2350_cropped.jpg "PCB front view")


###other teardowns of this device:

- [on xda-developers forum](https://forum.xda-developers.com/general/accessories/iwown-i5-ble-bracelet-teardown-debug-t3388230)
- [in a htc forum](https://www.htcmania.com/showthread.php?t=1085110)

### credits:
Special thanks to:
- [curtpw](https://github.com/curtpw/)
- [rogerclarkmelbourne](https://github.com/rogerclarkmelbourne)
- [Goran Mahovlic](https://github.com/goran-mahovlic)
- [dr. tune from xda-developers forum](https://forum.xda-developers.com/general/accessories/iwown-i5-ble-bracelet-teardown-debug-t3388230)
- [roninzgz from htcmania forum](https://www.htcmania.com/showthread.php?t=1085110)



this page was created by [Tim](https://github.com/54696d21), this same page is also avaible [here](https://github.com/54696d21/reverse_engineering_nrf5x-wearables/blob/master/I5Plus-nrf51822-activity-tracker/Documentation/components.md)