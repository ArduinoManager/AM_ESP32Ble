# AM_ESP32Ble

This is an Arduino library to work with Arduino Manager app available for iOS and macOS.

 * Supported Boards: Any ESP32 board
 * Protocol: Bluetooth Low Energy


__This version introduces a caching mechanism that sends only changed values to Arduino Manager, reducing the number of transmitted messages and Bluetooth traffic.__

## Arduino Manager

Arduino Manager allows you to control and receive data from any Arduino or Arduino compatible microcontroller. It provides:

* More than 30 widgets (switch, knob, slider, display, gauge, bar, etc.) you can choose from
* A Code Generator to generate Arduino code quickly and easily 

More information available here:

- iOS: https://sites.google.com/site/fabboco/home/arduino-manager-for-iphone-ipad
- macOS: https://sites.google.com/site/fabboco/home/arduino-manager-for-mac
