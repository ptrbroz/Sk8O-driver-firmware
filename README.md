# Sk80-driver-firmware
 
Firmware for Sk8O motor driver - a new bldc driver board inspired by Ben Katz's HKC Driver Integrated.

The goals of this repostory is the adaptation of Martin Gurtner's firmware (for HKC Driver) to:

* Use STM32G474RET6 (instead of STM32F446RE which was used in HKC)
* Adapt to different pinout in Sk80 driver board
* Shed the shackles of laggy and generally annoying os.mbed.com

(Martin Gurtner's firmware is available from: https://os.mbed.com/users/MartinGurtner/code/HKC_MiniCheetah/, but don't bother trying to download it from there, the download/clone function is bugged, or at least it was between March and April 2021. An unmodified version of his firmware will be one of the first commits in this repo). 