#ifndef __FLASHACCESS_H
#define __FLASHACCESS_H


#include "stm32g4xx_hal_flash.h"

void loadFromFlash();


int eraseReservedFlash();


int saveToFlash();





#endif