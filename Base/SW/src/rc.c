#include "rc.h"
#include "modem.h"
#include "radio.h"

uint32_t RC_Init() {
    uint32_t  result = 0;

    Modem_Init();
    Radio_Init();

    return result;
}
