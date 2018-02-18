#ifndef CAR_INTF_H
#define CAR_INTF_H

#include <stdint.h>

typedef enum {
    OFF = 0,
    ON
} State_t;

/////////////////////////////// Siren and lights ///////////////////////////////
void Car_MakeShortBeeps(uint32_t Count);
void Car_MakeLongBeep();
void Car_ShutUp();

/////////////////////////////// Common functions ///////////////////////////////
void CarIntf_Init();

#endif /* CAR_INTF_H */
