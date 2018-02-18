#include "car_intf.h"
#include "inout.h"

//////////////////////////// Car interface outputs /////////////////////////////
#define LIGHTS      (0x02)
#define ALARM       (0x04)
#define IGN_O       (0x08)
#define START       (0x10)
#define LOCK        (0x20)
#define HEATER      (0x40)
#define LED         (0x00000008)
#define OPEN        (0x00000010)
#define CLOSE       (0x00000020)

///////////////////////////// Car interface inputs /////////////////////////////
#define SHOCK_LO    (0x02)
#define SHOCK_HI    (0x04)
#define PARKING     (0x08)
#define EXT_220     (0x10)
#define GLOW        (0x20)
#define IGN_I       (0x40)
#define DOORS       (0x80)

/////////////////////////////// Siren and lights ///////////////////////////////
/////////////////////////////// Common functions ///////////////////////////////
void CarIntf_Init() {
    Inout_Init();
}
