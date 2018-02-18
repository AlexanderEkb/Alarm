#include <stdint.h>
#include "stm32f1xx.h"
#include "storage.h"
#include "manager.h"

#define STORAGE_MASK    (0x0000ffff)
#define CS_CONST        (0x12ab);
#define CS_CELL         (42)

static uint32_t *Storage = ((uint32_t *)BKP_BASE);
static const __packed uint16_t Defaults[41] = {
    0x0000,                     /* RESERVED */
    0x0000,                     /* Status */
    0x5059, 0x2365, 0x8041,     /* Phone N1 */
    0x4209, 0x6984, 0x8066,     /* Phone N2 */
    0x0000, 0x0000, 0x0000,     /* Phone N3 */
    0x0000, 0x0000, 0x0000,     /* Phone N4 */
    0x0000,                     /* Autostart temp. */
    0x0000,                     /* Autostart time */
    0x0000,                     /* Max. start time */
    0x0000,                     /* Max start attempts */
    0x0000,                     /* RPM lower threshold */
    0x0000,                     /* RPM upper threshold */
    0x0000,                     /* Voltage lower threshold */
    0x0000,                     /* Voltage upper threshold */
    0x0003,                     /* Glow times */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000
};

static uint16_t Storage_CalculateCS();
// #define Storage ((Storage_t *)BKP_BASE)


uint32_t Storage_Init() {
    uint32_t result = 0;
    uint32_t i;

    RCC->APB1ENR |= RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;

    if(Storage_CalculateCS() != Storage_Get(CS_CELL)) {
        result = STORAGE_CS_FAILED;
        for(i=1;i<CS_CELL;i++)
            Storage_Set(i, (uint32_t)Defaults[i]);
    }

    return result;
}

uint16_t Storage_Get(uint32_t n) {
    if(n>10)
        n += 5;
    return(Storage[n] & STORAGE_MASK);
}

void Storage_Set(uint32_t n, uint32_t v) {
    if(n>10)
        n += 5;
    Storage[n] = v & STORAGE_MASK;
    Storage[CS_CELL + 4] = Storage_CalculateCS();
}

static uint16_t Storage_CalculateCS() {
    uint16_t sum = CS_CONST;
    uint32_t i;

    for(i=1;i<CS_CELL;i++)
        sum += Storage_Get(i);

    return(sum);
}

uint16_t Storage_GetStatus() {
    return Storage_Get(STATUS);
}

void Storage_SetStatus(uint16_t s) {
    Storage_Set(STATUS, s);
}

