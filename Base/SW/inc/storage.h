#ifndef STORAGE_H
#define STORAGE_H

#define STATUS          (0x01)

uint32_t Storage_Init();
uint16_t Storage_Get(uint32_t n);
void Storage_Set(uint32_t n, uint32_t v);
uint16_t Storage_GetStatus();
void Storage_SetStatus(uint16_t s);
void Storage_SetState(uint8_t s);

#endif /* STORAGE_H */
