#ifndef CIPHER_H
#define CIPHER_H

#include <stdint.h>

void Cipher_Encode(uint8_t *src, uint8_t *dst, uint32_t sync, uint32_t len);
void Cipher_Decode(uint8_t *src, uint8_t *dst, uint32_t sync, uint32_t len);

#endif /* CIPHER_H */
