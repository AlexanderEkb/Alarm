#include "cipher.h"
#include <string.h>

#define ROUNDS          (32)
#define BYTES_PER_BLOCK (8)

const __packed uint32_t MasterKey[16] = {
    0x89915bec, 0x39e0f26b,
    0xd4d7c332, 0xedbe8679,
    0xda41ac97, 0x75afffdb,
    0x3497ac62, 0xa50792e5
};


uint8_t __packed SBoxes[8][16] = {
    {0x09, 0x06, 0x03, 0x02, 0x08, 0x0b, 0x01, 0x07, 0x0a, 0x04, 0x0e, 0x0f, 0x0c, 0x00, 0x0d, 0x05},
    {0x03, 0x07, 0x0e, 0x09, 0x08, 0x0a, 0x0f, 0x00, 0x05, 0x02, 0x06, 0x0c, 0x0b, 0x04, 0x0d, 0x01},
    {0x0E, 0x04, 0x06, 0x02, 0x0B, 0x03, 0x0D, 0x08, 0x0C, 0x0F, 0x05, 0x0A, 0x00, 0x07, 0x01, 0x09},
    {0x0E, 0x07, 0x0A, 0x0C, 0x0D, 0x01, 0x03, 0x09, 0x00, 0x02, 0x0B, 0x04, 0x0F, 0x08, 0x05, 0x06},
    {0x0B, 0x05, 0x01, 0x09, 0x08, 0x0D, 0x0F, 0x00, 0x0E, 0x04, 0x02, 0x03, 0x0C, 0x07, 0x0A, 0x06},
    {0x03, 0x0A, 0x0D, 0x0C, 0x01, 0x02, 0x00, 0x0B, 0x07, 0x05, 0x09, 0x04, 0x08, 0x0F, 0x0E, 0x06},
    {0x01, 0x0D, 0x02, 0x09, 0x07, 0x0A, 0x06, 0x00, 0x08, 0x0C, 0x04, 0x05, 0x0F, 0x03, 0x0B, 0x0E},
    {0x0B, 0x0A, 0x0F, 0x05, 0x00, 0x0C, 0x0E, 0x08, 0x06, 0x02, 0x03, 0x09, 0x01, 0x07, 0x0D, 0x04}
};

uint32_t Cipher_GetKey(uint32_t i);
uint32_t Cipher_F(uint32_t n, uint32_t m);

void Cipher_Encode(uint8_t *src, uint8_t *dst, uint32_t sync, uint32_t len) {
    uint32_t gamma[2];
    uint32_t i;

    gamma[0] = sync;
    gamma[1] = ~sync;


    for(i=0;i<len;i++) {
        // Refresh gamma
        if(!(i % BYTES_PER_BLOCK)) {
            if(i)
                memcpy((void *)gamma, (void *)(dst - BYTES_PER_BLOCK), BYTES_PER_BLOCK);
            Cipher_SimpleEncode(gamma);
        }
        *(dst++) = *(src++) ^ *((uint8_t *)gamma + (i % BYTES_PER_BLOCK));
    }
}

void Cipher_Decode(uint8_t *src, uint8_t *dst, uint32_t sync, uint32_t len) {
    Cipher_Encode(src, dst, sync, len);
//    union {
//        uint32_t _32[2];
//        uint8_t _8[8];
//    } gamma;
//    uint32_t i;
//    volatile uint32_t chunk;
//
//    gamma._32[0] = sync;
//    gamma._32[1] = ~sync;
//
//
//    for(i=0;i<len;i++) {
//        // Refresh gamma
//        if(!(i % BYTES_PER_BLOCK)) {
//            if(i)
//                memcpy((void *)gamma._32, (void *)(src - BYTES_PER_BLOCK), BYTES_PER_BLOCK);
//            Cipher_SimpleEncode(gamma._32);
//        }
////        chunk = (i & 0xfffffffc) + (3 - i % 4);
////        *(dst++) = *(src++) ^ gamma._8[chunk];
//        *(dst++) = *(src++) ^ *((uint8_t *)gamma._32 + (i % BYTES_PER_BLOCK));
//    }
}

void Cipher_SimpleEncode(Block_t data) {
    uint32_t i;
    uint32_t a = data[0];
    uint32_t b = data[1];
    uint32_t SubKey;
    uint32_t FResult;

    for(i=0;i<ROUNDS;i++) {
        SubKey = Cipher_GetKey(i);
        FResult = b ^ Cipher_F(a, SubKey);

        b = a;
        a = FResult;
    }

    data[0] = b;
    data[1] = a;
}

uint32_t Cipher_GetKey(uint32_t i) {
    return (i < 24)?MasterKey[i % 8]:MasterKey[7 - (i % 8)];
}

uint32_t Cipher_F(uint32_t n, uint32_t m) {
    uint32_t s = n + m;
    uint32_t i;
    uint32_t result = 0;
    uint32_t nibble;

    for(i=0;i<8;i++){
        nibble = s & 0x0f;
        result <<= 4;
        result |= SBoxes[i][nibble];
        s >>= 4;
    }

    return ((result << 11) & 0xfffff800) | ((result >> 21) & 0x000007ff);
}
