#ifndef PDU_H
#define PDU_H

#include <stdint.h>

#define ERROR   (0xffffffff)

typedef enum {
    INTERNATIONAL = 0x91,
    ALPHANUMERIC = 0xd0
} PhoneNumberFormat_TypeDef;

typedef struct {
    uint32_t Length;
    PhoneNumberFormat_TypeDef Format;
    char Number[20];
} PhoneNumber_TypeDef;

typedef __packed struct {
    uint8_t Year;
    uint8_t Month;
    uint8_t Day;
    uint8_t Hour;
    uint8_t Min;
    uint8_t Sec;
    uint8_t Timezone;
} Timestamp_TypeDef;

typedef enum {
    DCS_DEFAULT = 0x00,
    DCS_ASCII   = 0x04,
    DCS_UCS2    = 0x08
} DataCodingScheme_TypeDef;

typedef struct {
    PhoneNumber_TypeDef SMSCenter;
    PhoneNumber_TypeDef Origin;
    uint32_t TP_PDU_Type;
    uint32_t TP_PID;
    DataCodingScheme_TypeDef DataCodingScheme;
    Timestamp_TypeDef Timestamp;
    uint32_t TP_UDL;
    char Text[160];
} SMS_TypeDef;

uint32_t PDU_Decode(char *src, SMS_TypeDef *dest);

#endif /* PDU_H */