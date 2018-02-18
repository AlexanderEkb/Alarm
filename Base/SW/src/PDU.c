#include <string.h>
#include "PDU.h"

#define TP_SCA_TYPE_OFF     (2)
#define TP_SCA_OFF          (4)
#define TP_OA_LEN_OFF       (2)
#define TP_OA_TYPE_OFF      (4)
#define TP_OA_OFF           (6)
#define TP_DCS_OFF          (2)
#define TP_TS_OFF           (4)
#define TP_UDL_OFF          (18)
#define TP_UD_OFF           (20)
#define ONE_BYTE            (2)     /* Two semi-octets per byte */
#define TIMESTAMP_LENGTH    (7)

#define BITS_PER_BYTE       (8)
#define BITS_PER_SYMBOL     (7)
#define SYMBOL_MASK         (0x7f)
#define DCS_MASK            (0x0f)
#define DCS_DEFAULT         (0x00)
#define DCS_ASCII           (0x04)
#define DCS_UCS2            (0x08)

typedef enum {
    NUM_OF_SMS_CENTER, NUM_OF_ORIGIN
} NumberOf_TypeDef;

uint32_t DecodePhoneNumber(char *Text, uint32_t *Index, NumberOf_TypeDef NumberOf, PhoneNumber_TypeDef *Dest);
uint32_t DecodeOneByte(char *Text, uint32_t *Index, uint32_t *Dest);
uint32_t DecodeTimestamp(char *Text, uint32_t *Index, Timestamp_TypeDef *Dest);
uint32_t htoi(char *s, uint32_t *dest, uint32_t l);
void SwapNibbles(char *Text, char *Dest, uint32_t len);
void Decode7Bit(char *src, char *dest, uint32_t len);

uint32_t PDU_Decode(char *src, SMS_TypeDef *dest) {
    uint32_t i = 0;
    uint32_t buf;

    memset(dest, 0, sizeof(SMS_TypeDef));
/*
    if(htoi(src, &dest->TP_SCA_Len, 2))
        return ERROR;
    if(htoi(&src[TP_SCA_TYPE_OFF], &dest->TP_SCA_Type, 2))
        return ERROR;
    switch(dest->TP_SCA_Type) {
    case INTERNATIONAL:
        DecodeIntlPN(&src[TP_SCA_OFF], &dest->TP_SCA[0], dest->TP_SCA_Len - 1);
        i = (dest->TP_SCA_Len + 1) << 1;
        break;
    case ALPHANUMERIC:
        Length = (dest->TP_SCA_Len << 2) / BITS_PER_SYMBOL;
        Decode7Bit(&src[TP_SCA_OFF], &dest->TP_SCA[0], Length);
        i += 4 + dest->TP_SCA_Len;
        if(dest->TP_SCA_Len & 0x01)
            i++;
        break;
    default:
        return ERROR;
    }
*/
    if(DecodePhoneNumber(src, &i, NUM_OF_SMS_CENTER, &dest->SMSCenter))
        return ERROR;
    if(DecodeOneByte(src, &i, &dest->TP_PDU_Type))
        return ERROR;
    if(DecodePhoneNumber(src, &i, NUM_OF_ORIGIN, &dest->Origin))
        return ERROR;
    if(DecodeOneByte(src, &i, &dest->TP_PID))
        return ERROR;
    if(DecodeOneByte(src, &i, &buf))
        return ERROR;
    dest->DataCodingScheme = (DataCodingScheme_TypeDef)buf;
    if(DecodeTimestamp(src, &i, &dest->Timestamp))
        return ERROR;
    if(DecodeOneByte(src, &i, &dest->TP_UDL))
        return ERROR;
    switch (dest->DataCodingScheme & DCS_MASK) {
    case DCS_DEFAULT:
        Decode7Bit(&src[i], &dest->Text[0], dest->TP_UDL);
        break;
    case DCS_ASCII:
        break;
    case DCS_UCS2:
        memcpy(&dest->Text[0], &src[i], dest->TP_UDL);
        break;
    default:
        return ERROR;

    }
    return 0;
}

void SwapNibbles(char *Text, char *Dest,uint32_t len) {
    uint32_t i = 0;

    for(i=0;i<len;i++) {
        Dest[i << 1] = Text[(i << 1) + 1];
        Dest[(i << 1) + 1] = Text[i << 1];
    }
}

void Decode7Bit(char *src, char *dest,uint32_t len) {
    uint32_t i;
    uint32_t Conveyor;
    uint32_t BitsLeft;
    uint32_t Digit;
    uint32_t SrcPtr;
    uint8_t Symbol;

    SrcPtr = 0;
    Conveyor = 0;
    BitsLeft = 0;
    for(i=0;i<len;i++) {
        while(BitsLeft < BITS_PER_SYMBOL) {
            htoi(&src[SrcPtr], &Digit, 2);
            Conveyor |= Digit << BitsLeft;
            BitsLeft += BITS_PER_BYTE;
            SrcPtr += 2;
        }
        Symbol = Conveyor & SYMBOL_MASK;
        Conveyor >>= BITS_PER_SYMBOL;
        BitsLeft -= BITS_PER_SYMBOL;
        dest[i] = Symbol;
    }
}

uint32_t DecodePhoneNumber(char *Text, uint32_t *Index, NumberOf_TypeDef NumberOf, PhoneNumber_TypeDef *Dest) {
    uint32_t buf;
    uint32_t Length;

    if(htoi(&Text[*Index], &Dest->Length, ONE_BYTE))
        return ERROR;
    *Index += ONE_BYTE;
    if(htoi(&Text[*Index], &buf, ONE_BYTE))
        return ERROR;
    Dest->Format = (PhoneNumberFormat_TypeDef)buf;
    *Index += ONE_BYTE;
    switch(Dest->Format) {
    case INTERNATIONAL:
        if(NumberOf == NUM_OF_SMS_CENTER)
            Length = Dest->Length - 1;
        else {
            Length = Dest->Length >> 1;
            if(Dest->Length & 0x00000001)
                Length++;
        }
        SwapNibbles(&Text[*Index], Dest->Number, Length);
        *Index += Length << 1;
        break;
    case ALPHANUMERIC:
        Length = (Dest->Length << 2) / BITS_PER_SYMBOL;
        Decode7Bit(&Text[*Index], &Dest->Number[0], Length);
        *Index += Dest->Length;
        if(Dest->Length & 0x00000001)
            *Index++;
        break;
    default:
        return ERROR;
    }

    return 0;
}

uint32_t DecodeOneByte(char *Text, uint32_t *Index, uint32_t *Dest) {
    if(htoi(&Text[*Index], Dest, ONE_BYTE))
        return ERROR;
    *Index += ONE_BYTE;

    return 0;
}

uint32_t DecodeTimestamp(char *Text, uint32_t *Index, Timestamp_TypeDef *Dest) {
    char cbuf[2];
    char *c = (char *)Dest;
    uint32_t i;
    uint32_t ibuf;

    for(i=0;i<TIMESTAMP_LENGTH;i++) {
        cbuf[0] = Text[*Index + 1];
        cbuf[1] = Text[*Index];
        if(htoi(cbuf, &ibuf, ONE_BYTE) == ERROR)
            return ERROR;
        else
            *c = (char)ibuf;
        c++;
        *Index += ONE_BYTE;
    }

    return 0;
}

uint32_t htoi(char *s, uint32_t *dest, uint32_t l) {
    uint32_t result = 0;
    uint32_t i;

    for(i=0;i<l;i++) {
        result <<= 4;
        switch (*s) {
        case '0':
            break;
        case '1':
            result |= 0x01;
            break;
        case '2':
            result |= 0x02;
            break;
        case '3':
            result |= 0x03;
            break;
        case '4':
            result |= 0x04;
            break;
        case '5':
            result |= 0x05;
            break;
        case '6':
            result |= 0x06;
            break;
        case '7':
            result |= 0x07;
            break;
        case '8':
            result |= 0x08;
            break;
        case '9':
            result |= 0x09;
            break;
        case 'a':
        case 'A':
            result |= 0x0a;
            break;
        case 'b':
        case 'B':
            result |= 0x0b;
            break;
        case 'c':
        case 'C':
            result |= 0x0c;
            break;
        case 'd':
        case 'D':
            result |= 0x0d;
            break;
        case 'e':
        case 'E':
            result |= 0x0e;
            break;
        case 'f':
        case 'F':
            result |= 0x0f;
            break;
        default:
            return ERROR;
        }
        s++;
    }
    *dest = result;
    return 0;
}