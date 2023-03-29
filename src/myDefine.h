 
#ifndef MYDEFINE_H 
#define MYDEFINE_H   // pour interdire la reentrance 
 
#ifndef SIZEOFTAB 
    #define SIZEOFTAB(a) (sizeof(a) / sizeof(a[0])) 
#endif 


#pragma pack(1)
typedef union
{
    int16_t i16;
    uint16_t ui16;
    struct
    {
        uint8_t ui8L;
        uint8_t ui8H;
    };
    struct
    {
        int8_t i8L;
        int8_t i8H;
    };

} str16_t;
#pragma pack()

#pragma pack(1)
typedef union
{
    int32_t i32;
    uint32_t ui32;
    struct
    {
        uint16_t ui16L;
        uint16_t ui16H;
    };
    struct
    {
        int16_t i16L;
        int16_t i16H;
    };
    struct
    {
        uint8_t ui8LL;
        uint8_t ui8LH;
        uint8_t ui8HL;
        uint8_t ui8HH;
    };
    struct
    {
        int8_t i8LL;
        int8_t i8LH;
        int8_t i8HL;
        int8_t i8HH;
    };
} str32_t;
#pragma pack()


#endif 