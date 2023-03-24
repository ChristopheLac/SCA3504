#ifndef _COMMANDUSB_H
#define _COMMANDUSB_H

#include <stdint.h>

//-----------------------------------------|
//                                         |
//                Motor tuning              |
//                                         |
//-----------------------------------------|
#define RampWidthRatio 30 // %
#define RampWidthUpMax 450
#define RampWidthDnMax (RampWidthUpMax * 100 / RampWidthRatio) - RampWidthUpMax
#define SpeedMax 255
#define SpeedMin 15
#define SpeedMin2 SpeedMin + 10
#define work_delay 1000
#define overrun_delay work_delay + 500
#define I_max 1000 // mA
//-----------------------------------------|
//                                         |
//                Scale tuning             |
//                                         |
//-----------------------------------------|
#define Scale_coef 142

//-----------------------------------------|
//                                         |
//                defines                  |
//                                         |
//-----------------------------------------|

#define Count_360 6000
#define Coef_360 6

#define FOCUS FOCUS_BP_Read()     // PIN_IN
#define Shutter Shutter_BP_Read() // PIN_IN

#define BUFFER_SIZE (32)
#define SOFT_VERSION_MAJ 2  // MAJEURE
#define SOFT_VERSION_MIN 01 // MINEURE
#define HARD_VERSION_MAJ 1
#define HARD_VERSION_MIN 0

/* Active endpoints of USB device. */
#define USBFS_DEVICE (0u)
#define IN_EP_NUM (1u)
#define OUT_EP_NUM (2u)
/* Size of SRAM buffer to store endpoint data. */
// uint8_t length, i;
/**************************************
 *  GYRO L3GD20H
 ***************************************/
enum
{
    WHO_AM_I = 0x0F,
    CTRL1 = 0x20,
    CTRL2,
    CTRL3,
    CTRL4,
    CTRL5,
    REFERENCE,
    OUT_TEMP,
    STATUS,
    OUT_X_L,
    OUT_X_H,
    OUT_Y_L,
    OUT_Y_H,
    OUT_Z_L,
    OUT_Z_H,
    FIFO_CTRL,
    FIFO_SRC,
    IG_CFG,
    IG_SRC,
    IG_THS_XH,
    IG_THS_XL,
    IG_THS_YL,
    IG_THS_ZH,
    IG_THS_ZL,
    IG_DURATION,
    LOW_ODR
};
#define L3GD20H_ADDR 0x6b // 0xd6

#define CTRL1_INIT 0x0f // set data rate, bandwitch,power up,  enable xyz
#define CTRL2_INIT 0x00 //
#define CTRL3_INIT 0x00 //
#define CTRL4_INIT 0x20 // 2000 dps
#define CTRL5_INIT 0x00 //

#define STATUS_ZYXDA 0x08 // XYZ data available

#define buff_i2c_size 32
// uint8_t buff_i2c[buff_i2c_size];

#define timeout_i2c 100

typedef enum
{
    GYRO_STAT_INIT = 0,
    GYRO_STAT_RUN,
    GYRO_STAT_ERROR
} GYRO_STATE;

int8_t traiteCommande(uint8_t *pBufferIn, const uint8_t nbIn, uint8_t *pBufferOut, const uint8_t nbOutMax);

/*****************************************
   EXCHANGE table
*******************************************/
typedef enum
{
    USB_STAT_INIT = 0,
    USB_STAT_INIT2,
    USB_STAT_READ,
    USB_STAT_EXE,
    USB_STAT_WRITE,
} USB_STATE;
typedef enum
{
    BLE_STAT_INIT = 0,
    BLE_STAT_INIT2,
    BLE_STAT_READ,
    BLE_STAT_EXE,
    BLE_STAT_WRITE,
} BLE_STATE;

typedef enum
{
    SCALE_STAT_WAIT = 0,
    SCALE_STAT_READ,
    SCALE_STAT_FINISH
} SCALE_STATE;

typedef enum
{
    MOTOR_SPEED,
    MOTOR_ANGLE_SETUP,
    MOTOR_ANGLE_RUN,
    MOTOR_ANGLE_BRAKE,
    MOTOR_ANGLE_FINISH,
} MOTOR_STATE;

typedef enum
{
    CMD_VERSION = 1,
    CMD_STATUS,                     //(2u)
    CMD_LIGHT_ON_OFF,               //(3u)
    CMD_SCALE_T,                    //(4u)
    CMD_SCALE_R,                    //(5u)
    CMD_ROTATION_SENS_ANGLE_MULTIP, //    (6u)
    CMD_VITESS_ROTATION,            //(7u)
    CMD_POSITIONP,                  //(8u)
    CMD_INITPP,                     //(9u)
    CMD_ADC_MOT,                    //(10u)0x0a
    CMD_GYRO,                       //(11u)0x0b
    CMD_WRITE_SERIAL1,              //(12u)0x0c
    CMD_READ_SERIAL1,               //(12u)0x0d
    CMD_WRITE_SERIAL2,              //(12u)0x0c
    CMD_READ_SERIAL2,               //(12u)0x0d
    CMD_RESET = 0xFF,
    CMD_20 = 0x20,
} CMD_STATE;

typedef enum
{
    no_order,
    usb_order,
    ble_order
} ORDER;

typedef union
{
    uint16_t w;
    uint8_t b[2];
} WORD_VAL;

typedef union
{
    int16_t w;
    uint8_t b[2];
} int16_VAL;

typedef union
{
    uint32_t dw;
    uint8_t w[2];
    uint8_t b[4];
} DWORD_VAL;

typedef enum
{
    ERROR_NO = 0x00,    //	pas d'erreur
    ERROR_CMD = 0xFF,   // cde non valide
    ERROR_MOTOR = 0x55, // pas de rotation du moteur
    ERROR_I = 0xAA,     // depassement du courant

} eError;

typedef struct
{
    union
    {
        uint8_t USB_rx[BUFFER_SIZE];
        struct
        {
            CMD_STATE cmd;
            uint8_t data[BUFFER_SIZE - 1];
        } USB_buffin;
    };
    union
    {
        uint8_t USB_tx[BUFFER_SIZE];
        struct
        {
            uint8_t cmd;
            uint8_t error;
            uint8_t data[BUFFER_SIZE - 2];
        } USB_buffout;
    };

    uint8_t BLE_rx[BUFFER_SIZE];
    uint8_t BLE_tx[BUFFER_SIZE];

    USB_STATE usb_state;
    BLE_STATE ble_state;
    SCALE_STATE scale_state;
    MOTOR_STATE motor_state;
    GYRO_STATE gyro_state;
#define timer_1ms_size 8
    union
    {
        uint32_t timer_1ms[timer_1ms_size];
        struct
        {
            uint32_t motor;
            uint32_t led_usb;
            uint32_t led_debug;
            uint32_t motor_work;
            uint32_t motor_overange;
            uint32_t bp;
            uint32_t gyro;
            uint32_t ble;
        } _1ms;
    };

    uint8_t Light;
    int32_t motor_position;
    uint32_t dist_count;
    uint32_t dist_length;

    int32_t dist_destination;
    int32_t dist_error;
    uint32_t ramp_width_up;
    uint32_t ramp_width_dn;

    uint32_t max_speed;
    uint32_t I_Mot;
    uint32_t WEIGHT;
    uint32_t WEIGHT_Tare;
    uint8_t WEIGHT_Coef;

    uint8_t speed;
    WORD_VAL Gyro_x;
    WORD_VAL Gyro_y;
    WORD_VAL Gyro_z;
    WORD_VAL Gyro_threshold;
    uint8_t Gyro_flag;
    ORDER order;
} str_exchange_table;
extern str_exchange_table exchange_table;

/********************************************
    BACKUP Table
********************************************/
// #define   BACKUP_size CY_FLASH_SIZEOF_ROW // E2PROM SIZE
#define NVM_DATA_value 0x1234
#define SERIAL_SIZE 30
typedef union
{
    uint8_t total[100];
    struct
    {
        int NVM_DATA; // 4
        uint8_t Serial_string1[SERIAL_SIZE];
        uint8_t Serial_string2[SERIAL_SIZE];
        uint8_t cw;
        uint32_t dist_shutter;
        uint8_t speed_mul;
        uint32_t ramp_width_ratio;
        uint32_t ramp_width_up_max;
    } detail;
} myunion1;
extern myunion1 BACKUP;

#endif