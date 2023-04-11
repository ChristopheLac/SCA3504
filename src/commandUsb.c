#include "commandUsb.h"

#include <string.h>
#include "moteur.h"
#include "ledPwm.h"


#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(commandUsb, LOG_LEVEL_INF);

uint32_t Scale_Convert(void);
void SetDestination(uint16_t value);

str_exchange_table exchange_table;

myunion1 BACKUP;

int8_t readStorage(uint8_t adresse, uint8_t *pBuffer, uint8_t size);
int8_t writeStorage(uint8_t adresse, uint8_t *pBuffer, uint8_t size);

// interrupt
void commandUsb_init(void)
{
    //
    memset(&BACKUP, 0, sizeof(BACKUP));
    memset(&exchange_table, 0, sizeof(exchange_table));

    exchange_table.usb_state = USB_STAT_INIT;
    exchange_table.ble_state = BLE_STAT_INIT;
    exchange_table.scale_state = SCALE_STAT_WAIT;
    exchange_table.motor_state = MOTOR_ANGLE_FINISH;
    exchange_table.gyro_state = GYRO_STAT_INIT;

//    readStorage(0, &BACKUP, sizeof(BACKUP));
    if (BACKUP.detail.NVM_DATA != NVM_DATA_value)
    {
        BACKUP.detail.NVM_DATA = NVM_DATA_value;
        BACKUP.detail.cw = 1;
        BACKUP.detail.dist_shutter = 1500;
        BACKUP.detail.speed_mul = 10;
        BACKUP.detail.ramp_width_ratio = RampWidthRatio;
        BACKUP.detail.ramp_width_up_max = RampWidthUpMax;
        memcpy(BACKUP.detail.Serial_string1, "String1", sizeof("String1"));
        memcpy(BACKUP.detail.Serial_string2, "String2", sizeof("String2"));
//        writeStorage(0, &BACKUP, sizeof(BACKUP));
    }

    exchange_table._1ms.gyro = 100;
    exchange_table.Gyro_threshold.w = 1000;

    exchange_table.WEIGHT_Coef = 100;
}

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

#pragma pack(1)
typedef struct
{
    CMD_STATE cmd;
    union
    {
        uint8_t light;
        uint8_t coefWeight;
        struct
        {
            uint8_t sens;
            str16_t angle;
            uint8_t speedMul;
            uint8_t save;
            uint8_t ramp_width_ratio;
            str16_t ramp_width_up_max;
        } rotation;
        struct
        {
            uint8_t sens;
            uint8_t speed;
        } rotationVitesse;
        struct {
            str16_t threshold;
        } gyro;
        struct {
            uint8_t save;
            uint8_t txt[SERIAL_SIZE];
        }serial;
    };
} strCmdUsbIn;
#pragma pack()

#pragma pack(1)
typedef struct
{
    CMD_STATE cmd;
    eError error;
    union
    {
        struct
        {
            uint8_t softMajeur;
            uint8_t softMineur;
            uint8_t hardMajeur;
            uint8_t hardMineur;
        } version;
        struct {
            str16_t weight;
            str16_t motorPosition;
            str16_t I_Mot;
            eError error;
        } status;
        struct {
            uint8_t error;
        } light;
        struct {
            str32_t scaleConvert;
            uint8_t coefWeight;
        } weight;
        struct {
            uint8_t error;
        } rotation;
        struct {
            uint8_t error;
        } vitesse;
        str32_t tare;
        struct {
            str16_t position;
            uint8_t finish;
        } motor;
        str16_t adcMot;
        struct {
            str16_t Gyro_x;
            str16_t Gyro_y;
            str16_t Gyro_z;
        } gyro;
        uint8_t serial[SERIAL_SIZE];
    };
} strCmdUsbOut;
#pragma pack()

int8_t traiteCommande(uint8_t *pBufferIn, const uint8_t nbIn, uint8_t *pBufferOut, const uint8_t nbOutMax)
{
    WORD_VAL var16;
    DWORD_VAL var32;
    static uint8_t cmd20 = 0;
    eSensMoteur sens;

    strCmdUsbIn *pIn = (strCmdUsbIn *)pBufferIn;
    strCmdUsbOut *pOut = (strCmdUsbOut *)pBufferOut;

    pOut->cmd = pIn->cmd;
    pOut->error = ERROR_NO;
    exchange_table.usb_state = USB_STAT_WRITE;
    exchange_table.order = usb_order;

    switch (pIn->cmd)
    {
    case CMD_NONE:
//       	LOG_INF("cmd: CMD_NONE");
        return 0;
    case CMD_VERSION: //----------------------ok
       	LOG_INF("cmd: CMD_VERSION");
        pOut->version.softMajeur = SOFT_VERSION_MAJ;
        pOut->version.softMineur = SOFT_VERSION_MIN;
        pOut->version.hardMajeur = HARD_VERSION_MAJ;
        pOut->version.hardMineur = HARD_VERSION_MIN;
        return sizeof(*pOut);
    case CMD_STATUS: //----------------------ok
       	LOG_INF("cmd: CMD_STATUS");
        var32.dw = Scale_Convert();
        pOut->status.weight.ui16 = var32.dw;
        pOut->status.motorPosition.ui16 = moteur_getPositionDegreCentieme();
        pOut->status.I_Mot.ui16 = moteur_getCourantMa();
        if (exchange_table.gyro_state == GYRO_STAT_ERROR)
        {
            pOut->status.error = ERROR_CMD;
        } else {
            pOut->status.error = ERROR_NO;
        }
        return sizeof(*pOut);
    case CMD_LIGHT_ON_OFF: //----------------------a_v
       	LOG_INF("cmd: CMD_LIGHT_ON_OFF=%d", pIn->light);
        pOut->light.error = ledSetValue(pIn->light);
        return sizeof(*pOut);
    case CMD_SCALE_T: //--------------------------
       	LOG_INF("cmd: CMD_SCALE_T");
        exchange_table.WEIGHT_Tare = exchange_table.WEIGHT;
        pOut->tare.ui32 = exchange_table.WEIGHT_Tare;
        return sizeof(*pOut);
    case CMD_SCALE_R: //----------------------a_v
       	LOG_INF("cmd: CMD_SCALE_R coefWeight=%d", pIn->coefWeight);
        if (pIn->coefWeight > 0 && pIn->coefWeight < 201)
        {
            exchange_table.WEIGHT_Coef = pIn->coefWeight;
        }
        var32.dw = Scale_Convert();
        pOut->weight.scaleConvert.ui32 = var32.dw;
        pOut->weight.coefWeight = exchange_table.WEIGHT_Coef;
        return sizeof(*pOut);
    case CMD_ROTATION_SENS_ANGLE_MULTIP: //----------------------reste 2fct + a_v
       	LOG_INF("cmd: CMD_ROTATION_SENS_ANGLE_MULTIP sens=%d, angle=%d, speedMul=%d, ramp_width_ratio=%d, ramp_width_up_max=%d", 
        pIn->rotation.sens, pIn->rotation.angle.ui16, pIn->rotation.speedMul, pIn->rotation.ramp_width_ratio, pIn->rotation.ramp_width_up_max.ui16);
        if (pIn->rotation.sens) {
            sens = eSensAntiHoraire;
        } else {
            sens = eSensHoraire;
        }
        moteur_setPositionSens(pIn->rotation.angle.ui16, true, sens, pIn->rotation.speedMul, pIn->rotation.ramp_width_ratio, pIn->rotation.ramp_width_up_max.ui16);
        if (pIn->rotation.save == 1)
        {
            moteurSave();
        }
        pOut->rotation.error = 0;
        return sizeof(*pOut);
    case CMD_VITESS_ROTATION:
       	LOG_INF("cmd: CMD_VITESS_ROTATION, sens=%d, speed=%d", pIn->rotationVitesse.sens, pIn->rotationVitesse.speed);
        if (0 == pIn->rotationVitesse.speed) {
            if (true != moteur_getArrivePositionFin())
            moteur_stop();
        } else {
            if (pIn->rotationVitesse.sens) {
                sens = eSensAntiHoraire;
            } else {
                sens = eSensHoraire;
            }
            moteur_setRotationSens(pIn->rotationVitesse.speed, sens);
        }
        pOut->vitesse.error = 0;
        return sizeof(*pOut);
    case CMD_POSITIONP:
//       	LOG_INF("cmd: CMD_POSITIONP");
        pOut->motor.position.ui16 = moteur_getPositionDegreCentieme();
        if (true == moteur_getArrivePositionFin())
        {
            pOut->motor.finish = 0;
        }
        else
        {
            pOut->motor.finish = 1;
        }
        return sizeof(*pOut);
    case CMD_INITPP:                       // remise a  position plateau
       	LOG_INF("cmd: CMD_INITPP");
        moteur_setPosition(0);
        return sizeof(*pOut);
    case CMD_ADC_MOT:
       	LOG_INF("cmd: CMD_ADC_MOT");
        pOut->adcMot.ui16 = moteur_getCourantMa();
        return sizeof(*pOut);
    case CMD_GYRO:
       	LOG_INF("cmd: CMD_GYRO");
        var16.w = pIn->gyro.threshold.ui16;
        memcpy(var16.b, exchange_table.USB_buffin.data, 2);
        if (var16.w == !0)
        {
            exchange_table.Gyro_threshold.w = var16.w;
        }
        pOut->gyro.Gyro_x.ui16 = exchange_table.Gyro_x.w;
        pOut->gyro.Gyro_y.ui16 = exchange_table.Gyro_y.w;
        pOut->gyro.Gyro_z.ui16 = exchange_table.Gyro_z.w;
        return sizeof(*pOut);
    case CMD_WRITE_SERIAL1:
       	LOG_INF("cmd: CMD_WRITE_SERIAL1=%s", pIn->serial.txt);
        if (pIn->serial.save == 'W')
        {
            memcpy(BACKUP.detail.Serial_string1, pIn->serial.txt, SERIAL_SIZE);
//            writeStorage(0, &BACKUP, sizeof(BACKUP));
            return sizeof(*pOut);
        }
        exchange_table.USB_buffout.error = ERROR_CMD;
        return sizeof(*pOut);
    case CMD_READ_SERIAL1:
       	LOG_INF("cmd: CMD_READ_SERIAL1");
        memcpy(pOut->serial, BACKUP.detail.Serial_string1, SERIAL_SIZE);
        return sizeof(*pOut);
    case CMD_WRITE_SERIAL2:
       	LOG_INF("cmd: CMD_WRITE_SERIAL2=%s", pIn->serial.txt);
        if (pIn->serial.save == 'W')
        {
            memcpy(BACKUP.detail.Serial_string2, pIn->serial.txt, SERIAL_SIZE);
//            writeStorage(0, &BACKUP, sizeof(BACKUP));
        return sizeof(*pOut);
        }
            exchange_table.USB_buffout.error = ERROR_CMD;
        return sizeof(*pOut);

    case CMD_READ_SERIAL2:
       	LOG_INF("cmd: CMD_READ_SERIAL2");
        memcpy(pOut->serial, BACKUP.detail.Serial_string2, SERIAL_SIZE);
        return sizeof(*pOut);
    case CMD_RESET:
       	LOG_INF("cmd: CMD_RESET");
        while (1)
            ;
        break;

    case CMD_20: // quand reponse longue un code 0x20 est recu !!!
       	LOG_INF("cmd: CMD_20");
        if (cmd20)
        {
            cmd20 = 0;
            exchange_table.usb_state = USB_STAT_READ;
        }
        return -2;

    default:
       	LOG_INF("cmd: default:%d", pIn->cmd);
        pOut->error = ERROR_CMD;
        return -1;
    }
    return -3;
}

uint32_t Scale_Convert(void)
{
    uint64_t value;
    if (exchange_table.WEIGHT < exchange_table.WEIGHT_Tare)
    {
        exchange_table.WEIGHT_Tare = exchange_table.WEIGHT;
    }
    value = exchange_table.WEIGHT - exchange_table.WEIGHT_Tare;
    value *= (900 + exchange_table.WEIGHT_Coef);
    value /= 1000 * Scale_coef;
    return value;
}

#if 0
void Task_BP(void)
{
#define debounce 10
    static uint8_t debounce_f, debounce_s, progress;

    if (exchange_table._1ms.bp == 0)
    {
        exchange_table._1ms.bp = 10;
        if (FOCUS_BP_Read() == 0)
            debounce_f++;
        else
            debounce_f = 0;
        if (debounce_f == debounce)
        {
            Read_Flash();
            SetDestination(BACKUP.detail.dist_shutter);
            exchange_table.motor_state = MOTOR_ANGLE_SETUP;
        }
        if (debounce_f > debounce)
            debounce_f = debounce + 1;

        if (Shutter_BP_Read() == 0)
            debounce_s++;
        else
            debounce_s = 0;
        if (debounce_s == debounce)
        {
            progress = 1;
            BACKUP.detail.cw = 1;
            exchange_table.speed = 50;
            exchange_table.motor_state = MOTOR_SPEED;
        }
        if (debounce_s > debounce)
            debounce_s = debounce + 1;
        if (debounce_s == 0 && progress)
        {
            progress = 0;
            exchange_table.speed = 0;
        }
    }
}

void Task_Mot(void)
{
    if (BACKUP.detail.cw)
        CW_CCW_Write(0);
    else
        CW_CCW_Write(1);
    INT_VR_EXT_Write(0); // set external speed
    if (!exchange_table._1ms.motor)
    {
        TP2_Write(1);
        exchange_table._1ms.motor = 1;
        // read current motor
        exchange_table.I_Mot = ADC_GetResult16(0);
        exchange_table.I_Mot = ADC_CountsTo_mVolts(0, exchange_table.I_Mot);
        exchange_table.I_Mot *= 166;
        exchange_table.I_Mot /= 100;
        // select command

        switch (exchange_table.motor_state)
        {
        case MOTOR_SPEED:
            if (!exchange_table.speed)
            {
                START_STOP_Write(0);
                RUN_BRAKE_Write(0);
                exchange_table.motor_state = MOTOR_ANGLE_FINISH;
            }
            else
            {
                START_STOP_Write(1);
                RUN_BRAKE_Write(1);
            }
            exchange_table.dist_error = 0;
            PWM_M_WriteCompare(exchange_table.speed);
            if (exchange_table.I_Mot > I_max)
                exchange_table.motor_state = MOTOR_ANGLE_FINISH;
            break;
        case MOTOR_ANGLE_SETUP:
            exchange_table.speed = 0;
            exchange_table.dist_count = 0;
            PWM_M_WriteCompare(0);
            START_STOP_Write(1);
            RUN_BRAKE_Write(1);
            SetupRampConstants();
            exchange_table.motor_state = MOTOR_ANGLE_RUN;

        case MOTOR_ANGLE_RUN:
            exchange_table._1ms.motor_work = work_delay;
            if (exchange_table.dist_count < exchange_table.dist_length)
            {
                CalculatemotorSpeed();
                PWM_M_WriteCompare(exchange_table.speed);
            }
            else
            {
                exchange_table.motor_state = MOTOR_ANGLE_BRAKE;
                exchange_table._1ms.motor_work = 100;
            }
            //                if (exchange_table._1ms.motor_work == 0)
            //                {
            //                START_STOP_Write(0);
            //                RUN_BRAKE_Write(0);
            //                }
            break;
        case MOTOR_ANGLE_BRAKE:
            PWM_M_WriteCompare(0);
            START_STOP_Write(0);
            RUN_BRAKE_Write(0);

            if (exchange_table._1ms.motor_work == 0)
            {
                exchange_table.motor_state = MOTOR_ANGLE_FINISH;
                if (exchange_table.order == usb_order)
                    exchange_table.usb_state = USB_STAT_EXE;
                if (exchange_table.order == ble_order)
                    exchange_table.ble_state = BLE_STAT_EXE;
                exchange_table.order = no_order;
                exchange_table.dist_error = exchange_table.dist_destination - (exchange_table.motor_position * Coef_360);
            }
            break;

        case MOTOR_ANGLE_FINISH:
            PWM_M_WriteCompare(0);
            START_STOP_Write(0);
            RUN_BRAKE_Write(0);
            break;
        }
        TP2_Write(0);
    }
}

void CalculatemotorSpeed(void)
{
    float calcul;
    uint8_t delta = exchange_table.max_speed - SpeedMin;
    if (exchange_table.dist_count <= exchange_table.ramp_width_up)
    {

        calcul = exchange_table.dist_count;
        calcul /= exchange_table.ramp_width_up;
        calcul = asinf(calcul) / M_PI_2;
        calcul *= delta;
        exchange_table.speed = calcul + SpeedMin;
    }
    if (exchange_table.dist_count > exchange_table.ramp_width_up && exchange_table.dist_count < (exchange_table.dist_length - exchange_table.ramp_width_dn))
    {
        exchange_table.speed = exchange_table.max_speed;
    }
    if (exchange_table.dist_count >= exchange_table.dist_length - exchange_table.ramp_width_dn)
    {
        calcul = exchange_table.dist_length - exchange_table.dist_count;
        calcul /= exchange_table.ramp_width_dn;
        calcul = asinf(calcul) / M_PI_2;
        calcul *= delta;
        exchange_table.speed = calcul + SpeedMin;
    }
}

void SetupRampConstants(void)
{
    uint32 calcul;

    // Ramp Up
    exchange_table.ramp_width_up = exchange_table.dist_length * BACKUP.detail.ramp_width_ratio;
    exchange_table.ramp_width_up /= 100;
    // Ramp Dn
    calcul = exchange_table.dist_length * BACKUP.detail.ramp_width_ratio;
    calcul /= 100;
    exchange_table.ramp_width_dn = exchange_table.dist_length - calcul;
    // Speed Max
    calcul = exchange_table.ramp_width_up * SpeedMax * BACKUP.detail.speed_mul;
    calcul /= BACKUP.detail.ramp_width_up_max * 10;
    exchange_table.max_speed = calcul;
    // ecretage superieur
    if (exchange_table.max_speed > SpeedMax)
    {
        exchange_table.max_speed = SpeedMax;

        exchange_table.ramp_width_up = BACKUP.detail.ramp_width_up_max * 10;
        exchange_table.ramp_width_up /= BACKUP.detail.speed_mul;

        exchange_table.ramp_width_dn = exchange_table.ramp_width_up * (100 - BACKUP.detail.ramp_width_ratio);
        exchange_table.ramp_width_dn /= BACKUP.detail.ramp_width_ratio;
    }
    // ecretage inferieur
    if (exchange_table.max_speed < SpeedMin2)
        exchange_table.max_speed = SpeedMin2;
}
#endif

void SetDestination(uint16_t value)
{

    if (BACKUP.detail.cw)
    {
        exchange_table.dist_length = (value + exchange_table.dist_error) / Coef_360;
        exchange_table.dist_destination = (exchange_table.motor_position * Coef_360) + value + exchange_table.dist_error;
        if (exchange_table.dist_destination >= (Count_360 * Coef_360))
            exchange_table.dist_destination -= (Count_360 * Coef_360);
    }
    else
    {
        exchange_table.dist_length = (value - exchange_table.dist_error) / Coef_360;
        exchange_table.dist_destination = (exchange_table.motor_position * Coef_360) - value + exchange_table.dist_error;
        if (exchange_table.dist_destination <= 0)
            exchange_table.dist_destination += (Count_360 * Coef_360);
    }
}

#if 0
void Gyro_Write_Bytes(uint8_t Register, uint8_t Value)
{

    buff_i2c[0] = Register;
    buff_i2c[1] = Value;
    Master_I2CMasterWriteBuf(L3GD20H_ADDR, buff_i2c, 2, Master_I2C_MODE_COMPLETE_XFER);
    while (!(Master_I2CMasterStatus() & Master_I2C_MSTAT_WR_CMPLT) && exchange_table._1ms.gyro != 0)
        ;                          // Wait till the master completes writing
    Master_I2CMasterClearStatus(); // Clear I2C master status
}

uint8_t Gyro_Read_Byte(uint8_t Register)
{
    buff_i2c[0] = Register;
    Master_I2CMasterWriteBuf(L3GD20H_ADDR, buff_i2c, 1, Master_I2C_MODE_NO_STOP);
    while (!(Master_I2CMasterStatus() & Master_I2C_MSTAT_WR_CMPLT))
        ;                          // Wait till the master completes writing
    Master_I2CMasterClearStatus(); // Clear I2C master status

    Master_I2CMasterReadBuf(L3GD20H_ADDR, buff_i2c, 1, Master_I2C_MODE_REPEAT_START);
    while (!(Master_I2CMasterStatus() & Master_I2C_MSTAT_RD_CMPLT))
        ;                          // Wait till the master completes writing
    Master_I2CMasterClearStatus(); // Clear I2C master status
    return buff_i2c[0];
}

void Gyro_Read_Buf(uint8_t Register, uint8_t size)
{
    buff_i2c[0] = Register + 0x80;
    Master_I2CMasterWriteBuf(L3GD20H_ADDR, buff_i2c, 1, Master_I2C_MODE_NO_STOP);
    while (!(Master_I2CMasterStatus() & Master_I2C_MSTAT_WR_CMPLT))
        ;                          // Wait till the master completes writing
    Master_I2CMasterClearStatus(); // Clear I2C master status
    buff_i2c[0] = 0;
    Master_I2CMasterReadBuf(L3GD20H_ADDR, buff_i2c, size, Master_I2C_MODE_REPEAT_START);
    while (!(Master_I2CMasterStatus() & Master_I2C_MSTAT_RD_CMPLT))
        ;                          // Wait till the master completes writing
    Master_I2CMasterClearStatus(); // Clear I2C master status
}

void Read_Flash(void)
{
    memcpy(BACKUP.total, (void *)(CY_FLASH_SIZE - (2 * CY_FLASH_SIZEOF_ROW)), CY_FLASH_SIZEOF_ROW);
}

void Write_Flash(void)
{
    int rowNum = CY_FLASH_NUMBER_ROWS - 2; //((int)row - CY_FLASH_BASE) / CY_FLASH_SIZEOF_ROW;
    CySysFlashSetWaitCycles(48);
    CySysFlashWriteRow(rowNum, BACKUP.total);
}

uint8_t *uitoa(uint16_t Value, uint8_t *Buffer)
{
    uint8_t i;
    uint16_t Digit;
    uint16_t Divisor;
    uint8_t Printed = 0;

    if (Value)
    {
        for (i = 0, Divisor = 10000; i < 5u; i++)
        {
            Digit = Value / Divisor;
            if (Digit || Printed)
            {
                *Buffer++ = '0' + Digit;
                Value -= Digit * Divisor;
                Printed = 1;
            }
            Divisor /= 10;
        }
    }
    else
    {
        *Buffer++ = '0';
    }
    *Buffer++ = ' ';
    return Buffer;
}

void Timer_Int_Interrupt_InterruptCallback(void)
{
    for (uint8_t i = 0; i < timer_1ms_size; i++)
    {
        if (exchange_table.timer_1ms[i])
            exchange_table.timer_1ms[i]--;
    }
}

void Speed_Int_Interrupt_InterruptCallback(void)
{
    Speed_Int_ClearPending();
    SPEED_ClearInterrupt();

    if (BACKUP.detail.cw)
        exchange_table.motor_position++;
    else
        exchange_table.motor_position--;
    if (exchange_table.motor_position >= Count_360)
        exchange_table.motor_position -= Count_360;
    if (exchange_table.motor_position < 0)
        exchange_table.motor_position += Count_360;

    exchange_table.dist_count++;
    exchange_table._1ms.motor_overange = overrun_delay;
}

#endif
