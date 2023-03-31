/*
 *
 *
 *
 */

#include <zephyr/kernel.h>
 #include <zephyr/modbus/modbus.h>
#include <zephyr/logging/log.h>

#include "moteur.h"
#include "myDefine.h"

const uint32_t nbStep1Tour = 20000;
const uint8_t node = 1;

LOG_MODULE_REGISTER(moteur, LOG_LEVEL_DBG);

static int client_iface;

const static struct modbus_iface_param client_param = {
    .mode = MODBUS_MODE_RTU,
    .rx_timeout = 50000,
    .serial = {
        .baud = 115200,                           // 19200,
        .parity = UART_CFG_PARITY_EVEN,           // UART_CFG_PARITY_NONE,
        .stop_bits_client = UART_CFG_STOP_BITS_1, // UART_CFG_STOP_BITS_2,
    },
};

#define MODBUS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

int init_modbus_client(void)
{
    const char iface_name[] = {DEVICE_DT_NAME(MODBUS_NODE)};

    client_iface = modbus_iface_get_by_name(iface_name);

    return modbus_init_client(client_iface, client_param);
}

typedef enum
{
    eAvance = 0x4000,
    eArriere = 0x8000,

    eStop = 0x0000,
    ePosition0 = 0x0800,
    eStart = 0x0010,
    eArret = 0x0020,
} eCmdMoteurModbus;

typedef enum
{
    MOT_STOP,
    MOT_ROTATE_TO_TARGET,

} stateMoteur_t;

typedef struct
{
    stateMoteur_t state;
    int32_t positionActuelleSteps;
    int32_t positionCibleSteps;
    eSensMoteur sensRotation;
    uint32_t vitesse;
    uint32_t rampe;
} strMoteur;

strMoteur l_moteur;

typedef union
{
    int32_t u32;
    struct
    {
        uint16_t u16L;
        uint16_t u16H;
    };
} ui32_t;

typedef union
{
    uint32_t u32;
    int32_t i32;
    struct
    {
        uint16_t u16H;
        uint16_t u16L;
    };
} ui32_modbus_t;

int sendMoteur_avance(int32_t steps, uint32_t speed_mhz)
{
    ui32_t temp;
    union
    {
        struct
        {
            uint16_t steps_H;
            uint16_t steps_L;
            uint16_t speed_H;
            uint16_t speed_L;
        };
        uint16_t buff[4];
    } holding_reg;

    temp.u32 = steps;
    holding_reg.steps_H = temp.u16H;
    holding_reg.steps_L = temp.u16L;

    temp.u32 = speed_mhz;
    holding_reg.speed_H = temp.u16H;
    holding_reg.speed_L = temp.u16L;
    return modbus_write_holding_regs(client_iface, node, 0x1802, (uint16_t *)&holding_reg, sizeof(holding_reg) / sizeof(uint16_t));
}

int sendMoteur_start(void)
{
    return modbus_write_holding_reg(client_iface, node, 0x007D, 0x0008);
}

int sendMoteur_stop(void)
{
    return modbus_write_holding_reg(client_iface, node, 0x007D, eStop);
}

int sendMoteur_position0(void)
{
    return modbus_write_holding_reg(client_iface, node, 0x007D, ePosition0);
}

int sendMoteur_arret(void)
{
    return modbus_write_holding_reg(client_iface, node, 0x007D, eArret);
}

int sendMoteur_clearErrors(void){
    ui32_modbus_t data;
    data.u16H = 0;
    data.u16L = 1;
    return modbus_write_holding_regs(client_iface, node, 0x0180, (uint16_t *)&data, sizeof(data) / sizeof(uint16_t));
}

int32_t sendMoteur_getPosition(void)
{
    ui32_modbus_t data;
    // add 198
    if (0 == modbus_read_holding_regs(client_iface, node, 198, (uint16_t*)&data, 2))
    {
        str32_t steps;
        steps.i16H = data.u16H;
        steps.i16L = data.u16L;
        return steps.i32;
    }
    return 0;
}

typedef enum
{
    eMoteur_Absolut = 1,
    eMoteur_Incremental = 2,
} eMotteurType;

typedef enum
{
    eMotteur_TriggerAllDAtaUpdated = 1,
} eMotteurTrigger;
volatile int16_t errLocal = -1;
int sendMoteur_startSequence1(void)
{
    ui32_t temp;
    union
    {
        struct
        {
            ui32_modbus_t dataNumber;
            ui32_modbus_t type;
            ui32_modbus_t position;
            ui32_modbus_t speed;
            ui32_modbus_t acceleration;
            ui32_modbus_t deceleration;
            ui32_modbus_t courant;
            ui32_modbus_t trigger;
        };
        uint16_t buff[16];
    } holding_reg;

    memset(holding_reg.buff, 0, sizeof(holding_reg.buff));

    holding_reg.dataNumber.u16L = 0;
    holding_reg.type.u16L = eMoteur_Incremental;

    temp.u32 = l_moteur.positionCibleSteps;
    holding_reg.position.u16H = temp.u16H;
    holding_reg.position.u16L = temp.u16L;

    temp.u32 = l_moteur.vitesse;
    holding_reg.speed.u16H = temp.u16H;
    holding_reg.speed.u16L = temp.u16L;

    holding_reg.acceleration.u16L = l_moteur.rampe;
    holding_reg.deceleration.u16L = l_moteur.rampe;

    holding_reg.courant.u16L = 100; // pourcent

    holding_reg.trigger.u16L = eMotteur_TriggerAllDAtaUpdated;
    errLocal = modbus_write_holding_regs(client_iface, node, 0x0058, (uint16_t *)&holding_reg, sizeof(holding_reg) / sizeof(uint16_t));

    return 0;
}

int sendMoteur_startSequence(void)
{
    ui32_t temp;
    ui32_modbus_t reg;

    reg.u16H = 0;
    reg.u16L = eMoteur_Absolut;
    if (0 != modbus_write_holding_regs(client_iface, node, 0x1800, (uint16_t *)&reg, 2))
    {
        LOG_ERR("FC06 failed");
    }
    k_sleep(K_MSEC(10));

    temp.u32 = l_moteur.positionCibleSteps;
    reg.u16H = temp.u16H;
    reg.u16L = temp.u16L;
    if (0 != modbus_write_holding_regs(client_iface, node, 0x1802, (uint16_t *)&reg, 2))
    {
        LOG_ERR("FC06 failed");
    }
    k_sleep(K_MSEC(10));

    temp.u32 = l_moteur.vitesse;
    reg.u16H = temp.u16H;
    reg.u16L = temp.u16L;
    if (0 != modbus_write_holding_regs(client_iface, node, 0x1804, (uint16_t *)&reg, 2))
    {
        LOG_ERR("FC06 failed");
    }
    k_sleep(K_MSEC(10));

    temp.u32 = l_moteur.rampe;
    reg.u16H = temp.u16H;
    reg.u16L = temp.u16L;
    if (0 != modbus_write_holding_regs(client_iface, node, 0x1806, (uint16_t *)&reg, 2))
    {
        LOG_ERR("FC06 failed");
    }
    k_sleep(K_MSEC(10));

    temp.u32 = l_moteur.rampe;
    reg.u16H = temp.u16H;
    reg.u16L = temp.u16L;
    if (0 != modbus_write_holding_regs(client_iface, node, 0x1808, (uint16_t *)&reg, 2))
    {
        LOG_ERR("FC06 failed");
    }
    k_sleep(K_MSEC(10));

    reg.u16H = 0;
    reg.u16L = 8;
    if (0 != modbus_write_holding_regs(client_iface, node, 0x007C, (uint16_t *)&reg, 2))
    {
        LOG_ERR("FC06 failed");
    }
    k_sleep(K_MSEC(10));

    return 0;
}

uint16_t moteur_getPositionDegreCentieme(void) {
    int32_t positionSteps = l_moteur.positionActuelleSteps % nbStep1Tour;
    uint16_t angle = positionSteps * 360000 / positionSteps;
    return angle;
}

uint16_t moteur_getCourantMa(void) {
    return 0;
}

bool moteur_getArrivePositionFin(void)
{
    if (MOT_STOP == l_moteur.state)
    {
        return true;
    }
    return false;
}

int8_t moteur_setPosition(int32_t angleDegreCentieme)
{
    int32_t positionSteps = l_moteur.positionActuelleSteps % nbStep1Tour;
    int32_t angleSteps = angleDegreCentieme * nbStep1Tour / 36000; // 360° * 100(en centieme)
    int32_t addSteps;
    if (eSensHoraire == l_moteur.sensRotation)
    {
        addSteps = angleSteps - positionSteps;
        if (addSteps < 0)
        {
            addSteps = addSteps + nbStep1Tour;
        }
    }
    else
    {
        addSteps = angleSteps - positionSteps;
        if (addSteps > 0)
        {
            addSteps = addSteps - nbStep1Tour;
        }
    }
    l_moteur.positionCibleSteps = l_moteur.positionActuelleSteps + addSteps;
    return 0;
}

int8_t moteur_setPositionOffset(int32_t angleDegreCentieme)
{
    int32_t angleSteps = angleDegreCentieme * nbStep1Tour / 36000; // 360° * 100(en centieme)
    if (eSensHoraire == l_moteur.sensRotation)
    {
        l_moteur.positionCibleSteps += angleSteps;
    }
    else
    {
        l_moteur.positionCibleSteps -= angleSteps;
    }
    return 0;
}

int8_t moteur_setPositionSens(int32_t angleDegreCentieme, bool angleOffset, eSensMoteur sens, uint8_t coefVitesse, uint16_t coefRampe, uint16_t rampeMax)
{
    if (MOT_STOP != l_moteur.state) {
        return -1;
    }
    
    l_moteur.sensRotation = sens;

    if ((coefVitesse >= 1) && (coefVitesse <= 100))
    {
        l_moteur.vitesse = 5000 * coefVitesse;
        if (l_moteur.vitesse > 4000000) {
            l_moteur.vitesse = 4000000;
        }
    }
    if ((coefRampe >= 10) && (coefRampe <= 50))
    {
        l_moteur.rampe = 10000 * coefRampe;
        if (l_moteur.rampe > 1000000000) {
            l_moteur.rampe = 1000000000;
        }
    }
    if ((rampeMax >= 100) && (rampeMax <= 1000))
    {
    }
    if (false == angleOffset)
    {
        moteur_setPosition(angleDegreCentieme);
    }
    else
    {
        moteur_setPositionOffset(angleDegreCentieme);
    }
    return 0;
}

int8_t moteurSave(void)
{
    return 0;
}

void main_moteur(void)
{
    l_moteur.state = MOT_STOP;
    l_moteur.sensRotation = eSensHoraire;
    l_moteur.vitesse = 100;
    l_moteur.rampe = 5000;

    k_sleep(K_MSEC(1000));
    init_modbus_client();
    k_sleep(K_MSEC(10));

    sendMoteur_clearErrors();
    k_sleep(K_MSEC(10));
    sendMoteur_arret();
    k_sleep(K_MSEC(100));
    sendMoteur_stop();
    k_sleep(K_MSEC(100));

    l_moteur.positionActuelleSteps = sendMoteur_getPosition();
    l_moteur.positionCibleSteps = l_moteur.positionActuelleSteps;
    while (1)
    {
        switch (l_moteur.state)
        {
        case MOT_STOP:
            if (l_moteur.positionCibleSteps != l_moteur.positionActuelleSteps)
            {
                sendMoteur_startSequence();
                l_moteur.state = MOT_ROTATE_TO_TARGET;
            }
            break;
        case MOT_ROTATE_TO_TARGET:
            l_moteur.positionActuelleSteps = sendMoteur_getPosition();
            if (l_moteur.positionCibleSteps == l_moteur.positionActuelleSteps)
            {
                sendMoteur_stop();
                l_moteur.state = MOT_STOP;
            }
            break;
        }

        k_sleep(K_MSEC(10));
    }
}

/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

//K_THREAD_DEFINE(main_moteur_id, STACKSIZE, main_moteur, NULL, NULL, NULL, PRIORITY, 0, 0);
