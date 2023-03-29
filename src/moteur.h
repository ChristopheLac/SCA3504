#ifndef MOTEUR_H
#define MOTEUR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    eSensHoraire,
    eSensAntiHoraire,
} eSensMoteur;

uint16_t moteur_getPositionDegreCentieme(void);
uint16_t moteur_getCourantMa(void);
bool moteur_getArrivePositionFin(void);

int8_t moteur_setPosition(int32_t angleDegreCentieme);
int8_t moteur_setPositionSens(int32_t angleDegreCentieme, bool angleOffset, eSensMoteur sens, uint8_t coefVitesse, uint16_t coefRampe, uint16_t rampeMax);
int8_t moteurSave(void);

#endif