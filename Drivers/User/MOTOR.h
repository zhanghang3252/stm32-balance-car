#include "main.h"
void MOTOR_INIT(void);

void MOTOR_EN(uint8_t EN);

void MOTOR_SPEEN(unsigned char motor,uint16_t speed);

void MOTOR_ADVANCE(unsigned char motor);
void MOTOR_RECOIAL(unsigned char motor);

void MOTOR_TURN(unsigned char motor,unsigned char direction, unsigned int speed);