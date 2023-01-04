#include <stdint.h>
#include "stdbool.h"

#define P1		(*((volatile uint32_t *) (0x48000800 + 0x14)))				// BASE Address PORT C + Offset for Output Register
#define P2		(*((volatile uint32_t *) (0x48000800 + 0x14 + 1)))		// BASE Address PORT C + Offset for Output Register + Byte Shift (>> 8, to skip LEDs)
#define P3		(*((volatile uint32_t *) (0x48000000 + 0x14)))				// BASE Address PORT C + Offset for Output Register

//Matrix / 7-Segment
#define PORT1	1
#define PORT2	2
#define PORT3	3
#define ROW		1
#define COLUMN	2

extern uint8_t symbols[17];


enum symbol_e{
NUM_0,
NUM_1,
NUM_2,
NUM_3,
NUM_4,
NUM_5,
NUM_6,
NUM_7,
NUM_8,
NUM_9,
LET_A,
LET_b,
LET_C,
LET_d,
LET_E,
LET_F,
DOT
};











bool configHardware(uint8_t colPort, uint8_t rowPort, uint8_t c);
void setPattern(uint8_t *p);	
void showPattern();
void setTime(uint16_t time);
static void setPort(uint8_t mode, uint8_t val);
