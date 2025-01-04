#ifndef _UART_FOR_ART_H
#define _UART_FOR_ART_H
#include "zf_common_headfile.h"

void uart4_rx_interrupt_handler();
void uart1_rx_interrupt_handler();

void uart4_init();
void uart1_init();

typedef struct
{
    uint16 res_x1;
    uint16 res_y1;
    uint16 res_x2;
    uint16 res_y2;
}od_result_t;

extern uint8 *uart1_data;
extern char target_existence;
extern volatile od_result_t od_result[10];
extern int16 x,y;
extern int16 class_of_target;
extern uint8 card_bigtype;
#endif