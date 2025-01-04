

#include "zf_common_headfile.h"
#include "uart_for_art.h"
#include "car.h"



volatile od_result_t od_result[10];

uint8 uart4_get_data[64];                            // ?????????
uint8 fifo4_get_data[64];                            // fifo ???????
                    
uint32 fifo4_data_count = 0;                         // fifo ????
fifo_struct uart4_data_fifo;

char target_existence;
int16 x,y;

int16 class_of_target;
uint8 uart1_get_data[64];       

uint8 card_bigtype=0;


uint16 func_1(char temp){
	static uint8 count=1;
	static uint16 ans=0;
	static uint16 last=0;
	switch(count){
		case(1):{
			ans+= (temp-'0')*100;
			count = 2;
			break;
		}
		case(2):{
			ans+= (temp-'0')*10;
			count = 3;
			break;
		}
		case(3):{
			ans+= (temp-'0');
			count = 1;
			last = ans;
			ans = 0;
			
			break;
		}
	}
	return last;
}

void uart4_rx_interrupt_handler()
{ 
    uint8 get_data = 0;                                                             
    uint32 temp_length = 0;
    uint8 od_num = 0;
    uart_query_byte(UART_4, &get_data);  
    {
        fifo_write_buffer(&uart4_data_fifo, &get_data, 1);   
    }
    if(0xFF == get_data)
    {
        temp_length = 1;
        fifo_read_buffer(&uart4_data_fifo, fifo4_get_data, &temp_length, FIFO_READ_AND_CLEAN);
        if(0xAA == fifo4_get_data[0])
        {
            temp_length = 1;
            fifo_read_buffer(&uart4_data_fifo, fifo4_get_data, &temp_length, FIFO_READ_AND_CLEAN);
            od_num = fifo4_get_data[0];
            temp_length = 8;
            fifo_read_buffer(&uart4_data_fifo, fifo4_get_data, &temp_length, FIFO_READ_AND_CLEAN);
            memcpy((uint8*)(&od_result[od_num]), fifo4_get_data, 8);
        }
        fifo_clear(&uart4_data_fifo);
    }
	x = (od_result[0].res_x1+od_result[0].res_x2)*0.5;
	y = (od_result[0].res_y1+od_result[0].res_y2)*0.5;
		
}
void uart1_rx_interrupt_handler()
{ 
	static uint8 status;

	if(uart_query_byte(UART_1, uart1_get_data)){
	
		while(1){
				if(uart1_get_data[0]=='x'){
					status='x';
					break;
				}
				if(status=='x'){
					class_of_target = func_1(uart1_get_data[0]);
					break;
				}
				break;
			}
	}

}

void uart1_init(){
    //fifo_init(&uart1_data_fifo, FIFO_DATA_8BIT, uart1_get_data, 64);              // ??'?? fifo ?????????
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);             
    uart_rx_interrupt(UART_1, ZF_ENABLE);                                   // ???? UART_INDEX ?L??????
    interrupt_set_priority(LPUART1_IRQn, 11);                                   // ???ö?? UART_INDEX ??????????? 10

}

void uart4_init(){
    fifo_init(&uart4_data_fifo, FIFO_DATA_8BIT, uart4_get_data, 64);              // ??'?? fifo ?????????
    uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);             
    uart_rx_interrupt(UART_4, ZF_ENABLE);                                   // ???? UART_INDEX ?L??????
    interrupt_set_priority(LPUART4_IRQn, 10);                                   // ???ö?? UART_INDEX ??????????? 10
}