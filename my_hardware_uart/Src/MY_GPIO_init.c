#include "stm32f3xx_hal.h"
#include "MY_HEADERS.h"
// functions body
void MY_GPIO_init(void)
{
	GPIO_init_data init_data;
	GPIO_init_data *ptr_init_data;
	ptr_init_data = &init_data;
	
	// port E configurations
	ptr_init_data->mode = (uint32_t) OUTPUT_MODE << PIN_8 | OUTPUT_MODE << PIN_9 | OUTPUT_MODE << PIN_10 | OUTPUT_MODE << PIN_11 | OUTPUT_MODE << PIN_12 | OUTPUT_MODE << PIN_13 | OUTPUT_MODE << PIN_14 | OUTPUT_MODE << PIN_15;
	ptr_init_data->out = (uint32_t) LOW << DIG_0 | LOW << DIG_1 | LOW << DIG_2 | LOW << DIG_3 | LOW << DIG_4 | LOW << DIG_5 | LOW << DIG_6 | LOW << DIG_7 | LOW << DIG_8 | LOW << DIG_9 | LOW << DIG_10 | LOW << DIG_11 | LOW << DIG_12 | LOW << DIG_13 | LOW << DIG_14 | LOW << DIG_15;
	// seting it all
	my_GPIO_init_port(GPIO_E, ptr_init_data);
	
	//ptr_init_data->mode = (uint32_t) 0xA8000000; // analog mode for all pins
	//ptr_init_data->out = (uint32_t) 0x00000000; // open-drain output mode for all pins [0-15] == 0 ;; [16-31] == 1
	
	//Настроить ногу PA9 как выход push-pull в альтернативном режиме. Частота 50 МГц
	//Настроить ногу PA10 как вход без подтяжки
	//Включить тактирование UART1
	//Настроить параметры UART1: Скорость, кол-во стоп бит, проверку четности итд.
	
	// port B configurations
	
	// this register "MODER" defines I/O modes  
		//so we choose alternate mode for 8th pin (16th and 17t bits) [10] 
		//and input mode for 9th pin (18th and 19th bits) [00]
	ptr_init_data->mode = (uint32_t) 0x00000280 | ALTERNATE_MODE << PIN_8 | INPUT_MODE << PIN_9; //
//	ptr_init_data->mode = (uint32_t) 0x00000280;
//	ptr_init_data->mode = (uint32_t) ALTERNATE_MODE << PIN_8 | INPUT_MODE << PIN_9;

	
	// this register "OTYPER" defines output type 
		// push-pull or open-drain mode
		// bits 31-16 = 0 to be in reset state constantly,
		// bits 0-15 set value for open-drain only rest of cases are 0
		// bit 8 set value 0 for push-pull output mode 
		// and bit 9 set value 0 by default 
//	ptr_init_data->out = (uint32_t) 0x0000ffffU | LOW << DIG_8 | LOW << DIG_9; 
//	ptr_init_data->out = (uint32_t) 0x0000ffffU;
	ptr_init_data->out = (uint32_t) 0x00000000U | LOW << DIG_8 | LOW << DIG_9;
	
	
	// this register "OSPEEDR" defines speed 
		// 0x000000c0U is reset value for OSPEEDR of port B 
		// [10] for 8th pin ( 16th and 17th bits ) 
		// pin 9 does not need any settings here because it will be input
//	ptr_init_data->speed = (uint32_t) 0x000000c0U | MY_SPEED << PIN_8;
//	ptr_init_data->speed = (uint32_t) 0x000000c0U;
	ptr_init_data->speed = (uint32_t) 0x000000c0U | MY_SPEED << PIN_8;
	
	
	// this register "PUPDR" defines output type 
		// 0x00000100U is reset value for PUPDR of port B
		// 9th pin needs to be set with [00] (without pull-up/pull-down)
		// 10th pin needs to be set with [00] (without pull-up/pull-down)
//	ptr_init_data->pull = (uint32_t) 0x00000100U | LOW << DIG_16 | LOW << DIG_17 | LOW << DIG_18 | LOW << DIG_19;
//	ptr_init_data->pull = (uint32_t) 0x00000100U;
	ptr_init_data->pull = (uint32_t) 0x00000100U | LOW << DIG_16 | LOW << DIG_17 | LOW << DIG_18 | LOW << DIG_19;
	
		//other settings for port E for testing only
//		ptr_init_data->mode = (uint32_t) 0x0000000 | ALTERNATE_MODE << PIN_8 | INPUT_MODE << PIN_9; 
//		ptr_init_data->out = (uint32_t) 0x00000000U | LOW << DIG_8 | LOW << DIG_9;
//		ptr_init_data->speed = (uint32_t) 0x00000000U | MY_SPEED << PIN_8;
//		ptr_init_data->pull = (uint32_t) 0x00000000U | LOW << DIG_16 | LOW << DIG_17 | LOW << DIG_18 | LOW << DIG_19;
	
	// set it all to port B
	my_GPIO_init_port(GPIO_B, ptr_init_data);
	
}

void my_GPIO_init_port(GPIO_port *GPIO_port_X, GPIO_init_data* _init_data)
{
	
	GPIO_port_X->moder = _init_data->mode;
	GPIO_port_X->otyper = _init_data->out;
	GPIO_port_X->ospeedr = _init_data->speed;
	GPIO_port_X->pupdr = _init_data->pull;
}

void my_send_pack(GPIO_port * port_x, uint32_t content, uint32_t timer)
{
	//uint32_t start_bit = 0x00U;
	//uint32_t stop_bit = 0xFFU;
	unsigned char data[8] = {0x0};
	// 0) sort each separate bit to particular item
	for (int i = 0; i < 8; i++)
	{
		data[i] = (unsigned char) content & (HIGH << i);
		data[i] >>= i;
	}
	
	// 1) send start bit
	port_x->odr = port_x->odr & ~(HIGH << DIG_8);
	my_delay(timer);
	// 2) send 0 th data bit
	// 3) send 1 th data bit
	// 4) send N-1 th data bit
	// 5) send N th data bit
	for (uint16_t data_counter = 0x1U; data_counter < 0x8U; data_counter++)
	{
		port_x->odr = (uint32_t) data[data_counter] << DIG_8;
		my_delay(timer);
	}
	// 6) send stop bit
	port_x->odr = (uint32_t) HIGH << DIG_8;
	my_delay(timer);
}

void my_delay(uint32_t count)
{
	while(count != 0)
		--count;
}
// !!!!!!!!!!!!!!!! to test UART Bray's Terminal
