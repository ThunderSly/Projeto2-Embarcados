/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <stdio.h>
#include <asf.h>
#include <string.h>

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/** Header printf */
#define STRING_EOL    "\r"
#define STRING_HEADER "-- AFEC Temperature Sensor Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)

/************************************************************************/
/* Globals                                                              */
/************************************************************************/


/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;
volatile bool g_is_res_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;
volatile uint32_t g_res_value = 0;

volatile bool g_delay = false;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 2
#define AFEC_CHANNEL_RES_PIN 5


// LED
#define LED_PIO           PIOC
#define LED_PIO_ID        12
#define LED_PIO_IDX       8u
#define LED_PIO_IDX_MASK  (1u << LED_PIO_IDX)


// Analog Button
#define BUT0_PIO_ID				ID_PIOC
#define BUT0_PIO				PIOC
#define BUT0_PIO_IDX			30
#define BUT0_PIO_IDX_MASK		(1 << BUT0_PIO_IDX)
#define BUT0_DEBOUNCING_VALUE   79

#define BUT1_PIO_ID				ID_PIOA
#define BUT1_PIO				PIOA
#define BUT1_PIO_IDX			3
#define BUT1_PIO_IDX_MASK		(1 << BUT1_PIO_IDX)
#define BUT1_DEBOUNCING_VALUE   79

#define BUT2_PIO_ID				ID_PIOC
#define BUT2_PIO				PIOC
#define BUT2_PIO_IDX			31
#define BUT2_PIO_IDX_MASK		(1 << BUT2_PIO_IDX)
#define BUT2_DEBOUNCING_VALUE   79

#define BUT3_PIO_ID				ID_PIOA
#define BUT3_PIO				PIOA
#define BUT3_PIO_IDX			19
#define BUT3_PIO_IDX_MASK		(1 << BUT3_PIO_IDX)
#define BUT3_DEBOUNCING_VALUE   79

#define BUT4_PIO_ID				ID_PIOD
#define BUT4_PIO				PIOD
#define BUT4_PIO_IDX			21
#define BUT4_PIO_IDX_MASK		(1 << BUT4_PIO_IDX)
#define BUT4_DEBOUNCING_VALUE   79

// Joystick 
#define BUT8_PIO_ID				ID_PIOA
#define BUT8_PIO				PIOA
#define BUT8_PIO_IDX			0
#define BUT8_PIO_IDX_MASK		(1 << BUT8_PIO_IDX)
#define BUT8_DEBOUNCING_VALUE   79

#define BUT9_PIO_ID				ID_PIOC
#define BUT9_PIO				PIOC
#define BUT9_PIO_IDX			17
#define BUT9_PIO_IDX_MASK		(1 << BUT9_PIO_IDX)
#define BUT9_DEBOUNCING_VALUE   79

#define BUT10_PIO_ID			ID_PIOD
#define BUT10_PIO				PIOD
#define BUT10_PIO_IDX			28
#define BUT10_PIO_IDX_MASK		(1 << BUT10_PIO_IDX)
#define BUT10_DEBOUNCING_VALUE   79

#define BUT11_PIO_ID			ID_PIOA
#define BUT11_PIO				PIOA
#define BUT11_PIO_IDX			4
#define BUT11_PIO_IDX_MASK		(1 << BUT11_PIO_IDX)
#define BUT11_DEBOUNCING_VALUE   79



//TERMINAR DEFINES

#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

volatile long g_systimer = 0;

volatile uint8_t flag_but0 = 0;
volatile uint8_t flag_but1 = 0;
volatile uint8_t flag_but2 = 0;
volatile uint8_t flag_but3 = 0;
volatile uint8_t flag_but4 = 0;
volatile uint8_t flag_but5 = 0;
volatile uint8_t flag_but6 = 0;
volatile uint8_t flag_but7 = 0;
volatile uint8_t flag_but8 = 0;
volatile uint8_t flag_but9 = 0;
volatile uint8_t flag_but10 = 0;
volatile uint8_t flag_but11 = 0;
volatile uint8_t flag_change = 0;

volatile uint8_t value_but0 = 0;
volatile uint8_t value_but1 = 0;
volatile uint8_t value_but2 = 0;
volatile uint8_t value_but3 = 0;
volatile uint8_t value_but4 = 0;
volatile uint8_t value_but5 = 0;
volatile uint8_t value_but6 = 0;
volatile uint8_t value_but7 = 0;
volatile uint8_t value_but8 = 0;
volatile uint8_t value_but9 = 0;
volatile uint8_t value_but10 = 0;
volatile uint8_t value_but11 = 0;

/************************************************************************/
/* Callbacks: / Handler                                                 */
/************************************************************************/

/**
 * \brief AFEC interrupt callback function.
 */
static void AFEC_Temp_callback(void)
{
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	g_is_conversion_done = true;
}

static void AFEC_Res_callback(void)
{
	g_res_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_RES_PIN);
	g_is_res_done = true;
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	afec_start_software_conversion(AFEC0);

}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
	afec_start_software_conversion(AFEC0);
	
}

void SysTick_Handler() {
	g_systimer++;
}

void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	 // RX - PB0  TX - PB1 
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);	
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEFolCos", 1000);
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN1337", 1000);
	usart_log("hc05_server_init", buffer_rx);
}

void Button0_Handler(void){
	flag_but0 = 1;
 	value_but0 = !pio_get(BUT0_PIO, PIO_INPUT, BUT0_PIO_IDX_MASK);
}

void Button1_Handler(void){
	flag_but1 = 1;
	value_but1 = !pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK);
}

void Button2_Handler(void){
	flag_but2 = 1;
	value_but2 = !pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK);
}

void Button3_Handler(void){
	flag_but3 = 1;
	value_but3 = !pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK);
}

void Button4_Handler(void){
	flag_but4 = 1;
	value_but4 = !pio_get(BUT4_PIO, PIO_INPUT, BUT4_PIO_IDX_MASK);
}
// 
// void Button5_Handler(void){
// 	flag_but5 = 1;
// 	value_but5 = !pio_get(BUT5_PIO, PIO_INPUT, BUT5_PIO_IDX_MASK);
// }
// 
// void Button6_Handler(void){
// 	flag_but6 = 1;
// 	value_but6 = !pio_get(BUT6_PIO, PIO_INPUT, BUT6_PIO_IDX_MASK);
// }
// 
// void Button7_Handler(void){
// 	flag_but7 = 1;
// 	value_but7 = !pio_get(BUT7_PIO, PIO_INPUT, BUT7_PIO_IDX_MASK);
// }
// 
void Button8_Handler(void){
	flag_but8 = 1;
	value_but8 = !pio_get(BUT8_PIO, PIO_INPUT, BUT8_PIO_IDX_MASK);
}

void Button9_Handler(void){
	flag_but9 = 1;
	value_but9 = !pio_get(BUT9_PIO, PIO_INPUT, BUT9_PIO_IDX_MASK);
}

void Button10_Handler(void){
	flag_but10 = 1;
	value_but10 = !pio_get(BUT10_PIO, PIO_INPUT, BUT10_PIO_IDX_MASK);
}

void Button11_Handler(void){
	flag_but11 = 1;
	value_but11 = !pio_get(BUT11_PIO, PIO_INPUT, BUT11_PIO_IDX_MASK);
}

static int32_t convert_adc_to_axis(int32_t ADC_value){
	
	return (ADC_value-2200);
}

static void config_ADC_TEMP_RES(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_2,	AFEC_Temp_callback, 1);
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_5,	AFEC_Res_callback, 1);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, 0x200);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_RES_PIN, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa convers?o */
	//afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	//afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
}



void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, ((ul_sysclk) / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}
void BUT_init(void){
	//TERMINAR INIT PARA TODOS OS BOTOES
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT4_PIO_ID);
	pio_set_input(BUT4_PIO, BUT4_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT0_PIO_ID);
	pio_set_input(BUT0_PIO, BUT0_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT8_PIO_ID);
	pio_set_input(BUT8_PIO, BUT8_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT9_PIO_ID);
	pio_set_input(BUT9_PIO, BUT9_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT10_PIO_ID);
	pio_set_input(BUT10_PIO, BUT10_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT11_PIO_ID);
	pio_set_input(BUT11_PIO, BUT11_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT0_PIO, BUT0_PIO_IDX_MASK);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_enable_interrupt(BUT4_PIO, BUT4_PIO_IDX_MASK);
	pio_enable_interrupt(BUT8_PIO, BUT8_PIO_IDX_MASK);
	pio_enable_interrupt(BUT9_PIO, BUT9_PIO_IDX_MASK);
	pio_enable_interrupt(BUT10_PIO, BUT10_PIO_IDX_MASK);
	pio_enable_interrupt(BUT11_PIO, BUT11_PIO_IDX_MASK);
	
	pio_handler_set(BUT0_PIO, BUT0_PIO_ID, BUT0_PIO_IDX_MASK, PIO_IT_EDGE, Button0_Handler);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_EDGE, Button1_Handler);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_EDGE, Button2_Handler);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_EDGE, Button3_Handler);
	pio_handler_set(BUT4_PIO, BUT4_PIO_ID, BUT4_PIO_IDX_MASK, PIO_IT_EDGE, Button4_Handler);
	pio_handler_set(BUT8_PIO, BUT8_PIO_ID, BUT8_PIO_IDX_MASK, PIO_IT_EDGE, Button8_Handler);
	pio_handler_set(BUT9_PIO, BUT9_PIO_ID, BUT9_PIO_IDX_MASK, PIO_IT_EDGE, Button9_Handler);
	pio_handler_set(BUT10_PIO, BUT10_PIO_ID, BUT10_PIO_IDX_MASK, PIO_IT_EDGE, Button10_Handler);
	pio_handler_set(BUT11_PIO, BUT11_PIO_ID, BUT11_PIO_IDX_MASK, PIO_IT_EDGE, Button11_Handler);
	
	NVIC_EnableIRQ(BUT0_PIO_ID);
	NVIC_SetPriority(BUT0_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT4_PIO_ID);
	NVIC_SetPriority(BUT4_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT8_PIO_ID);
	NVIC_SetPriority(BUT8_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT9_PIO_ID);
	NVIC_SetPriority(BUT9_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT10_PIO_ID);
	NVIC_SetPriority(BUT10_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT11_PIO_ID);
	NVIC_SetPriority(BUT11_PIO_ID, 1);
};

int main (void)
{
	printf("comecou");
	board_init();
	sysclk_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	
	/* inicializa e configura adc */
	config_ADC_TEMP_RES();
	
	
	
	TC_init(TC0, ID_TC1, 1, 1);
	TC_init(TC0, ID_TC0, 0, 10);
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	pio_set(PIOC, LED_PIO_IDX_MASK);
	
	
	
	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	afec_start_software_conversion(AFEC0);
	if (g_is_conversion_done){
		afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
		afec_start_software_conversion(AFEC0);
	}
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	BUT_init();
	
	char eof = 'X';
	char buffer[1024];
	char axisX = '0';
	char axisY = '0';
	
	while(1) {
		char button0 = 'N';
		char button1 = 'N';
		char button2 = 'N';
		char button3 = 'N';
		char button4 = 'N';
		char button5 = 'N';
		char button6 = 'N';
		char button7 = 'N';
		char button8 = 'N';
		char button9 = 'N';
		char button10 = 'N';
		char button11 = 'N';
		int x = 0;
		int y = 0;
		if (g_is_conversion_done==true){
			x = (convert_adc_to_axis(g_ul_value));
			if(x>500){
				usart_put_string(USART1, "X positivo ");
				axisX = 'P';
				flag_change = 1;
			} else if(x <-500){
				usart_put_string(USART1, "X negativo ");
				axisX = 'N';
				flag_change = 1;
			} else{
				
				if (axisX != '0'){
					usart_put_string(USART1, "X parado ");
					axisX = '0';
					flag_change = 1;
				}
			}
			g_is_conversion_done = false;
		}
		if(g_is_res_done==true){
			y = (convert_adc_to_axis(g_res_value));
			if(y>500){
				usart_put_string(USART1, "Y positivo \n");
				axisY = 'P';
				flag_change = 1;
				} else if(y <-500){
				usart_put_string(USART1, "Y negativo \n");
				axisY = 'N';
				flag_change = 1;
				} else{
				if (axisY != '0'){
					usart_put_string(USART1, "Y parado \n");
					axisY = '0';
					flag_change = 1;
				}
			}
			g_is_res_done = false;
		}
		
		
		
		if(flag_but0) {
			if(value_but0){
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				usart_put_string(USART1, "Botao do analogico apertado \n");
				button0 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				
				button0 = '0';
			}
			flag_but0 = 0;
			flag_change = 1;
		}
		if(flag_but1) {
			if(value_but1){
				usart_put_string(USART1, "Botao 1 apertado \n");
				
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				button1 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button1 = '0';
			}
			flag_but1 = 0;
			flag_change = 1;
		}
		if(flag_but2) {
			if(value_but2){
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				usart_put_string(USART1, "Botao 2 apertado \n");
				button2 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button2 = '0';
			}
			flag_but2 = 0;
			flag_change = 1;
		}
		if(flag_but3) {
			if(value_but3){
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				usart_put_string(USART1, "Botao 3 apertado \n");
				button3 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button3 = '0';
			}
			flag_but3 = 0;
			flag_change = 1;
		}
		if(flag_but4) {
			if(value_but4){
				usart_put_string(USART1, "Botao 4 apertado \n");
				
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				
				button4 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button4 = '0';
			}
			flag_but4 = 0;
			flag_change = 1;
		}
		if(flag_but5) {
			if(value_but5){
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				
				button5 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button5 = '0';
			}
			flag_but5 = 0;
			flag_change = 1;
		}
		if(flag_but6) {
			if(value_but6){
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				
				button6 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button6 = '0';
			}
			flag_but6 = 0;
			flag_change = 1;
		}
		if(flag_but7) {
			if(value_but7){
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				
				button7 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button7 = '0';
			}
			flag_but7 = 0;
			flag_change = 1;
		}
		if(flag_but8) {
			if(value_but8){
				usart_put_string(USART1, "Botao 8 apertado \n");
				
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				
				button8 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button8 = '0';
			}
			flag_but8 = 0;
			flag_change = 1;
		}
		if(flag_but9) {
			if(value_but9){
				usart_put_string(USART1, "Botao 9 apertado \n");
				
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				
				button9 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button9 = '0';
			}
			flag_but9 = 0;
			flag_change = 1;
		}
		if(flag_but10) {
			if(value_but10){
				usart_put_string(USART1, "Botao 10 apertado \n");
				
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				
				button10 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button10 = '0';
			}
			flag_but10 = 0;
			flag_change = 1;
		}
		if(flag_but11) {
			if(value_but11){
				usart_put_string(USART1, "Botao 11 apertado \n");
				
				pio_clear(PIOC, LED_PIO_IDX_MASK);
				
				button11 = '1';
			}
			else{
				pio_set(PIOC, LED_PIO_IDX_MASK);
				button11 = '0';
			}
			flag_but11 = 0;
			flag_change = 1;
		}
		if(flag_change){
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button0);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button1);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button2);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button3);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button4);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button5);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button6);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button7);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button8);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button9);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button10);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, button11);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, axisX);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, axisY);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, eof);
			flag_change = 0;
		}
		else{
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		}
	}
}