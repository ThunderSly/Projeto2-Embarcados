#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// usart (bluetooth)
#define USART_COM_ID ID_USART0
#define USART_COM    USART0


volatile bool g_is_conversion_done = false;
volatile bool g_is_res_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;
volatile uint32_t g_res_value = 0;
volatile bool g_delay = false;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 2
#define AFEC_CHANNEL_RES_PIN 5

/** Header printf */
#define STRING_EOL    "\r"
#define STRING_HEADER "-- AFEC Temperature Sensor Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"a-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)

// LED
#define LED_PIO           PIOC
#define LED_PIO_ID        ID_PIOC
#define LED_PIO_IDX       8u
#define LED_PIO_IDX_MASK  (1u << LED_PIO_IDX)

#define BUZ_PIO           PIOD
#define BUZ_PIO_ID        ID_PIOD
#define BUZ_PIO_IDX       30u
#define BUZ_PIO_IDX_MASK  (1u << BUZ_PIO_IDX)

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

#define BUT5_PIO_ID				ID_PIOD
#define BUT5_PIO				PIOD
#define BUT5_PIO_IDX			20
#define BUT5_PIO_IDX_MASK		(1 << BUT5_PIO_IDX)
#define BUT5_DEBOUNCING_VALUE   79

#define BUT6_PIO_ID				ID_PIOD
#define BUT6_PIO				PIOD
#define BUT6_PIO_IDX			26
#define BUT6_PIO_IDX_MASK		(1 << BUT6_PIO_IDX)
#define BUT6_DEBOUNCING_VALUE   79

#define BUT7_PIO_ID				ID_PIOA
#define BUT7_PIO				PIOA
#define BUT7_PIO_IDX			21
#define BUT7_PIO_IDX_MASK		(1 << BUT7_PIO_IDX)
#define BUT7_DEBOUNCING_VALUE   79

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

/** RTOS  */
#define TASK_PROCESS_STACK_SIZE            (2*4096/sizeof(portSTACK_TYPE))
#define TASK_PROCESS_STACK_PRIORITY        (tskIDLE_PRIORITY)

typedef struct{
	uint button;
	int status;
} press;

typedef struct{
	uint axis;
	uint value;
} analog;

volatile long g_systimer = 0;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


/** prototypes */
static void ECHO_init(void);
static void USART1_init(void);
uint32_t usart_puts(uint8_t *pstring);

QueueHandle_t xQueueAfec;
QueueHandle_t xQueueBut;
volatile uint32_t g_tcCv = 0;


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure the console UART.
 */

static void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

uint32_t usart_puts(uint8_t *pstring){
	uint32_t i ;

	while(*(pstring + i))
		if(uart_is_tx_empty(USART_COM))
			usart_serial_putchar(USART_COM, *(pstring+i++));
}

static void AFEC_Temp_callback(void)
{
	analog x;
	x.axis = 0;
	x.value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	xQueueSendFromISR( xQueueAfec, &x, 0);
	
}

static void AFEC_Res_callback(void)
{
	analog y;
	y.axis = 1;
	y.value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_RES_PIN);
	xQueueSendFromISR( xQueueAfec, &y, 0);
}

void Button0_Handler(void){
	press press0;
	press0.button = 0;
 	press0.status = !pio_get(BUT0_PIO, PIO_INPUT, BUT0_PIO_IDX_MASK);
	xQueueSendFromISR( xQueueBut, &press0, 0);
}
 
void Button1_Handler(void){
	press press1;
	press1.button = 1;
	press1.status = !pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press1, 0);
 }

void Button2_Handler(void){
	press press2;
	press2.button = 2;
	press2.status = !pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK);
	xQueueSendFromISR( xQueueBut, &press2, 0);
}

void Button3_Handler(void){
	press press3;
	press3.button = 3;
	press3.status = !pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press3, 0);
}

void Button4_Handler(void){
	press press4;
	press4.button = 4;
	press4.status = !pio_get(BUT4_PIO, PIO_INPUT, BUT4_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press4, 0);
}

void Button5_Handler(void){
	press press5;
	press5.button = 5;
	press5.status = !pio_get(BUT5_PIO, PIO_INPUT, BUT5_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press5, 0);
}

void Button6_Handler(void){
	press press6;
	press6.button = 6;
	press6.status = !pio_get(BUT6_PIO, PIO_INPUT, BUT6_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press6, 0);
}

void Button7_Handler(void){
	press press7;
	press7.button = 7;
	press7.status = !pio_get(BUT7_PIO, PIO_INPUT, BUT7_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press7, 0);
}

void Button8_Handler(void){
	press press8;
	press8.button = 8;
	press8.status = !pio_get(BUT8_PIO, PIO_INPUT, BUT8_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press8, 0);
}

void Button9_Handler(void){
	press press9;
	press9.button = 9;
	press9.status = !pio_get(BUT9_PIO, PIO_INPUT, BUT9_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press9, 0);
}

void Button10_Handler(void){
	press press10;
	press10.button = 10;
	press10.status = !pio_get(BUT10_PIO, PIO_INPUT, BUT10_PIO_IDX_MASK);
	xQueueSendFromISR(xQueueBut, &press10, 0);
}

void Button11_Handler(void){
	press press11;
	press11.button = 11;
	press11.status = !pio_get(BUT11_PIO, PIO_INPUT, BUT11_PIO_IDX_MASK);
	xQueueSendFromISR( xQueueBut, &press11, 0);
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

void io_init(void){
	//TERMINAR INIT PARA TODOS OS BOTOES
	
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);
	
	pio_set_input(BUT0_PIO, BUT0_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT4_PIO, BUT4_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT5_PIO, BUT5_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT6_PIO, BUT6_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT7_PIO, BUT7_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT8_PIO, BUT8_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT9_PIO, BUT9_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT10_PIO, BUT10_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT11_PIO, BUT11_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pio_set_debounce_filter(BUT0_PIO, BUT0_PIO_IDX_MASK, BUT0_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, BUT1_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, BUT2_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, BUT3_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT4_PIO, BUT4_PIO_IDX_MASK, BUT4_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT5_PIO, BUT5_PIO_IDX_MASK, BUT5_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT6_PIO, BUT6_PIO_IDX_MASK, BUT6_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT7_PIO, BUT7_PIO_IDX_MASK, BUT7_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT8_PIO, BUT8_PIO_IDX_MASK, BUT8_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT9_PIO, BUT9_PIO_IDX_MASK, BUT9_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT10_PIO, BUT10_PIO_IDX_MASK, BUT10_DEBOUNCING_VALUE);
	pio_set_debounce_filter(BUT11_PIO, BUT11_PIO_IDX_MASK, BUT11_DEBOUNCING_VALUE);
	
	pio_handler_set(BUT0_PIO, BUT0_PIO_ID, BUT0_PIO_IDX_MASK, PIO_IT_EDGE, Button0_Handler);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_EDGE, Button1_Handler);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_EDGE, Button2_Handler);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_EDGE, Button3_Handler);
	pio_handler_set(BUT4_PIO, BUT4_PIO_ID, BUT4_PIO_IDX_MASK, PIO_IT_EDGE, Button4_Handler);
	pio_handler_set(BUT5_PIO, BUT5_PIO_ID, BUT5_PIO_IDX_MASK, PIO_IT_EDGE, Button5_Handler);
	pio_handler_set(BUT6_PIO, BUT6_PIO_ID, BUT6_PIO_IDX_MASK, PIO_IT_EDGE, Button6_Handler);
	pio_handler_set(BUT7_PIO, BUT7_PIO_ID, BUT7_PIO_IDX_MASK, PIO_IT_EDGE, Button7_Handler);
	pio_handler_set(BUT8_PIO, BUT8_PIO_ID, BUT8_PIO_IDX_MASK, PIO_IT_EDGE, Button8_Handler);
	pio_handler_set(BUT9_PIO, BUT9_PIO_ID, BUT9_PIO_IDX_MASK, PIO_IT_EDGE, Button9_Handler);
	pio_handler_set(BUT10_PIO, BUT10_PIO_ID, BUT10_PIO_IDX_MASK, PIO_IT_EDGE, Button10_Handler);
	pio_handler_set(BUT11_PIO, BUT11_PIO_ID, BUT11_PIO_IDX_MASK, PIO_IT_EDGE, Button11_Handler);
	
	NVIC_EnableIRQ(BUT0_PIO_ID);
	NVIC_SetPriority(BUT0_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 5);

	NVIC_EnableIRQ(BUT4_PIO_ID);
	NVIC_SetPriority(BUT4_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT5_PIO_ID);
	NVIC_SetPriority(BUT5_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT6_PIO_ID);
	NVIC_SetPriority(BUT6_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT7_PIO_ID);
	NVIC_SetPriority(BUT7_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT8_PIO_ID);
	NVIC_SetPriority(BUT8_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT9_PIO_ID);
	NVIC_SetPriority(BUT9_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT10_PIO_ID);
	NVIC_SetPriority(BUT10_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT11_PIO_ID);
	NVIC_SetPriority(BUT11_PIO_ID, 5);
	
	pio_enable_interrupt(BUT0_PIO, BUT0_PIO_IDX_MASK);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_enable_interrupt(BUT4_PIO, BUT4_PIO_IDX_MASK);
	pio_enable_interrupt(BUT5_PIO, BUT5_PIO_IDX_MASK);
	pio_enable_interrupt(BUT6_PIO, BUT6_PIO_IDX_MASK);
	pio_enable_interrupt(BUT7_PIO, BUT7_PIO_IDX_MASK);
	pio_enable_interrupt(BUT8_PIO, BUT8_PIO_IDX_MASK);
	pio_enable_interrupt(BUT9_PIO, BUT9_PIO_IDX_MASK);
	pio_enable_interrupt(BUT10_PIO, BUT10_PIO_IDX_MASK);
	pio_enable_interrupt(BUT11_PIO, BUT11_PIO_IDX_MASK);
	
}

void usart_put_string(Usart *usart, char str[]) {
  usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
  uint32_t rx;
  uint32_t counter = 0;
  uint32_t start = xTaskGetTickCount();
  while( (xTaskGetTickCount() - start < timeout_ms) && (counter < bufferlen - 1)) {
    if(usart_read(usart, &rx) == 0) {
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

void hc05_config_server(void) {
  sysclk_enable_peripheral_clock(USART_COM_ID);
  usart_serial_options_t config;
  config.baudrate = 9600;
  config.charlength = US_MR_CHRL_8_BIT;
  config.paritytype = US_MR_PAR_NO;
  config.stopbits = false;
  usart_serial_init(USART_COM, &config);
  usart_enable_tx(USART_COM);
  usart_enable_rx(USART_COM);
  
  // RX - PB0  TX - PB1
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
  char buffer_rx[128];
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100); printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100); printf(buffer_rx);
    usart_send_command(USART0, buffer_rx, 1000, "AT", 100); printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEFolCos", 1000); printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT", 1000); printf(buffer_rx);
  usart_send_command(USART0, buffer_rx, 1000, "AT+PIN1337", 1000); printf(buffer_rx);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_bluetooth(void){
	xQueueAfec = xQueueCreate( 10, sizeof( analog ) );
	xQueueBut = xQueueCreate( 13, sizeof( press ) );
	printf("Bluetooth initializing \n");
	hc05_config_server();
	hc05_server_init();
	printf("init" );
	io_init();
	pio_clear(BUZ_PIO, BUZ_PIO_IDX_MASK);
	delay_ms(200);
	pio_set(BUZ_PIO, BUZ_PIO_IDX_MASK);
	char buttons[13];
	int x = 0;
	int y = 0;
	char axisX = '0';
	char axisY = '0';
	char eof = 'X';
	press press_main;
	char buffer[100];
	analog axis_main;

	while(1){
		for (int i = 0; i<=12; i++){
			buttons[i] = 'N';
		}
		if (xQueueReceive(xQueueBut, &(press_main), ( TickType_t )  1 / portTICK_PERIOD_MS)) {
			printf("Button: %d   Status: %d\n", press_main.button, press_main.status);
			if (press_main.status){
				buttons[press_main.button] = '1';
				pio_clear(LED_PIO, LED_PIO_IDX_MASK);
			}
			else{
				buttons[press_main.button] = '0';
				pio_set(LED_PIO, LED_PIO_IDX_MASK);
			}
		}
		if (xQueueReceive(xQueueAfec, &(axis_main), ( TickType_t )  1 / portTICK_PERIOD_MS)){
			if(axis_main.axis == 0){
				x = (convert_adc_to_axis(axis_main.value));
				if(x>500){
					usart_put_string(USART1, "X positivo \n");
					axisX = 'P';
					} else if(x <-500){
					usart_put_string(USART1, "X negativo \n");
					axisX = 'N';
					} else{
					
					if (axisX != '0'){
						usart_put_string(USART1, "X parado \n");
						axisX = '0';
					}
				}
			}
			else if (axis_main.axis == 1){
				y = (convert_adc_to_axis(axis_main.value));
				if(y>500){
					usart_put_string(USART1, "Y positivo \n");
					usart_put_string(USART1, "VAI SE FUDER ATMEL/SAME \n");
					axisY = 'P';
					} else if(y <-500){
					usart_put_string(USART1, "Y negativo \n");
					axisY = 'N';
					} else{
						if (axisY != '0'){
							usart_put_string(USART1, "Y parado \n");
							axisY = '0';
						}
					}
			}
		}
		for (int i = 0; i<=12; i++){
			while(!usart_is_tx_ready(USART_COM));
			usart_write(USART_COM, buttons[i]);
		}
		
		while(!usart_is_tx_ready(USART_COM));
		usart_write(USART_COM, axisX);
		while(!usart_is_tx_ready(USART_COM));
		usart_write(USART_COM, axisY);
		while(!usart_is_tx_ready(USART_COM));
		usart_write(USART_COM, eof);
		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
}

void task_afec(void){
	config_ADC_TEMP_RES();
	const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
	for(;;){
		afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
		afec_start_software_conversion(AFEC0);
		vTaskDelay(xDelay);
		afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
		afec_start_software_conversion(AFEC0);
		vTaskDelay(xDelay);

	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void){
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(BUZ_PIO, BUZ_PIO_IDX_MASK, 0, 0, 0);
	pio_set(BUZ_PIO, BUZ_PIO_IDX_MASK);
	//pio_clear(LED_PIO, LED_PIO_IDX_MASK);
	//pio_set(BUZ_PIO, BUZ_PIO_IDX_MASK);
	//delay_us(100);
	
	
	
	

	/* Initialize the console uart */
	configure_console();

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_PROCESS_STACK_SIZE, NULL,	TASK_PROCESS_STACK_PRIORITY, NULL);
	
	
	
	if (xTaskCreate(task_afec, "afec", TASK_PROCESS_STACK_SIZE, NULL, TASK_PROCESS_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test afec task\r\n");
	}
	//pio_clear(BUZ_PIO, BUZ_PIO_IDX_MASK);

	/* Start the scheduler. */
	vTaskStartScheduler();
	
	

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
