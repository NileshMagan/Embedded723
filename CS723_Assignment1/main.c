// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "system.h"
#include "io.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "sys/alt_irq.h"

#include <altera_avalon_pio_regs.h>
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Task Priorities
#define SWITCH_POLLING_TASK_PRIORITY 5
#define KEYBOARD_LOGIC_PRIORITY 4
#define COMPUTE_TASK_PRIORITY 3
#define OUTPUT_LOGIC_TASK_PRIORITY 2
#define VGA_OUTPUT_TASK_PRIORITY 1

// Definition of Message Queue
#define   FREQUENCY_DATA_QUEUE_SIZE  10
// QueueHandle_t msgqueue;

// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
SemaphoreHandle_t counterSemaphore0, counterSemaphore1;
//
//unsigned int[FREQUENCY_DATA_QUEUE_SIZE] frequencyDataQueue = {0}; // Happy with the size?
// TO DO: Need to decide between making our own queue^ and using an RTOS queue (without interrupts)

// Global variables
unsigned int TOF_500;
unsigned int currentSwitchValue, enterMaintenanceState;

// NEED TO IMPLEMENT OUR OWN QUEUE FUNCTIONALITY IN C

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);
int initFlags(void);
int initButtonsPIO(void);
int initFrequency(void);
int initKeyboard(void);
void initAll(void);

// Methods



/************/
/*  ISRs     */
/************/
alt_u32 timerISR(void* context)
{
	//Set global flag
	TOF_500 = 1;
	
	// Give the semaphore
	xSemaphoreGive(counterSemaphore1);
	return 0;
}


void buttonsISR(void* context) {

	//Read edge capture register's value
	int button_click = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	if (button_click == 4) { // Is this the right button press?
		enterMaintenanceState = 1;
	} else {
		enterMaintenanceState = 0;
	}

	//Reset edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

	printf("eMS: %d\n", enterMaintenanceState);
	//Give the semaphore
	xSemaphoreGive(counterSemaphore1);
}

void frequencyISR(void* context) {

	// Read ADC value
	// TO DO

	// PERFORM BASIC CALCULATION (sample -> f)
	// TO DO

	// ADD TO QUEUE
	// TO DO

	// RESET BITS FOR NEXT ISR
	// TO DO

	// Give the semaphore
	xSemaphoreGive(counterSemaphore1);
}

void ps2ISR(void* context) {

	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;
	status = decode_scancode (context, &decode_mode , &key , &ascii) ;
	if ( status == 0 ) //success
	{
		// print out the result
		switch ( decode_mode )
		{
			// TO DO: Adds key value to queue
		IOWR(SEVEN_SEG_BASE,0 ,key);
		}
	}

	// Give the semaphore
	xSemaphoreGive(counterSemaphore0);
}


/************/
/*  METHODS */
/************/



/************/
/*   TASKS  */
/************/

void switchPollingTask(void *pvParameters)
{
	while (1)
	{
		//Read edge capture register's value
		int switch_value = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

		int switch_changed = 0;
		if (switch_value != currentSwitchValue) {
			//Set global flag
			currentSwitchValue = switch_value;
			switch_changed = 1;
		}

		currentSwitchValue = switch_value;
		
		if (switch_changed) { // TO DO: May need to check switch value here as well
			xSemaphoreGive(counterSemaphore1);
		}
	}
}

void keyboardLogicTask(void *pvParameters)
{
	while (1)
	{
		// TO DO
		// Need to take value from queue and process it and see if it represents a threshold. If it does -> update global variable

		// Below is code from the keyboard example that may be helpful
				// case KB_ASCII_MAKE_CODE :
				// 	printf ( "ASCII   : %x\n", key ) ; 
				// 	break ;
				// case KB_LONG_BINARY_MAKE_CODE :
				// 	// do nothing
				// case KB_BINARY_MAKE_CODE :
				// 	printf ( "MAKE CODE : %x\n", key ) ;
				// 	break ;
				// case KB_BREAK_CODE :
				// 	// do nothing
				// default :
				// 	printf ( "DEFAULT   : %x\n", key ) ;
				// 	break ;
				// }
	}
}


void computeTask(void *pvParameters)
{
	while (1)
	{
		// TO DO
	}
}

void outputLogicTask(void *pvParameters)
{
	while (1)
	{
		// TO DO

	}
}

void vgaOutputTask(void *pvParameters)
{
	while (1)
	{
		// TO DO

	}
}



/************/
/**  INIT  **/
/************/
// This function simply creates initial data used in the scope of program
int initOSDataStructs(void)
{
	counterSemaphore0 = xSemaphoreCreateCounting( 9999, 1 );
	counterSemaphore1 = xSemaphoreCreateCounting( 9999, 1 );
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{	
	xTaskCreate(switchPollingTask, "switchPollingTask", TASK_STACKSIZE, NULL, SWITCH_POLLING_TASK_PRIORITY, NULL);
	xTaskCreate(keyboardLogicTask, "keyboardLogicTask", TASK_STACKSIZE, NULL, KEYBOARD_LOGIC_PRIORITY, NULL);
	xTaskCreate(computeTask, "computeTask", TASK_STACKSIZE, NULL, COMPUTE_TASK_PRIORITY, NULL);
	xTaskCreate(outputLogicTask, "outputLogicTask", TASK_STACKSIZE, NULL, OUTPUT_LOGIC_TASK_PRIORITY, NULL);
	xTaskCreate(vgaOutputTask, "vgaOutputTask", TASK_STACKSIZE, NULL, VGA_OUTPUT_TASK_PRIORITY, NULL);
	return 0;
}

// This function initialises all global flags
int initFlags(void)
{
	TOF_500 = 0;
	enterMaintenanceState = 0;
	currentSwitchValue = 0;
	return 0;
}

int initButtonsPIO(void)
{
	//Enable first four interrupts
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	// Reset the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	//Register the interrupt handler, context is unused so pass in garbage
	void* context = 0;
	alt_irq_register(PUSH_BUTTON_IRQ, context, buttonsISR);
	printf("Finished button init");
	return 0;
}

int initFrequency(void) {

	// INITIALISE/CREATE QUEUE
	// TODO

	// ENABLE ISR STUFF FOR FREQUENCY STUFF
	// TODO

	//Register the interrupt handler, context is unused so pass in garbage
	void* context = 0;
	//alt_irq_register(ADC_base_value?, context, frequencyController);

	return 0; // success
}

int initKeyboard(void) {
	// Open port
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	// Error
	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
		return 1;
	}

	// Reset buffer
	alt_up_ps2_clear_fifo (ps2_device) ;

	// Register the PS/2 interrupt
	alt_irq_register(PS2_IRQ, ps2_device, ps2ISR);

	IOWR_8DIRECT(PS2_BASE,4,1);


	return 0; // success
}



void initAll(void) {
	initOSDataStructs();
	//initCreateTasks();
	//initFlags();
	initButtonsPIO();
	//initFrequency();
	//initKeyboard();
	printf("Initialiased all\n");
	return;
}

/************/
/**  MAIN  **/
/************/

int main(int argc, char* argv[], char* envp[])
{
	initAll();
	vTaskStartScheduler();
	for (;;);
	return 0;
}


