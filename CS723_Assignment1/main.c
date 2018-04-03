// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <altera_avalon_pio_regs.h>

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Task Priorities
#define SWITCH_POLLING_TASK_PRIORITY 5
#define KEYBOARD_LOGIC_PRIORITY 4
#define COMPUTE_TASK_PRIORITY 3
#define OUTPUT_LOGIC_TASK_PRIORITY 2
#define VGA_OUTPUT_TASK_PRIORITY 1

// Definition of Message Queue
#define   MSG_QUEUE_SIZE  30
// QueueHandle_t msgqueue;

// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
SemaphoreHandle_t counterSemaphore1;

// Global variables
unsigned int TOF_500;
unsigned int currentSwitchValue, maintenanceState;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);
int initFlags(void);
int initButtonsPIO(void);



/* =========================================================================================================
 * INTERRUPT SERVICE ROUTINES
 * =========================================================================================================
 */

/* DESCRIPTION: Handles the traffic light timer interrupt
 * PARAMETER:   context - opaque reference to user data
 * RETURNS:     Number of 'ticks' until the next timer interrupt. A return value
 *              of zero stops the timer.
 */
alt_u32 tlc_timer_isr(void* context)
{
	// TO DO
	return 0;
}


/************/
/*  METHODS */
/************/

void buttonsController(void* context) {

	//Read edge capture register's value
	int button_click = IORD_ALTERA_AVALON_PIO_EDGE_CAP(BUTTONS_BASE);

	if (button_click == 1) { // Is this the right button press?
		maintenanceState = 1;
	} else { 
		maintenanceState = 0;
	}

	//reset edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTONS_BASE, 0);

	xSemaphoreGive(counterSemaphore1);
}




/************/
/*   TASKS  */
/************/

void switchPollingTask(void *pvParameters)
{
	while (1)
	{

		//Read edge capture register's value
		int switch_value = IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE);
		// Switch 17, the configuration switch, is on
		int switch_changed = 0;
		if (switch_value != currentSwitchValue) {
			//Set global flag
			currentSwitchValue = switch_value;
			switch_changed = 1;
		}

		currentSwitchValue = switch_value;
		
		if (switch_changed) {
			xSemaphoreGive(counterSemaphore1);
		}
	}
}

void keyboardLogicTask(void *pvParameters)
{
	while (1)
	{
		// TO DO
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
	// msgqueue = xQueueCreate( MSG_QUEUE_SIZE, sizeof( void* ) );
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
	maintenanceState = 0;
	currentSwitchValue = 0;
	return 0;
}

void initButtonsPIO(void)
{
	//Enable first four interrupts
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(BUTTONS_BASE, 0xf);
	// Reset the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTONS_BASE, 0x0);
	//Register the interrupt handler, context is unused so pass in garbage
	void* context = 0;
	alt_irq_register(BUTTONS_IRQ, context, buttonsController);

	return 0;
}

/************/
/**  MAIN  **/
/************/
int main(int argc, char* argv[], char* envp[])
{
	initOSDataStructs();
	initCreateTasks();
	initFlags();
	initButtonsPIO();
	vTaskStartScheduler();
	for (;;);
	return 0;
}


