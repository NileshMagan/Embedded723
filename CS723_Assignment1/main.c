// Standard includes
#include "stddef.h"
#include "stdio.h"
#include "string.h"

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
#include "sys/alt_alarm.h"

#include "altera_avalon_pio_regs.h"
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Task Priorities
#define SWITCH_POLLING_TASK_PRIORITY 5
#define KEYBOARD_LOGIC_PRIORITY 4
#define COMPUTE_TASK_PRIORITY 3
#define OUTPUT_LOGIC_TASK_PRIORITY 2
#define VGA_OUTPUT_TASK_PRIORITY 1

// Definition of Time delay
#define portMAX_DELAY 0

// Definition of Message Queue
#define   FREQUENCY_DATA_QUEUE_SIZE  10
#define   TIMER_200_QUEUE_SIZE  10

QueueHandle_t timer200Queue;

static alt_alarm timer500;

// Definition of Semaphore
SemaphoreHandle_t counterSemaphore0, counterSemaphore1, counterSemaphore2;
//
//unsigned int[FREQUENCY_DATA_QUEUE_SIZE] frequencyDataQueue = {0}; // Happy with the size?
// TODO: Need to decide between making our own queue^ and using an RTOS queue (without interrupts)

// Global variables
alt_u32 tickPerSecond; // Constants
unsigned int context;
static volatile unsigned int currentSwitchValue; // Data
static volatile unsigned int TOF_500, switchChanged, enterMaintenanceState, timerStarted; // Flags
static volatile unsigned int systemState; // State
// 	0  IDLE
//  1  STABLE
//  2  UNSTABLE
//  3  MAINTENANCE




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
int aboveRateOfFrequency(void);
int belowThresholdFrequency(void);
void updateLEDs(unsigned int greenLEDValue, unsigned int redLEDValue);


/************/
/*  ISRs     */
/************/
alt_u32 timer500ISR(void* context)
{
	//Set global flag
	TOF_500 = 1;
	
	// Give the semaphore
	xSemaphoreGive(counterSemaphore1);
	return 0;
}


void buttonsISR(void* context) {

	//Read edge capture register's value
	int buttonClick = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	if (buttonClick == 4) { // Is this the right button press?
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
	// TODO

	// PERFORM BASIC CALCULATION (sample -> f)
	// TODO

	// ADD TO QUEUE
	// TODO

	// RESET BITS FOR NEXT ISR
	// TODO

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
			// TODO: Adds key value to queue
		IOWR(SEVEN_SEG_BASE,0 ,key);
		}
	}

	// Give the semaphore
	xSemaphoreGive(counterSemaphore0);
}


/************/
/*  METHODS */
/************/


int aboveRateOfFrequency(void) {
	// Check most recent value of frequency queue, compare it previous and find gradient
	// TODO
	int length = frequencyQ.length();
	result = (frequencyQ[length] - frequencyQ[length - 1])/timePeriod;
	return (result > frequencyChangeThreshold); // 1 is Unstable
}


int belowThresholdFrequency(void) {
	// TODO
	// Check most recent value of frequency queue, compare it to threshold
	int length = frequencyQ.length();
	return (frequencyQ[length] < frequencyLowerThreshold); // 1 is Unstable
}

void updateLEDs(unsigned int greenLEDValue, unsigned int redLEDValue) {
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLEDValue);
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, redLEDValue);
}




/************/
/*   TASKS  */
/************/

void switchPollingTask(void *pvParameters)
{
	while (1)
	{
		//Read edge capture register's value
		int switch_value = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

		if (switch_value != currentSwitchValue) {
			//Set global flag
			switchChanged = 1;
			currentSwitchValue = switch_value;
			xSemaphoreGive(counterSemaphore1);
		}
	}
}

void keyboardLogicTask(void *pvParameters)
{
	while (1)
	{
		// TODO
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
		xSemaphoreTake(counterSemaphore1, portMAX_DELAY);

		unsigned int nextSystemState = systemState;

		if (enterMaintenanceState) {
			nextSystemState = 3; // 3 is Maintenance state

			// TODO: May need to use a mutex/semaphore to reset
			enterMaintenanceState = 0;
			// May need to use a mutex/semaphore to reset

		} else {
			int frequencyUnstable = aboveRateOfFrequency() || belowThresholdFrequency();

			if ( frequencyUnstable ) {
				nextSystemState = 2; // 2 is Unstable state
			} else if ( !frequencyUnstable ) {
				nextSystemState = 1; // 1 is Stable state
			}
		}

		// If nextState != prevState then restart timers,
		if (systemState != nextSystemState || TOF_500 || switchChanged) {

			if (nextSystemState != 3) { // 3 is Maintenance state
				// Store timer TODO: Get timer syntax
				xQueueSendToBack( timer200Queue, alt_nticks()/tickPerSecond, portMAX_DELAY );
			}

			// Clear timer flag: TODO: Check no mutex needed
			TOF_500 = 0;
			// Clear switch flag // TODO: Check no mutex needed
			switchChanged = 0;
			//Give semaphore // TODO: Check no mutex needed
			systemState = nextSystemState;
			xSemaphoreGive(counterSemaphore2); // Trigger output logic
			// xSemaphoreGive(counterSemaphore1); // TODO: Give back compute semaphore?
		}

	}
}



void outputLogicTask(void *pvParameters)
{
	while (1)
	{
		// TODO
		xSemaphoreTake(counterSemaphore2, portMAX_DELAY);
		switch(systemState) {
			case 0: // IDLE
			case 1: // STABLE
				// Stop 500ms timer
				if (timerStarted == 1) {
					alt_alarm_stop(&timer500);
				}
				checkSwitches();
					currentSwitchValue;
				checkLED();
					int redValue = 	IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
					int greenValue = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
				checkPriority();
					// Compare currentSwitch with Green
					if (greenValue == 0) {
						redValue = currentSwitchValue;
					} else {
						greenValue = redValue ^ currentSwitchValue;
						int bitPosition = checkPriority(greenValue, 1); // Check for high priority, returns
						greenValue = toggleBit(bitPosition, greenValue);
						redValue = toggleBit(bitPosition, redValue);
					}

				// Update LEDs
				updateLEDs(greenValue, redValue); // Red LEDs mimick switch state, Green LEDs all off

				// Start 500ms timer & set flag
				alt_alarm_start(&timer500, 500, timer500ISR, context);
				timerStarted = 1;
			case 2: // UNSTABLE
				// Stop 500ms timer
				if (timerStarted == 1) {
					alt_alarm_stop(&timer500);
				}

				checkSwitches();
				checkLED();
				checkPriority();
				updateLEDs();

				// Start 500ms timer & set flag
				alt_alarm_start(&timer500, 500, timer500ISR, context);
				timerStarted = 1;
			case 3: // MAINTENANCE
				// Stop 500ms timer
				if (timerStarted == 1) {
					alt_alarm_stop(&timer500);
				}

				// Update LEDs
				updateLEDs(0, currentSwitchValue); // Red LEDs mimick switch state, Green LEDs all off
		}
	}
}

void vgaOutputTask(void *pvParameters)
{
	while (1)
	{
		// TODO
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
	counterSemaphore2 = xSemaphoreCreateCounting( 9999, 1 );

	timer200Queue = xQueueCreate(TIMER_200_QUEUE_SIZE, sizeof(alt_u32));

	tickPerSecond = alt_ticks_per_second();
	context = 0;
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
	timerStarted = 0;
	switchChanged = 0;
	enterMaintenanceState = 0;
	currentSwitchValue = 0;

	// State
	systemState = 0; //
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
	initFlags();
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


