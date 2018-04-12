// Standard includes
#include "stddef.h"
#include "stdio.h"
#include "string.h"
#include <math.h>

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
#define   FREQUENCY_DATA_QUEUE_SIZE  100
#define   TIMER_200_QUEUE_SIZE  10
#define   KEYBOARD_QUEUE_SIZE  20
#define   REACTION_ARRAY_SIZE  5

QueueHandle_t timer200Queue, frequencyQueue, keyboardQueue;

static alt_alarm timer500;

// Definition of Semaphore
SemaphoreHandle_t counterSemaphore0, counterSemaphore1, counterSemaphore2;


// Global variables
alt_u32 tickPerSecond; // Constants
unsigned int context;
static volatile reactionTimes[REACTION_ARRAY_SIZE], frequencyData[FREQUENCY_DATA_QUEUE_SIZE],  rateOfChangeData[FREQUENCY_DATA_QUEUE_SIZE]; // Storage arrays
static volatile unsigned int currentSwitchValue; // Data
static volatile unsigned int TOF_500, switchChanged, enterMaintenanceState, timerStarted; // Flags
static volatile unsigned int systemState; // State
static volatile unsigned int levelThreshold, rateOfChangeThreshold;
static volatile unsigned int frequencyIndex;
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
void checkNewFrequencyValues(void);
void addToArray(alt_u32 numberTodAdd);
void handleReactionTimer(void);
int aboveRateOfFrequency(void);
int belowThresholdFrequency(void);
void updateLEDs(unsigned int greenLEDValue, unsigned int redLEDValue);
int checkPriority(int value, int priority);
int highestBit(int val);
int lowestBit(int val);

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
	double temp = IORD(FREQUENCY_ANALYSER_BASE, 0);

	// PERFORM BASIC CALCULATION (sample -> f)
	temp = 16000/temp;

	// ADD TO QUEUE
	xQueueSendToBackFromISR(frequencyQueue, temp, portMAX_DELAY);

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
		switch ( decode_mode )
		{
		 case KB_ASCII_MAKE_CODE :
			xQueueSendToBackFromISR(keyboardQueue, ascii, portMAX_DELAY);
			printf ( "ASCII   : %x\n", key ) ;

			// Give the semaphore
			xSemaphoreGive(counterSemaphore0);
			break ;
		 default :
			printf ( "DEFAULT   : %x\n", key ) ;
			break ;
		}
	}
}


/************/
/*  METHODS */
/************/


void checkNewFrequencyValues(void) {
	while(uxQueueMessagesWaiting( frequencyQueue ) != 0) {
		xQueueReceive( frequencyQueue, frequencyData+frequencyIndex, 0 );

		//calculate frequency RoC
		if(frequencyIndex==0){
			rateOfChangeData[0] = (frequencyData[0]-frequencyData[99]) * 2.0 * frequencyData[0] * frequencyData[99] / (frequencyData[0]+frequencyData[99]);
		}
		else{
			rateOfChangeData[frequencyIndex] = (frequencyData[frequencyIndex]-frequencyData[frequencyIndex-1]) * 2.0 * frequencyData[frequencyIndex]* frequencyData[frequencyIndex-1] / (frequencyData[frequencyIndex]+frequencyData[frequencyIndex-1]);
		}

		if (rateOfChangeData[frequencyIndex] > 100.0){
			rateOfChangeData[frequencyIndex] = 100.0;
		}
		frequencyIndex = ++frequencyIndex%100;
	}
}

int aboveRateOfFrequency(void) {
	// Check most recent value of frequency queue, compare it previous and find gradient
	return (rateOfChangeData[frequencyIndex - 1] > rateOfChangeThreshold); // 1 is Unstable
}

void handleReactionTimer(void) {
	alt_u32 previousTime, newTime, differenceInTime;
	xQueueReceive( timer200Queue, previousTime ,portMAX_DELAY );

	// Find difference
	newTime = alt_nticks()/tickPerSecond;
	differenceInTime = newTime - previousTime;
	addToArray(differenceInTime);
}

int belowThresholdFrequency(void) {
	// Check most recent value of frequency queue, compare it to threshold
	return (frequencyData[frequencyIndex - 1] > levelThreshold); // 1 is Unstable
}

void updateLEDs(unsigned int greenLEDValue, unsigned int redLEDValue) {
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLEDValue);
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, redLEDValue);
}

int checkPriority(int value, int priority){
	if (priority == 1) { // Find highest
		return highestBit(value);
	} else { // Find lowest
		return lowestBit(value);
	}
}

void addToArray(alt_u32 numberTodAdd) {
	int length = sizeof(reactionTimes)/sizeof(reactionTimes[0]);
	for (int z = (length - 1); z > 0 ; z--) {
		reactionTimes[z] = reactionTimes[z - 1];
	}
	reactionTimes[0] = numberTodAdd;
}

int highestBit(int val) {
	// Return NULL if no bits are 1
    if (!val)
        return NULL;

    // Loop through to find highest bit
    int shiftedValue = 1;

    while (val >>= 1)
    	shiftedValue <<= 1;

    // Return the number divided by base 2 to find the index
    return log(shiftedValue)/log(2);
}

int lowestBit(int val) {
	// Return NULL if no bits are 1
    if (!val)
        return NULL;

    // Loop through to find lowest bit
    int index;
    for(index = 0;!(val & 1);index++)
        val >>= 1;

    return index;
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

	// Array to hold keys
	int length = 5;
	char keyValues[length + 1] = {0};
	int i = 0;


	while (1)
	{
		xSemaphoreTake(counterSemaphore0,portMAX_DELAY);
		// Check if received enough characters yet
		if (i < 1 + length) {
			if (xQueueReceive(keyboardQueue, keyValues[i], portMAX_DELAY)) { //TODO check if receive buffer works
				// Check that we have started correctly
				if ((keyValues[0] != 'L') && (keyValues[0] != 'R')) {
					i++;
				}
			}
		} else { // Loop through filled array of characters and parse it to thresholds
			int thresholdTemp = 0;
			int invalidCharFlag = 0;


			for (int z = 0; z < length; z++) {
				int numberConverted = keyValues[z] - '0';

				// Validity checking
				if ((numberConverted >= 10) && (numberConverted <= -1)){
					invalidCharFlag = 1;
				}
				thresholdTemp = thresholdTemp + numberConverted*pow(10, length - 1 - z);
			}

			// Update thresholds and give semaphore if valid
			if (!invalidCharFlag) {
				if (keyValues[0] == 'L') {
					levelThreshold = thresholdTemp;
				} else if (keyValues[0] == 'R') {
					rateOfChangeThreshold = thresholdTemp;
				}
				xSemaphoreGive(counterSemaphore1);
			}

			i = 0;
		}
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
			checkNewFrequencyValues();
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
				break;
			case 1: // STABLE
				// Stop 500ms timer
				if (timerStarted == 1) {
					alt_alarm_stop(&timer500);
				}
				// Store current LED values
				int redValue = 	IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
				int greenValue = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);

				// If relay is not withholding loads, turn on all loads according to switches
				if (greenValue == 0) {
					redValue = currentSwitchValue;
				} else {
					// Update green value to store loads currently withheld and loads not yet handled
					greenValue = redValue ^ currentSwitchValue;

					// Check priority to find bit position of the load to turn on
					int bitPosition = checkPriority(greenValue, 1); // Check for high priority (1)

					// Turn load on
					greenValue = toggleBit(bitPosition, greenValue);
					redValue = toggleBit(bitPosition, redValue);
				}

				// Update LEDs
				updateLEDs(greenValue, redValue);

				// Check 200ms timer
				handleReactionTimer();

				// Start 500ms timer & set flag
				alt_alarm_start(&timer500, 500, timer500ISR, context);
				timerStarted = 1;
				break;
			case 2: // UNSTABLE
				// Stop 500ms timer
				if (timerStarted == 1) {
					alt_alarm_stop(&timer500);
				}

				// Store current LED values
				int redValue = 	IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
			    int greenValue = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);

			    if (redValue == 0) {
			    	break; // No loads to shed
				} else {
					// Update red value to store loads currently on and loads not yet handled
					redValue = greenValue ^ currentSwitchValue;

					// Check priority to find bit position of the load to turn off
					int bitPosition = checkPriority(redValue, 0); // Check for low priority (0)

					// Turn load on
					greenValue = toggleBit(bitPosition, greenValue);
					redValue = toggleBit(bitPosition, redValue);
				}

			    // Update LEDs
			    updateLEDs(greenValue, redValue);

				// Check 200ms timer
				handleReactionTimer();

				// Start 500ms timer & set flag
				alt_alarm_start(&timer500, 500, timer500ISR, context);
				timerStarted = 1;
				break;
			case 3: // MAINTENANCE
				// Stop 500ms timer
				if (timerStarted == 1) {
					alt_alarm_stop(&timer500);
				}

				// Update LEDs
				updateLEDs(0, currentSwitchValue); // Red LEDs mimick switch state, Green LEDs all off
				break;
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

	// INITIALISE/CREATE QUEUE
	frequencyQueue = xQueueCreate(FREQUENCY_DATA_QUEUE_SIZE, sizeof(double));
	timer200Queue = xQueueCreate(TIMER_200_QUEUE_SIZE, sizeof(alt_u32));
	keyboardQueue = xQueueCreate(KEYBOARD_QUEUE_SIZE, sizeof(char));


	frequencyIndex = 0;
	reactionTimes = [0,0,0,0,0];
	frequencyData = {0};
	rateOfChangeData = {0};
	tickPerSecond = alt_ticks_per_second();
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
	context = 0;

	// State
	systemState = 0;

	// Thresholds
	levelThreshold = 0;
	rateOfChangeThreshold = 0; // TODO
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

	return 0;
}

int initFrequency(void) {

	//Register the interrupt handler, context is unused so pass in garbage
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, context, frequencyISR);

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


