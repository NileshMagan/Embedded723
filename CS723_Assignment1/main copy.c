// Standard includes
#include "stddef.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <math.h>

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Altera includes
#include "system.h"
#include "io.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include "sys/alt_timestamp.h"
#include "altera_avalon_pio_regs.h"
#include "alt_types.h"                 	
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"

// =============== Defines ===============

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Letters for keyboard interaction
#define L 76
#define R 82

// Definition of Task Priorities
#define SWITCH_POLLING_TASK_PRIORITY 3
#define KEYBOARD_LOGIC_PRIORITY 2
#define COMPUTE_TASK_PRIORITY 5
#define OUTPUT_LOGIC_TASK_PRIORITY 4
#define VGA_OUTPUT_TASK_PRIORITY 1

// Definition of computation constants
#define SAMPLING_FREQ 16000.0
#define THRESHOLD_NUMBER_LENGTH 5
#define POLLING_DELAY 300
#define VGA_DELAY 10

// Definition of Message Queue
#define FREQUENCY_DATA_QUEUE_SIZE  100
#define TIMER_200_QUEUE_SIZE  10
#define KEYBOARD_QUEUE_SIZE  20
#define REACTION_ARRAY_SIZE  100

//Definition of constants for VGA frequency plot
#define FREQPLT_ORI_X 101		// x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	// Pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		// y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	// Number of pixels per Hz (y axis scale)
#define BASE_TIME 42.9 			// Timer overflow value
#define ROCPLT_ORI_X 101		// x axis pixel position at the plot origin
#define ROCPLT_GRID_SIZE_X 5	// Pixel separation in the x axis between two data points
#define ROCPLT_ORI_Y 259.0		// y axis pixel position at the plot origin
#define ROCPLT_ROC_RES 0.5		// Number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 			// Minimum frequency to draw


// =============== Global Variables ===============

// Definition of Queues
QueueHandle_t frequencyQueue, keyboardQueue;

// Definition of Semaphores
SemaphoreHandle_t counterSemaphore0, counterSemaphore1, counterSemaphore2; 					// Counting
SemaphoreHandle_t binarySemaphore0, binarySemaphore1, binarySemaphore2, binarySemaphore3; 	// Binary

// Data storage arrays
double reactionTimes[REACTION_ARRAY_SIZE] = {1};
double frequencyData[FREQUENCY_DATA_QUEUE_SIZE] = {0};
double rateOfChangeData[FREQUENCY_DATA_QUEUE_SIZE] = {0};

//Mutex protected
static volatile unsigned int currentSwitchValue; 				// Guarded by binarySemaphore3
static volatile unsigned int systemState; 						// Guarded by binarySemaphore2
static volatile unsigned int frequencyIndex; 					// Guarded by binarySemaphore0
static volatile double prevTime500; 							// Guarded by binarySemaphore1
static volatile double levelThreshold, rateOfChangeThreshold; 	// Guarded by binarySemaphore0

// Not mutex protected
static volatile double timerOverflow, prevTime200;								// Data stroage for reaction time handling
static volatile unsigned int changedState, twoTimes, reactionCount;				// Flags for reaction time handling
static volatile unsigned int TOF_500, switchChanged, enterMaintenanceState; 	// Other flags 
static volatile unsigned int context;											// To be given where needed, never read

// Definition used for plotting frequency in VGA
typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

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
void addReactionTimeArray(double numberTodAdd);
void handleReactionTimer(void);
int aboveRateOfFrequency(void);
int belowThresholdFrequency(void);
void updateLEDs(unsigned int greenLEDValue, unsigned int redLEDValue);
int checkPriority(int value, int priority);
int highestBit(int val);
int lowestBit(int val);
int toggleBit(int bitPosition, int numberToToggle);
double getTime(void);
double maxValueArray(double Array[], int size);
double minValueArray(double Array[], int size);
double AverageOfArray(double Array[], int size);

/************/
/*  ISRs     */
/************/
void buttonsISR(void* context) {

	//Read edge capture register's value
	int buttonClick = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	if (buttonClick == 4) { 

		enterMaintenanceState = !enterMaintenanceState;

		//Give the semaphore, scheduling Compute task
		xSemaphoreGiveFromISR(counterSemaphore1, pdTRUE);
	}

	//Reset edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

void frequencyISR(void* context) {

	// Read ADC value
	double temp = (double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	// Perform basic calculation (sample -> f)
	temp = SAMPLING_FREQ/temp;

	// Add to queue
	xQueueSendToBackFromISR(frequencyQueue, &temp, pdFALSE);

	// Give the semaphore, scheduling Compute task
	xSemaphoreGiveFromISR(counterSemaphore1, context);
}

void ps2ISR(void* context) {
		char ascii;
		int status = 0;
		unsigned char key = 0;
		KB_CODE_TYPE decode_mode;


		// Determine is key was successfully scanned
		status = decode_scancode (context, &decode_mode , &key , &ascii) ;
		if ( status == 0 ) {

			// Handled the double occurance of ISR for single key press
			if (twoTimes == 1) {
				twoTimes = 0;

				switch (decode_mode) {

				 case KB_ASCII_MAKE_CODE :
				 	// Store ascii value on queue
					xQueueSendToBackFromISR(keyboardQueue, &ascii, pdTRUE);
					// Give semaphore, scheduling keyboard task
					xSemaphoreGiveFromISR(counterSemaphore0, context);
					break ;

				 default :
					break ;
				}

			} else {
				twoTimes++;
			}
		}
}


/************/
/*  METHODS */
/************/


void checkNewFrequencyValues(void) {
	while(uxQueueMessagesWaiting(frequencyQueue) != 0) {

		// Receive value from queue
		double temp;
		xQueueReceive(frequencyQueue, (void*) &temp, portMAX_DELAY);
		frequencyData[frequencyIndex] = temp;


		// Calculate frequency RoC
		if(frequencyIndex==0){
			rateOfChangeData[0] = fabs(frequencyData[0]-frequencyData[99]) * 2.0 * frequencyData[0] * frequencyData[99] / (frequencyData[0]+frequencyData[99]);
		}
		else{
			rateOfChangeData[frequencyIndex] = fabs(frequencyData[frequencyIndex]-frequencyData[frequencyIndex-1]) * 2.0 * frequencyData[frequencyIndex]* frequencyData[frequencyIndex-1] / (frequencyData[frequencyIndex]+frequencyData[frequencyIndex-1]);
		}

		if (rateOfChangeData[frequencyIndex] > 100.0){
			rateOfChangeData[frequencyIndex] = 100.0;
		}
		// Increments index 0-99 in a circular fashion 
		frequencyIndex = ++frequencyIndex%100;

	}
}

void handleReactionTimer(void) {
	double newTime, differenceInTime;
		// Find difference in time
		newTime = getTime();
		differenceInTime = newTime - prevTime200;
		// Add to array
		addReactionTimeArray(differenceInTime);
}

int aboveRateOfFrequency(void) {
	// Check most recent RoC value and compare to constraint value 
	if (!frequencyIndex) {
		return (rateOfChangeData[99] > rateOfChangeThreshold); // 1 is Unstable
	} else {
		return (rateOfChangeData[frequencyIndex - 1] > rateOfChangeThreshold); // 1 is Unstable
	}
}

int belowThresholdFrequency(void) {
	// Check most recent frequency value, compare it to threshold
	if (!frequencyIndex) {
		return (frequencyData[99] < levelThreshold); // 1 is Unstable
	} else {
		return (frequencyData[frequencyIndex - 1] < levelThreshold); // 1 is Unstable
	}
}

void updateLEDs(unsigned int greenLEDValue, unsigned int redLEDValue) {

	// Write to LEDs
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLEDValue);
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, redLEDValue);
}

int checkPriority(int value, int priority){

	if (priority == 1) { 
		// Find highest priority bit
		return highestBit(value);
	} else { 
		// Find lowest priority bit
		return lowestBit(value);
	}
}

void addReactionTimeArray(double numberTodAdd) {
	int length = sizeof(reactionTimes)/sizeof(reactionTimes[0]);
	int i;

	// Suffle all elements in array towards the back by one place
	for(i = (length - 1); i > 0; i--) {
		reactionTimes[i] = reactionTimes[i - 1];
	}
	// Store new value at front of array
	reactionTimes[0] = numberTodAdd;

	// Define how many elements are in the array until full
	if (reactionCount != REACTION_ARRAY_SIZE){
		reactionCount++;
	}
}

double maxValueArray(double Array[], int size) {
    int i;
    double maxValue = Array[0];

    // Cheack each array element to find the maximum value
    for (i = 1; i < size; ++i) {
        if ( Array[i] > maxValue ) {
            maxValue = Array[i];
        }
    }
    return maxValue;
}

double minValueArray(double Array[], int size) {
    int i;
    double minValue = Array[0];

    // Cheack each array element to find the minimum value
    for (i = 1; i < size; ++i) {
        if ( Array[i] < minValue ) {
            minValue = Array[i];
        }
    }
    return minValue;
}

double AverageOfArray(double Array[], int size) {
    int i;
    double averageValue = Array[0];

    // Access every element of the array to calculate the average
    for (i = 1; i < size; ++i) {
    	averageValue = Array[i];
    }
    return averageValue/size;
}

int highestBit(int val) {
	// Return NULL if no bits are 1
    if (!val)
        return 100;

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
        return 100;

    // Loop through to find lowest bit
    int index;
    for(index = 0;!(val & 1);index++)
        val >>= 1;

    return index;
}

int toggleBit(int bitPosition, int numberToToggle) {
	int shift = 1;
	// Shift 1 by bit position and XOR to toggle
	numberToToggle ^= shift << bitPosition;
	return numberToToggle;
}

double getTime(void) {

	// If timer has overflowed increment counter 
	if ((double)alt_timestamp() == 0) {
		timerOverflow++;
		// Start the overflowed timer
		alt_timestamp_start();
	}
	// Current time is the timestamp value (in ms) plus any overflows
	return ((double)alt_timestamp()/100000000)+(timerOverflow * BASE_TIME);
}

/************/
/*   TASKS  */
/************/

void switchPollingTask(void *pvParameters)
{

	printf("STARTED SWITCH POLLING TASK\n");

	while (1)
	{
		// Read edge capture register's value
		int switch_value = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

		xSemaphoreTake(binarySemaphore3, portMAX_DELAY);
		if (switch_value != currentSwitchValue) {
			// Set global flag and store value to global variable
			switchChanged = 1;
			currentSwitchValue = switch_value;
			xSemaphoreGive(counterSemaphore1);
		}
		xSemaphoreGive(binarySemaphore3);

		vTaskDelay(POLLING_DELAY);
	}
}

void keyboardLogicTask(void *pvParameters)
{

	printf("STARTED KEYBOARD LOGIC TASK\n");

	// Array to hold keys
	char keyValues[THRESHOLD_NUMBER_LENGTH + 1] = {0};
	int i = 0;
	int decimalIndex = 0;

	while (1)
	{
		xSemaphoreTake(counterSemaphore0,portMAX_DELAY);

		// Check if received enough characters yet
		if (i < THRESHOLD_NUMBER_LENGTH) {
				if (xQueueReceive(keyboardQueue, keyValues+i, portMAX_DELAY)) { 
					// Check that we have started correctly
					if ((keyValues[0] == L) || (keyValues[0] == R)) {
						int numberConverted = keyValues[i] - '0';

						// Validity checking
						if (!(((numberConverted >= 10) || (numberConverted <= -1)) && (numberConverted != -2) && (i != 0))) {
							i++;
						}
					} else {
						keyValues[i] = 0;
					}
				}
		// Loop through filled array of characters and set thresholds
		} else if (xQueueReceive(keyboardQueue, keyValues+i, portMAX_DELAY)) {
			float thresholdTemp = 0;
			int invalidCharFlag = 0;
			int decimalFlag = 0;

			int z;
		    for (z = 1; z < THRESHOLD_NUMBER_LENGTH + 1; z++) {
		    	// Find decimal place
		        if ((keyValues[z] == 46) && !decimalFlag) { 
		            decimalFlag = 1;
		            decimalIndex = z;
		        }

		        int numberConverted = keyValues[z] - '0';

		        // Validity checking
		        if (((numberConverted >= 10) && (numberConverted <= -1)) && !decimalFlag) {
		            invalidCharFlag = 1;
		        }


		        if (!decimalFlag) {
		        	// No decimal to handle
		            thresholdTemp = numberConverted + thresholdTemp*10;
		        } else if (decimalIndex != z) {
		        	// Handle decimal
		            thresholdTemp = numberConverted*pow(10, decimalIndex-z) + thresholdTemp;
		        }

		    }

			// Update thresholds
			if (!invalidCharFlag) {
				xSemaphoreTake(binarySemaphore0, portMAX_DELAY);
				if (keyValues[0] == L) {
					levelThreshold = thresholdTemp;
					printf("STORED L THRESHOLD: %f\n", levelThreshold);
				} else if (keyValues[0] == R) {
					rateOfChangeThreshold = thresholdTemp;
					printf("STORED R THRESHOLD: %f\n", rateOfChangeThreshold);
				}
				xSemaphoreGive(binarySemaphore0);

				// Reset array to 0
				for (z = 0 ; z < THRESHOLD_NUMBER_LENGTH + 1; z++) { keyValues[z] = 0; } 
				// Give semaphore, scheduling Compute task
				xSemaphoreGive(counterSemaphore1);
			}

			i = 0;
		}
	}
}



void computeTask(void *pvParameters)
{
	printf("STARTED COMPUTE TASK\n");

	while (1)
	{
		xSemaphoreTake(counterSemaphore1, portMAX_DELAY);

		// make local copy to mitigate need for mutex protection
		xSemaphoreTake(binarySemaphore2, portMAX_DELAY);
		unsigned int currentSystemState = systemState;
		xSemaphoreGive(binarySemaphore2);

		unsigned int nextSystemState = currentSystemState;

		xSemaphoreTake(binarySemaphore1, portMAX_DELAY);
		double difference = getTime() - prevTime500;

		// Check if 500 ms has passed and set flag
		if ((difference >= 0.5) && (prevTime500 != 0)) {
			TOF_500 = 1;
			prevTime500 = 0;
		}
		xSemaphoreGive(binarySemaphore1);


		// Update frequency array and determine stability
		xSemaphoreTake(binarySemaphore0, portMAX_DELAY);
		checkNewFrequencyValues();
		int frequencyUnstable = aboveRateOfFrequency() || belowThresholdFrequency();
		xSemaphoreGive(binarySemaphore0);

		// Determine system state
		if (enterMaintenanceState) {
			nextSystemState = 3; // 3 is Maintenance state
		} else {
			if ( frequencyUnstable ) {
				nextSystemState = 2; // 2 is Unstable state
			} else if ( !frequencyUnstable ) {
				nextSystemState = 1; // 1 is Stable state
			}
		}

		// Determine if load adjustment is needed 
		if ((currentSystemState != nextSystemState) || TOF_500 || switchChanged) {

//			printf("RoC: %d, Threshold: %d\n", aboveRateOfFrequency(), belowThresholdFrequency());
//			printf("StateChange: %d, TOV: %d, SWC: %d\n", (systemState != nextSystemState), TOF_500, switchChanged);

			// Store value for reaction time if first load is being shed/turned on
			if ((currentSystemState != nextSystemState) && (nextSystemState != 3)) { // 3 is Maintenance state
				// Store time
				prevTime200 = getTime();
				changedState = 1;
			}

			// Clear flags
			TOF_500 = 0;
			switchChanged = 0;


			// Write to global variable
			xSemaphoreTake(binarySemaphore2, portMAX_DELAY);
			systemState = nextSystemState;
			xSemaphoreGive(binarySemaphore2);

			// Give semaphore, scheduling output logic
			xSemaphoreGive(counterSemaphore2); 
		}

	}
}



void outputLogicTask(void *pvParameters)
{
	printf("STARTED OUTPUT LOGIC TASK\n");

	int redValue, greenValue;
	greenValue = 0;
	redValue = 0;

	while (1)
	{
		xSemaphoreTake(counterSemaphore2, portMAX_DELAY);

		int bitPosition;

		// make local copy to mitigate need for mutex protection
		xSemaphoreTake(binarySemaphore2, portMAX_DELAY);
		unsigned int currentSystemState = systemState;
		xSemaphoreGive(binarySemaphore2);

		switch(currentSystemState) {
			case 0: // IDLE
				break;
			case 1: // STABLE

				printf("---- STABLE ----\n");

				// Store current LED values
				redValue = 	IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
				greenValue = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);

				// make local copy to mitigate need for mutex protection
				xSemaphoreTake(binarySemaphore3, portMAX_DELAY);
				unsigned int localSwitchValue = currentSwitchValue;
				xSemaphoreGive(binarySemaphore3);


				// If relay is not withholding loads, turn on all loads according to switches
				if (greenValue == 0) {
					redValue = currentSwitchValue;

				    updateLEDs(greenValue, redValue);

				    // Check 200ms timer
				    if(changedState == 1){
					    handleReactionTimer();
					    changedState = 0;
				    }

				    break; // No loads to turn back on

				} else {
					// Update green value to store loads currently withheld and loads not yet handled
					greenValue = redValue ^ currentSwitchValue;

					// Check priority to find bit position of the load to turn on
					bitPosition = checkPriority(greenValue, 1); // Check for high priority (1)
					if(bitPosition != 100) {
						// Turn load on
						greenValue = toggleBit(bitPosition, greenValue);
						redValue = toggleBit(bitPosition, redValue);
					}
				}

				// Update LEDs
				updateLEDs(greenValue, redValue);

				// Check 200ms timer
				if(changedState == 1){
					handleReactionTimer();
					changedState = 0;
				}

				break;
			case 2: // UNSTABLE

				// DEBUG
				printf("---- UNSTABLE ----\n");

				// Store current LED values
				redValue = 	IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
			    greenValue = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);

			    if (redValue == 0) {

			    	greenValue = currentSwitchValue;
				    updateLEDs(greenValue, redValue);

//				    // Check 200ms timer
				    if(changedState == 1){
				    	handleReactionTimer();
				    	changedState = 0;
				    }

			    	break; // No loads to shed
				} else {

					// Update red value to store loads currently on and loads not yet handled
					redValue = greenValue ^ currentSwitchValue;

					// Check priority to find bit position of the load to turn off
					bitPosition = checkPriority(redValue, 0); // Check for low priority (0)
					if(bitPosition != 100) {
						// Turn load on
						greenValue = toggleBit(bitPosition, greenValue);
						redValue = toggleBit(bitPosition, redValue);
					}
				}

			    // Update LEDs
			    updateLEDs(greenValue, redValue);

				// Check 200ms timer
			    if(changedState == 1){
			   		handleReactionTimer();
			   		changedState = 0;
			   	}

				break;
			case 3: // MAINTENANCE

				printf("---- MAINTENANCE ----\n");

				// Update LEDs
				updateLEDs(0, currentSwitchValue); // Red LEDs mimick switch state, Green LEDs all off
				break;
		}

		xSemaphoreTake(binarySemaphore1, portMAX_DELAY);
		// Store time for 500ms
		prevTime500 = getTime();
		xSemaphoreGive(binarySemaphore1);

	}
}

void vgaOutputTask(void *pvParameters)
{
	printf("STARTED VGA OUTPUT TASK\n");

	while (1)
	{
		//initialize VGA controllers
		alt_up_pixel_buffer_dma_dev *pixel_buf;
		pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
		if(pixel_buf == NULL){
			printf("can't find pixel buffer device\n");
		}
		alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

		alt_up_char_buffer_dev *char_buf;
		char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
		if(char_buf == NULL){
			printf("can't find char buffer device\n");
		}
		alt_up_char_buffer_clear(char_buf);



		// Set up plot axes for top and bottom graphs - Draw lines
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

		// Set up plot axes for top and bottom graphs - Put "frequency" on Y axis and put scale numbers on it
		alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
		alt_up_char_buffer_string(char_buf, "52", 10, 7);
		alt_up_char_buffer_string(char_buf, "50", 10, 12);
		alt_up_char_buffer_string(char_buf, "48", 10, 17);
		alt_up_char_buffer_string(char_buf, "46", 10, 22);

		// Set up plot axes for top and bottom graphs - Put "f/dt(Hz/s" on Y axis and put scale numbers on it
		alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
		alt_up_char_buffer_string(char_buf, "60", 10, 28);
		alt_up_char_buffer_string(char_buf, "30", 10, 30);
		alt_up_char_buffer_string(char_buf, "0", 10, 32);
		alt_up_char_buffer_string(char_buf, "-30", 9, 34);
		alt_up_char_buffer_string(char_buf, "-60", 9, 36);

		// Set up headings for additional information
		alt_up_char_buffer_string(char_buf, "State/Stability: ", 4, 40);
		alt_up_char_buffer_string(char_buf, "Rate Of Change: ", 4, 44);
		alt_up_char_buffer_string(char_buf, "Lower Threshold: ", 4, 46);
		alt_up_char_buffer_string(char_buf, "Most Recent Reaction Times: ", 4, 50);
		alt_up_char_buffer_string(char_buf, "Minimum Reaction Time:", 4, 52);
		alt_up_char_buffer_string(char_buf, "Maximum Reaction Time:", 4, 54);
		alt_up_char_buffer_string(char_buf, "Average Reaction Time:", 4, 56);
		alt_up_char_buffer_string(char_buf, "System Up Time: ", 4, 58);

		Line line_freq, line_roc;
		char* charData;

		while(1){

			//clear old graph to draw new graph
			alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
			alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

			int j;
			for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
				if (((int)(frequencyData[(frequencyIndex+j)%100]) > MIN_FREQ) && ((int)(frequencyData[(frequencyIndex+j+1)%100]) > MIN_FREQ)){
					//Calculate coordinates of the two data points to draw a line in between
					//Frequency plot
					line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
					line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (frequencyData[(frequencyIndex+j)%100] - MIN_FREQ));

					line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
					line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (frequencyData[(frequencyIndex+j+1)%100] - MIN_FREQ));

					//Frequency RoC plot
					line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
					line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * rateOfChangeData[(frequencyIndex+j)%100]);

					line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
					line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * rateOfChangeData[(frequencyIndex+j+1)%100]);

					//Draw
					alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
					alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
				}
			}


			// Store loacal variable to mitigate need for mutex protection 
			xSemaphoreTake(binarySemaphore2, portMAX_DELAY);
			unsigned int currentSystemState = systemState;
			xSemaphoreGive(binarySemaphore2);


			// Source new data, update buffer, draw

			// System state/stability 
			switch(currentSystemState) {
				case 0: // IDLE
					sprintf(charData, "Unknown     "); // spaces are intentional, padding to avoid characters not being overwriten
					break;
				case 1: // STABLE
					sprintf(charData, "Stable      ");
					break;
				case 2: // UNSTABLE
					sprintf(charData, "Unstable    ");
					break;
				case 3: // MAINTENANCE
					sprintf(charData, "Maintenance ");
					break;
			}
			alt_up_char_buffer_string(char_buf, charData, 21, 40);

			// Stability conditions
			sprintf(charData, "%.2f Hz/s", rateOfChangeThreshold);
			alt_up_char_buffer_string(char_buf, charData, 20, 44);

			sprintf(charData, "%f Hz", levelThreshold);
			alt_up_char_buffer_string(char_buf, charData, 21, 46);

			//last five reaction times
			sprintf(charData, "%.4f, %.4f, %.4f, %.4f, %.4f ms", reactionTimes[0], reactionTimes[1], reactionTimes[2], reactionTimes[3], reactionTimes[4]);
			alt_up_char_buffer_string(char_buf, charData, 32, 50);

			// Min, Max, Average reaction times 
			double minTime = minValueArray(reactionTimes, reactionCount);
			sprintf(charData, "%.4f ms", minTime);
			alt_up_char_buffer_string(char_buf, charData, 27, 52);

			double maxTime = maxValueArray(reactionTimes, reactionCount);
			sprintf(charData, "%.4f ms", maxTime);
			alt_up_char_buffer_string(char_buf, charData, 27, 54);

			double averageTime = AverageOfArray(reactionTimes, reactionCount);
			sprintf(charData, "%.4f ms", averageTime);
			alt_up_char_buffer_string(char_buf, charData, 27, 56);

			// System up time
			double upTime = getTime();
			sprintf(charData, "%d sec     ", (int) upTime);
			alt_up_char_buffer_string(char_buf, charData, 20, 58);

			vTaskDelay(VGA_DELAY);

		}
	}
}



/************/
/**  INIT  **/
/************/
// This function simply creates initial data used in the scope of program
int initOSDataStructs(void)
{
	counterSemaphore0 = xSemaphoreCreateCounting( 9999, 0 );
	counterSemaphore1 = xSemaphoreCreateCounting( 9999, 0 );
	counterSemaphore2 = xSemaphoreCreateCounting( 9999, 0 );

	// Initilize/create queues
	frequencyQueue = xQueueCreate(FREQUENCY_DATA_QUEUE_SIZE, sizeof(double));
	keyboardQueue = xQueueCreate(KEYBOARD_QUEUE_SIZE, sizeof(char));


	// Binary Semaphores
	binarySemaphore0 = xSemaphoreCreateBinary(); //
	binarySemaphore1 = xSemaphoreCreateBinary();
	binarySemaphore2 = xSemaphoreCreateBinary();
	binarySemaphore3 = xSemaphoreCreateBinary();
	// Inital give so semaphore is ready to be taken
	xSemaphoreGive(binarySemaphore0);
	xSemaphoreGive(binarySemaphore1);
	xSemaphoreGive(binarySemaphore2);
	xSemaphoreGive(binarySemaphore3);

	frequencyIndex = 0;
	return 0;
}


// This function creates all tasks
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
	switchChanged = 0;
	enterMaintenanceState = 0;
	currentSwitchValue = 0;
	context = 0;

	prevTime500 = 0;
	twoTimes = 0;
	reactionCount = 0;
	timerOverflow = 0;

	// State
	systemState = 0;
	changedState = 0;


	// Thresholds
	levelThreshold = 49;
	rateOfChangeThreshold = 10;
	return 0;
}

int initButtonsPIO(void)
{
	//Enable first four interrupts
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	// Reset the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	//Register the interrupt handler, context is unused so pass in garbage
	alt_irq_register(PUSH_BUTTON_IRQ, context, buttonsISR);

	return 0;
}

int initFrequency(void) {

	//Register the interrupt handler, context is unused so pass in garbage
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, context, frequencyISR);

	return 0; 
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

	IOWR_8DIRECT(PS2_BASE,4,1); //WHAT IS THIS??

	return 0;
}



void initAll(void) {
	initOSDataStructs();
	initCreateTasks();
	initFlags();
	initButtonsPIO();
	initFrequency();
	initKeyboard();
	taskENTER_CRITICAL();
	printf("Initialised all\n");
	taskEXIT_CRITICAL();
	return;
}

/************/
/**  MAIN  **/
/************/

int main(int argc, char* argv[], char* envp[])
{
	initAll();
	alt_timestamp_start();
	vTaskStartScheduler();
	for (;;);
	return 0;
}


