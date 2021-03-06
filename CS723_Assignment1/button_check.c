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
#include "altera_avalon_pio_regs.h" 	// to use PIO functions
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types

/************/
/*  ISRs     */
/************/
int button_click = 0;
void buttonsISR(void* context,  alt_u32 id) {

	//Read edge capture register's value
	button_click = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);
	printf("bf: %d\n", button_click);
	if (button_click == 1) { // Is this the right button press?
		//enterMaintenanceState = 1;
	} else {
		//enterMaintenanceState = 0;
	}

	//reset edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

	// Give the semaphore
	//xSemaphoreGive(counterSemaphore1);
}

/************/
/*  METHODS */
/************/


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

/************/
/**  MAIN  **/
/************/
//int main(int argc, char* argv[], char* envp[])
//{
//	printf("RUNNING BUTTON_CHECK.C");
//
//	initButtonsPIO();
//	vTaskStartScheduler();
//	while(1){}
//	return 0;
//}
//

