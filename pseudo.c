================
Global Variables
================
// Parameters 
frequencyChangeThreshold = 0;
frequencyLowerThreshold = 0;
timePeriod = 0;

// Flags and general variables
ButtonState = 0;//1 is maintainence state, 0 relay states
TOF = 0;//500ms timer overflow 
systemStatus = 0; // 0 is idle, 1 stable, 2 unstable, 3 maintenance 
//state = 0;

// Soft outputs
greenLEDValue = 0b0;
redLEDValue = 0b0;

// Queues
measurementsQ[5] = [0, 0, 0, 0, 0];
frequencyQ[10] = {0};

==============
ISRS 
==============
FreqDataISR {
	//possibly calculate freq from sampleCount here
	//store value in queue
	addToQueue(input, frequencyQ);
}

buttonISR {
	ButtonState = !ButtonState;
}

Timer500ISR {
	TOF = 1;
}

Timer200ISR {
	//print system has failed!
}

===========
Tasks
===========

LEDOutput {
	//read value of 200ms timer, and stop/reset
	timerValue = timer200.getTime();
	timer200.stop();
	timer200.reset();
	//if timer value not zero, store value in measurements queue
	if (timerValue != 0) {
		semephoreMeasurements.lock();
		addToQueue(timerValue, measurements);
		semephoreMeasurements.unlock();
	}
	//output LED base value
	semephoreLEDs.lock();
	POI_ALTERA_LEDS(GreenLEDBase, greenLEDValue);
	POI_ALTERA_LEDS(RedLEDBase,redLEDValue);
	semephoreLEDs.unlock();
}

VGAOutput {
	semephoreMeasurements.lock();
	//output five measurements 
	semephoreMeasurements.unlock();
	//output frequency graph	
}

NextStateLogic {
	nextSystemStatus = systemStatus;


	// 0 0 = Stable
	// 1 0 = UnStable
	// 0 1 = UnStable
	// 1 1 = UnStable
	
	//check button status 
	if (buttonState = 1) {
		nextSystemStatus = 3; // 3 is the maintenance state 
	
	//check rate of change condition check threshold condition
	} else {

		frequencyUnstable = aboveRateOfFrequency() || belowThresholdFrequency();
	
		if ( frequencyUnstable ) {
			nextSystemStatus = 2; // 2 is unstable state
		
		} else if ( !frequencyUnstable ) {
			nextSystemStatus = 1; // 1 is stable state		
		}
	}
		
	// If nextState != prevState then restart timers, 
	if (systemStatus != nextSystemStatus || TOF) {
		//give semaphore
		if (nextSystemStatus != 3) {
			// Restart timer
			timer200.reset();
			timer200.start();
		}
	}
	
	semephoreSystemStatus.lock();
	systemStatus = nextSystemStatus;
	semephoreSystemStatus.unlock();
}



// enFSM is affected by:
//		- TO  				
//		- ButtonPress  
//		- Stability change

FSM {
	
	if (enFSM) { // enFSM gets changed by TO
		switch(systemStatus) {
			case Unstable:
				// Restart timer
				timer500.stop();
				timer500.reset();
				timer500.start();
			
				checkSwitches();
				checkLED();
				checkPriority();
				updateLEDs(); // Soft Outputs
			case Stable:
			// Restart timer
				timer500.stop();
				timer500.reset();
				timer500.start();
			
				checkSwitches();
				checkLED();
				checkPriority();
				updateLEDs(); // Soft Outputs
			case Maintenance:
				checkSwitches();
				updateLEDs(); // Soft Outputs
		}
		
		enFSM = 0;
		
	}
}

===========
Methods
===========

boolean aboveRateOfFrequency() {
	// Check most recent value of frequency queue, compare it previous and find gradient
	int length = frequencyQ.length();
	result = (frequencyQ[length] - frequencyQ[length - 1])/timePeriod;
	return (result > frequencyChangeThreshold); // 1 is Unstable 
}


boolean belowThresholdFrequency() {
	// Check most recent value of frequency queue, compare it to threshold
	int length = frequencyQ.length();
	return (frequencyQ[length] < frequencyLowerThreshold); // 1 is Unstable
}



