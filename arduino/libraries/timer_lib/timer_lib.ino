#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define SIZEOF_ARRAY(x)  (sizeof (x) / sizeof (x[0]))

#endif

#define UNKNOWN_BOARD 0
#define ARDUINO_UNO 1
#define ARDUINO_MEGA 2

#if defined(__AVR_ATmega328P__)
   #define MICRO_PROCESSOR_BOARD ARDUINO_UNO
#elif defined(__AVR_ATmega2560__)
   #define MICRO_PROCESSOR_BOARD ARDUINO_MEGA
#else
   #define MICRO_PROCESSOR_BOARD UNKNOWN_BOARD
#endif

// volatile so that the interrupt actually changes it. COPY THIS LOCALLY WITHIN FUNCTIONS.
volatile unsigned long totalms = 0;

volatile unsigned long start_of_isr, funcs_start, funcs_end, timed_end, funcs_period;


// Data structure (doubly linked list) for storing timed function parameters.
typedef struct timed_function {
    void (*user_function)(uint8_t);
    uint8_t function_argument;
    // time the timed function was started.
    unsigned long created_time_us;
    // time to launch the function relative to creation time in microseconds.
    unsigned long launch_time_offset_us;
    // period in microseconds when the function is periodic.
    unsigned long launch_period_us;
    // time the function was last run in microseconds.
    unsigned long launched_time_us;
    // Status flag for the function (running/paused).
    boolean running;
    // Flag indicating this function is done and the memory can be freed.
    boolean stop;
    // Flag indicating whether this function was already deleted.
    boolean deleted;
    // Flag indicating this timed function runs only once.
    boolean single_run;
    struct timed_function *next;
    struct timed_function *previous;
} timed_function;

timed_function *timed_functions_running = NULL;
timed_function *timed_functions_paused = NULL;

// Create a new timed function.
timed_function *create_timed_function(void (*user_function)(uint8_t), \
                                     uint8_t function_argument, \
                                     unsigned long time_offset_us, \
                                     unsigned long period_us, \
                                     boolean single_run) {

   timed_function * new_timed_function;
   new_timed_function = (timed_function *)malloc(sizeof (timed_function));
   new_timed_function->user_function = user_function;
   new_timed_function->created_time_us = micros();
   new_timed_function->launch_time_offset_us = time_offset_us;
   new_timed_function->launch_period_us = period_us;
   new_timed_function->launched_time_us = 0L;
   new_timed_function->running = false;
   new_timed_function->stop = false;
   new_timed_function->deleted = false;
   new_timed_function->single_run = single_run;
   new_timed_function->next = NULL;
   new_timed_function->previous = NULL;

   return new_timed_function;
}

void delete_timed_function(timed_function *user_function) {

    if (!user_function->deleted) {
        cli();
        // If already in a list, reassign previous and next's pointers.
        if (timed_functions_running == user_function)
            timed_functions_running = user_function->next;
        else if (timed_functions_paused == user_function)
            timed_functions_paused = user_function->next;
        if (user_function->next != NULL)
            user_function->next->previous = user_function->previous;
        if (user_function->previous != NULL)
            user_function->previous->next = user_function->next;

        user_function->next = NULL;
        user_function->previous = NULL;
        user_function->running = false;
        user_function->stop = true;
        user_function->deleted = true;

        // Free memory. Warning: This may fragment heap memory..not sure.
        //free(user_function);
        sei();
    }
}

void run_timed_function(timed_function *user_function) {

    if (user_function->stop)
        delete_timed_function(user_function);
    else if (!user_function->running) {

        cli();
        user_function->running = true;

        // If already in a list, reassign previous and next's pointers.
        if (user_function->next != NULL)
            user_function->next->previous = user_function->previous;
        if (user_function->previous != NULL)
            user_function->previous->next = user_function->next;
        if (timed_functions_paused == user_function)
            timed_functions_paused = user_function->next;

        // insert the timed function at the head of the running list.
        if (timed_functions_running == NULL) {
            timed_functions_running = user_function;
            user_function->previous = NULL;
            user_function->next = NULL;
        }
        else {
            user_function->next = timed_functions_running;
            user_function->previous = NULL;
            timed_functions_running->previous = user_function;
            timed_functions_running = user_function;
        }
        sei();
    }
}

void pause_timed_function(timed_function *user_function) {

    if (user_function->stop)
        delete_timed_function(user_function);
    else if (user_function->running) {

        cli();
        user_function->running = false;

        // If already in a list, reassign previous and next's pointers.
        if (timed_functions_running == user_function)
            timed_functions_running = user_function->next;
        if (user_function->next != NULL)
            user_function->next->previous = user_function->previous;
        if (user_function->previous != NULL)
            user_function->previous->next = user_function->next;

        // insert the timed function at the head of the paused list.
        if (timed_functions_paused == NULL) {
            timed_functions_paused = user_function;
            user_function->previous = NULL;
            user_function->next = NULL;
        }
        else {
            user_function->next = timed_functions_paused;
            user_function->previous = NULL;
            timed_functions_paused->previous = user_function;
            timed_functions_paused = user_function;
        }
        sei();
    }
}

template <size_t NUMBER_VALUES>
unsigned long find_prescaler(unsigned long (&prescaler_values)[NUMBER_VALUES],\
                             unsigned long frequency, \
                             unsigned long max_counter_value) {

    /*for(int element = 0; element<NUMBER_VALUES; element++) {
        Serial.print("f-value: ");
        Serial.print(prescaler_values[element]);
        Serial.print("\tf-address: ");
        Serial.println((unsigned int)&prescaler_values[element]);

    }*/

    unsigned int element;
    unsigned long maximum_prescaler, prescaler;

    // use clock frequency F_CPU , to calculate prescaler.
    // this is the max. multiplier we can use.
    maximum_prescaler = F_CPU/frequency;

    // pick the smallest prescaler that can be used to setup the timer for the
    // desired frequency.
    prescaler = 0;  // 0 indicates that no prescaler was found.
    // Computed register counter value.
    unsigned long largest_counter_value = 0;
    unsigned long counter_value = 0;

    for(element=0; element<NUMBER_VALUES; element++) {
        counter_value = F_CPU/(prescaler_values[element] * frequency);
        if ((counter_value < 2 ) || (counter_value > max_counter_value))
            continue;
        else if(counter_value > largest_counter_value) {
            //Serial.print("used prescaler: ");
            //Serial.println(prescaler_values[element]);
            //Serial.print("counter value: ");
            //Serial.println(counter_value);
            // the register is 0 indexed.
            largest_counter_value = counter_value - 1;
            prescaler = prescaler_values[element];
        }
    }
    return prescaler;
}


// Setup a timer of given frequency (in Hz)
int setup_timer(unsigned long frequency, uint8_t timer_number) {

	unsigned long used_prescaler, compare_match_value;

	/*
	Definitions: where n = timer number {0, 1, ...}
	TCNTn = Timer n counter value. 16bit.
	OCRnA, OCRnB = Output compare registers A, B of timer n. 16bit.
	TCCRnA, TCCRnB, TCCRnC = Timer n control register A,B,C. 8bit.
	TIMSKn = Timer n interrupt mask register.


	TCCRxB = Timer counter
	*/

	// (timer speed in Hz) = (Arduino clock speed) / prescaler

	unsigned long MAX_COUNTER_VALUE;

	// Setup given timer to generate interrupts on compare mode.
    if((MICRO_PROCESSOR_BOARD == ARDUINO_UNO)|| (MICRO_PROCESSOR_BOARD == ARDUINO_MEGA)){

        switch(timer_number) {
            case 1: {
                // SETUP TIMER 1 to generate interrupt TIMER1_COMPA_vector.

                // Timer 1 in Arduino UNO (ATmega328P processor) is 16bit.
                MAX_COUNTER_VALUE = 65535;  // indexed from 0

                unsigned long prescaler_values[5] = {1, 8, 64, 256, 1024};
                //unsigned long (* pointer_to_prescaler_values)[5];
                //pointer_to_prescaler_values = &prescaler_values;
                //Serial.println(sizeof(*pointer_to_prescaler_values));

                // pick a prescaler and validate it.
                used_prescaler = find_prescaler(prescaler_values, \
                                                frequency, \
                                                MAX_COUNTER_VALUE);

                if (used_prescaler == 0)
                    return 0;
                // Calculate register counter.
                compare_match_value = F_CPU/(used_prescaler * frequency) - 1;

                //char* message;
                //sprintf(message, "prescaler: %i", used_prescaler);
                //Serial.println(message);

                /*Serial.print("used prescaler: ");
                Serial.println(used_prescaler);

                Serial.print("Compare match value: ");
                Serial.println(compare_match_value);
                delay(500);*/

                // Disable interrupts.
                cli();
                // Reset compare match registers.
                TCCR1A = 0;
                TCCR1B = 0;
                //initialize counter value to 0
                TCNT1  = 0;

                // turn on CTC mode
                TCCR1B |= (1 << WGM12);

                // Set appropriate register flags for the calculated prescaler.
                switch(used_prescaler) {
                    case 1:
                        // Set CS10 bit for prescaler = 1.
                        TCCR1B |= (1 << CS10);
                        break;
                    case 8:
                        TCCR1B |= (1 << CS11);
                        break;
                    case 64:
                        // alternative way to set the register bits:
                        // TCCR1B |= (1 << CS10) | (1 << CS11);
                        sbi(TCCR1B, CS10);
                        sbi(TCCR1B, CS11);
                        break;
                    case 256:
                        TCCR1B |= (1 << CS12);
                        break;
                    case 1024:
                        // Set CS10 and CS12 bits for 1024 prescaler.
                        TCCR1B |= (1 << CS12) | (1 << CS10);
                        break;
                }
                // set compare match register.
                OCR1A = compare_match_value;
                // enable timer compare interrupt
                TIMSK1 |= (1 << OCIE1A);
                sei(); //re-enable interrupts
                break;
            }

            case 0: {

                // this timer is used by the millis() function.
                // TODO: add code that handles the time count?

                MAX_COUNTER_VALUE = 255;
                unsigned long prescaler_values[5] = {1, 8, 64, 256, 1024};

                // pick a prescaler and validate it.
                used_prescaler = find_prescaler(prescaler_values, \
                                                frequency, \
                                                MAX_COUNTER_VALUE);

                //Serial.print("used prescaler: ");
                //Serial.println(used_prescaler);

                if (used_prescaler == 0)
                   return 0;



                // Calculate register counter.
                compare_match_value = F_CPU/(used_prescaler * frequency) - 1;
                // Serial.print("Compare match value: ");
                // Serial.println(compare_match_value);

                // Disable interrupts.
                cli();
                // Reset compare match registers.
                TCCR0A = 0;
                TCCR0B = 0;
                //initialize counter value to 0
                TCNT0  = 0;

                // turn on CTC mode
                TCCR0A |= (1 << WGM01);

                // Set appropriate register flags for the calculated prescaler.
                switch(used_prescaler) {
                    case 1:
                        // Set CS10 bit for prescaler = 1.
                        TCCR0B |= (1 << CS00);
                        break;
                    case 8:
                        TCCR0B |= (1 << CS01);
                        break;
                    case 64:
                        TCCR0B |= ((1 << CS01) | (1 << CS00));
                        break;
                    case 256:
                        TCCR0B |= (1 << CS02);
                        break;
                    case 1024:
                        // Set CS10 and CS12 bits for 1024 prescaler.
                        TCCR0B |= (1 << CS02) | (1 << CS00);
                        break;
                }
                // set compare match register.
                OCR0A = compare_match_value;
                // enable timer compare interrupt
                TIMSK0 |= (1 << OCIE0A);
                sei(); //re-enable interrupts*/
                break;
            }
        }
    }

	return used_prescaler;

}

/*
SIGNAL(TIMER5_COMPA_vect) { // this is the actual interrupt routine that occurs every 1 ms.
	int i;
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = totalms;

	m++; // this timer vector ticks once per ms.
	totalms = m; //rewrite to the volatile variable.

	// run user timer routines as required.
	for (i = 0; i < MAX_FUNC; i++) {
		if ((funcFree[i] ==0) && (funcEnable[i] != 0) && (funcTime[i] <= m)) {
			funcEnable[i] = 0;
			(*p_func[i])(funcArg[i]);
		}
	}
}*/

/* TODO: 3. Flag for timed functions execution. Check in ISR and call it there?
         4. Check: exiting an ISR does interrupt reset?
         5. Circular buffer (struct?) of time+values. Class instead?
         6. Sampled on timer.
         7. Serial I/O.
*/

ISR(TIMER1_COMPA_vect) {

    volatile unsigned long elapsed_time_us, time_us;

    // for testing
    start_of_isr = micros();


    // Run the timed functions.
    if (timed_functions_running != NULL) {
       timed_function *next = timed_functions_running;
       // traverse the linked list.
       while (next != NULL) {
           time_us = micros();
           if (next->stop) {
               timed_function *new_next = next->next;
               delete_timed_function(next);
               next = new_next;
           }
           else if(!next->running) {
               timed_function *new_next = next->next;
               // reset flag to allow pausing. It will be set again in the
               // pausing function.
               next->running = true;
               pause_timed_function(next);
               next = new_next;
           }
           else if (next->launched_time_us == 0) {
               elapsed_time_us = time_us - next->created_time_us;
               if(elapsed_time_us >= next->launch_time_offset_us) {
                   // launch the function.
                   next->launched_time_us = time_us;
                   (*(next->user_function))(next->function_argument);
                   // for debugging.
                   funcs_period = next->launch_period_us;
               }
               next = next->next;
           }
           else {
               elapsed_time_us = time_us - next->launched_time_us;
               if(elapsed_time_us >= next->launch_period_us) {
                   next->launched_time_us = time_us;
                   (*(next->user_function))(next->function_argument);
                   // for debugging.
                   funcs_period = next->launch_period_us;
               }
               next = next->next;
           }
       }
    }
    timed_end = micros();
}

ISR(TIMER0_COMPA_vect) {
   // for testing
   return;
}
