
// this code simulates a standard PWM receiver. 
//The sent signal values are generated in the main function, 
//with possibility to change them from the serial communication.
// The OUTPUTS have this form : 
//	up to 8 signals on 1 port. the Trame frequency is fixed at 50 Hz.
//  each signals sends 1 positive pulse of a duration between 1000 and 2000 microseconds.
//  after the first pin finishes a delay of 45µs is observed before the next pulse.
//  after tha last signal finishes, the first one starts after a delay assuring the 50Hz.
//
// The INPUTS have this form : 
//	up to 6 inputs are possible on the port C on pins A0 to A5.
//  
//
//
//

//-------channels Reading 
// ! do not try to go above 6 channels.
#define INPUT_COUNT 6 // number of channels for reading (max 8)
#define INPUT_STARTMASK 1<<0 // used to offset the channels on the port. 1<<0 means pins 0 to N-1, 1<<2 means pins 2 to N+1
volatile uint8_t bUpdateFlagsShared;
uint8_t bUpdateFlags;
volatile uint16_t unAxisInShared[INPUT_COUNT];
uint16_t unAxisIn[INPUT_COUNT];
uint16_t riseTime[INPUT_COUNT]; // time of the last rising edge.
uint8_t busState = 0; // memory for the state of the current channel.


//--servos writing
#define SERVOS_COUNT 6 // number of channels for servo writing (max 8)
#define SERVOS_PORT PORTD // port handling the servos
#define SERVOS_DDR DDRD // DDR register on the same port 
#define SERVOS_STARTMASK 1<<0 // used to offset the servos on the port. 1<<0 means pins 0 to N-1, 1<<2 means pins 2 to N+1

// default times in ticks(half micros)
#define SERVO_MIN_TICKS 2000
#define SERVO_DEF_TICKS 3000
#define SERVO_MAX_TICKS 4000

// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_AIL_LEFT 1
#define SERVO_ELEV 2
#define SERVO_RUDD 3
#define SERVO_AIL_RIGHT 4
#define SERVO_AUX 5


volatile uint16_t ServoPulseWidths[SERVOS_COUNT];
uint8_t ServoCurrentChannel;
uint8_t servoState;
uint16_t nextTrame;



// The setup() function runs once each time the micro-controller starts
void setup()
{
	Serial.begin(115200);
	Serial.println("----------------------");

	
	
	ServosBegin();
	// Attach the interrupt to the vector of the specified pin, 6 channels can be connected to this Port.
	AttachPortC();
	
}

// Add the main program code into the continuous loop() function
void loop()
{

	////one char per loop, must check if fast enough
	if(Serial.available() > 0)
		char a = (Serial.read());

	// read all the channels values if available
	// check shared update flags to see if any channels have a new signal
	 // turn interrupts off quickly while we take local copies of the shared variables
	// take a local copy of which channels were updated in case we need to use this in the rest of loop
	uint8_t sreg = SREG;
	cli();
	bUpdateFlags = bUpdateFlagsShared;
	bUpdateFlagsShared = 0;
	for (uint8_t i = 0; i < 6; i++)
	{
		if (bUpdateFlags & _BV(i))
		{
			
			unAxisIn[i] = unAxisInShared[i];
			
			// set servo value as copy
			setServoChannel(i, unAxisIn[i]);
		}
	}
	SREG = sreg;

	return;

}

// this function enalble Pinchange interrupt on the portC.
void AttachPortC() {

	for (uint8_t i = 0; i < INPUT_COUNT; i++)
	{
		// set pins as input
		DDRC &= ~(INPUT_STARTMASK<<i);
		//disable pullups
		PORTC &= ~(INPUT_STARTMASK<<i);
		//enable pinchange interrupt on the pin
		PCMSK1 |= INPUT_STARTMASK<<i;
	}

	// enable vector 1
	PCICR |= B00000010;
}

/* 
This is called every time one pin changes on the port C
any pulse on any registered pin is registered as a pulse width in ticks(0.5µs per tick)
*/
ISR(PCINT1_vect) {
	uint16_t timer = TCNT1;
	uint8_t newState = PINC;

	uint8_t rising  = (newState & (~busState));
	uint8_t falling  = (busState & (~newState));

	for (uint8_t i = 0; i < INPUT_COUNT; i++)
	{
		uint8_t bitVal = INPUT_STARTMASK<<i;
		if(rising & bitVal){
			riseTime[i]=timer;
		}
		else if(falling & bitVal){
			unAxisInShared[i] = (timer - riseTime[i]); // rightshift for ticks to micros ( div 2)
			bUpdateFlagsShared |= bitVal;
		}

	}
	busState = newState;
}

void ServosBegin()
{
	// initialize servo registers and pulse width
	for (uint8_t i = 0; i < SERVOS_COUNT; i++)
	{
		ServoPulseWidths[i] = SERVO_DEF_TICKS; // center
		SERVOS_DDR|= (SERVOS_STARTMASK << i); // set as output
		SERVOS_PORT &= ~(SERVOS_STARTMASK << i); //set to low
	}
	servoState=0;
	ServoCurrentChannel = 0;
	
	// set throttle to 0
	ServoPulseWidths[0] = SERVO_MIN_TICKS;

	TCNT1 = 0;              // clear the timer count 

							// Initilialise Timer1
	TCCR1A = 0;             // normal counting mode
	TCCR1B = 2;     // set prescaler of 8 -> 1 tick = 0.5us (see ATmega328 datasheet pgs. 134-135)

					// ENABLE TIMER1 OCR1A INTERRUPT to enabled the first bank (A) of ten servos
	TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
	TIMSK1 |= _BV(OCIE1A); // enable the output compare interrupt

	OCR1A = TCNT1 + 4000; // Start in two milli seconds
}

void setServoChannel(uint8_t nChannel, uint16_t unTicks)
{
	// dont allow a write to a non existent channel
	if (nChannel >= SERVOS_COUNT)
		return;


	if (nChannel < SERVOS_COUNT - 1)
		unTicks = constrain(unTicks, SERVO_MIN_TICKS, SERVO_MAX_TICKS);

	// disable interrupts while we update the multi byte value output value
	uint8_t sreg = SREG;
	cli();

	ServoPulseWidths[nChannel] = unTicks;

	// enable interrupts
	SREG = sreg;
}

// interrupt for the timer1 compare register used to toggle the output channels.
ISR(TIMER1_COMPA_vect)
{
	uint16_t timer = TCNT1;

	if (servoState){
		// servo pin is high, set it low
		SERVOS_PORT &= ~(SERVOS_STARTMASK << ServoCurrentChannel);
		ServoCurrentChannel+=1;
		if (ServoCurrentChannel < SERVOS_COUNT){
			OCR1A = timer+ 90; // 90 ticks is 45µs delay between channels.
		}
		else{
			// last servo finished writing, wait for next trame
			OCR1A = nextTrame;
			ServoCurrentChannel=0;
		}
		servoState=0;
	}
	else{
		//write pin high
		SERVOS_PORT |= (SERVOS_STARTMASK << ServoCurrentChannel);
		if (ServoCurrentChannel == 0){
			// we are starting the trame.
			nextTrame = timer+40000; // each trame is 20 ms long
		}
		// set the duration of the output pulse
		OCR1A = timer + ServoPulseWidths[ServoCurrentChannel];
		servoState=1;
	}
}

ISR(__vector_default) {}
