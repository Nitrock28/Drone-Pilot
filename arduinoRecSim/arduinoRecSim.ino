
//main logic

//-------channels Reading 
// holds the update flags bit per bit, with each axis represented by one bit in the byte
volatile uint8_t bUpdateFlagsShared;
uint8_t bUpdateFlags;
volatile unsigned long lastTimeAxisRead;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unAxisInShared[6];
uint16_t unAxisIn[6];

//used in interrupt : 
uint8_t channel = 10; // current channel number.
bool up = false; // memory for the state of the current channel.
uint8_t chan0 = 0; // memory for the state of the channel 0
uint16_t riseTime = 0; // time of the last rising edge.


//--servos writing
// Change to set the number of servos/ESCs + if a delay is needed between two frames to reduce the writing frequency 
#define SERVOS_COUNT 7 
#define SERVOS_PORT PORTD
#define SERVOS_DDR DDRD
#define SERVOS_STARTMASK 4

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
#define SERVO_FRAME_SPACE 6 // always the number of servos (equivalent to a pulse on a non existent pin to make a delay)


volatile uint16_t ServoPulseWidths[SERVOS_COUNT];
uint8_t ServoCurrentChannel;



// The setup() function runs once each time the micro-controller starts
void setup()
{
	Serial.begin(115200);
	Serial.println("----------------------");

	
	//no need to attach servos, control the port, number of servos and start pin in the .h file.
	ServosBegin();
	// set the delay between the last servo and the first one.
	setServoChannel(6,100);//6*1500+(100/2)= 
	// Attach the interrupt to the vector of the specified pin, 6 channels can be connected to this Port.
	AttachPortC();
	
}

// Add the main program code into the continuous loop() function
void loop()
{

	////one char per loop, must check if fast enough
	if(Serial.available() > 0)
		gps.encode(Serial.read());

	// read all the channels values if available
	// check shared update flags to see if any channels have a new signal
	 // turn interrupts off quickly while we take local copies of the shared variables
	// take a local copy of which channels were updated in case we need to use this in the rest of loop
	uint8_t sreg = SREG;
	cli();
	unsigned long timeSinceLastRead = millis() - lastTimeAxisRead;
	bUpdateFlags = bUpdateFlagsShared;
	bUpdateFlagsShared = 0;
	SREG = sreg;


	
	if (timeSinceLastRead > 100)
		controlMode = 100;
	else
		controlMode = 0;


	if (controlMode == 0 && bUpdateFlags) {
		// only copy when the flags tells us we can.
		//disable interrupts while writing to all channels.
		sreg = SREG;
		cli();
		for (uint8_t i = 0; i < 6; i++)
		{
			if (bUpdateFlags & _BV(i))
			{
				
				unAxisIn[i] = unAxisInShared[i];
				
				// set servo value as copy
				setServoChannel(i, unAxisIn[i]);
			}
		}
		//re enable interrupts.
		SREG = sreg;
	}
	
	// 6th axis at 4000 activates the auto mode.
	if (controlMode == 0 && unAxisIn[5] > 3500) {
		controlMode = 100;
	}

	if (controlMode == 100) {
		/*
		This is the automatic mode, at this point, we can query data from the GPs and the MPU. 
		the craft should be brought back to a normal attitude if it is not already the case, 
		when in normal mode, we can get the GPS data to acquire the required and current heading.
		the four axis are controlled in this manner in normal flight conditons:
			-Throtthle : fixed in the way point ; varies with the current altitude; 100% if lower than; 0 if higher than + 100m
			-Rudder : 0 when heading is aligned to the target distance progressive in a band of 10° then full deflection outside the range.
			-Elevator : PID with EMU to be at 15° from horizontal under the target altitude, degressive between target and target+50m, fixed at 0° if higher
			-Ailerons : PID with EMU to maintain a roll value of 0 when in the 10° band, then progressively increase outside this zone.

		*/

		if (gps.location.isUpdated()) {
			distanceToTarget =
				TinyGPSPlus::distanceBetween(
					gps.location.lat(),
					gps.location.lng(),
					WPLat[WPIndex],
					WPLon[WPIndex]);
			courseToTarget =
				TinyGPSPlus::courseTo(
					gps.location.lat(),
					gps.location.lng(),
					WPLat[WPIndex],
					WPLon[WPIndex])*100.0;

			if (distanceToTarget < 20) {
				// target reached, update on next fix.
				//TODO check for end of targets.
				WPIndex++;
			}
		}
		
		// check if values are availables
		myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
		myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
		myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
		myIMU.updateTime();
		MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
			myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
			myIMU.mx, myIMU.mz, myIMU.deltat);


		// yaw pitch and roll are givien in tait bryan order.
		YAW = *getYPR();//Always the direction of the nose of the aircraft, limited by the gimball lock at pitch close to + -pi / 2
		PITCH = *(getYPR()+1); //always the direction of the nose in the vertical direction, regardless of the fact that the plane can be facing down.
		ROLL = *(getYPR()+2);//0 always mean that the plane is upside up, limited by the gimball lock at pitch close to + -pi / 2

		double throttle = 0.0; // value between 0 and 1
		double rudder=0.0; // value between -1 and 1
		double elev=0.0; //value between -1 and 1 also the output of the PID
		double ail=0.0; //value between -1 and 1 also the output of the PID

		if (PITCH > 60.0 || PITCH < -60.0 || ROLL>40.0 || ROLL <-40.0)
		{
			// non normal flight conditions, try to get back to level
			// mix rudder and elevator to get back flat in any roll condition ; set roll motion to come back to 0.
			//TODO disable PID to have a clean reactivation
			//TODO check directions of angles and affectors
			
			throttle = constrain(PITCH*0.015+0.1, 0.1, 1.0); // 0.1 when pitch is negative, progressive to 1 when pitch at 60° and above.
			rudder = constrain(  -PITCH*0.0166667 *sin(ROLL),-1.0,1.0); // -1 when turned to the right, 0 at center or upside down, 1 to the left at 60° pitch, degressive towards 0
			elev = constrain(-PITCH*0.0166667 *cos(ROLL),-1.0,1.0); // 0 when turned, -1 at center, 1 upside down at 60° pitch, degressive towards 0
			ail = constrain(ROLL*0.025, -1.0, 1.0); // 1 at ROll 40°; -1 at -40°

			
		}
		else {
			//we are in the standard conditions to manoever the aircraft.
			//TODO check isValid also
			if (gps.location.age() > 3000 || gps.course.age() > 3000 || gps.altitude.age() > 3000) {

				//gps data not available. turning in circles.
				//TODO check directions of angles and effectors
				throttle = 0.5;
				rudder = 0.5;
				Pit_SetPoint = 2.0;
				Rol_SetPoint = 10.0;
			}
			else {
				// aiming for the next target based on heading and target direction.
				//TODO discouple the GPS data reading from the EMU, PID setpoints have to be set once par GPS reading; PID compute to be called every time the EMU is read.
				
				throttle = gps.altitude.meters()>(100.0+WPAlt[WPIndex])? 0.0: gps.altitude.meters()< WPAlt[WPIndex]? 1.0 : WPThrott[WPIndex];
				
				rudder = constrain((gps.course.value() - courseToTarget)*0.001, -1.0, 1.0); //0 when heading is aligned to the target distance progressive in a band of 10° then full deflection outside the range.
				Pit_SetPoint = constrain((WPAlt[WPIndex] - gps.altitude.meters() )*0.3+15.0, 0.0, 15.0); //PID with EMU to be at 15° from horizontal under the target altitude, degressive between target and target + 50m, fixed at 0° if higher
				Rol_SetPoint = Pit_SetPoint > 10 ? 0.0 : constrain((gps.course.value() - courseToTarget)*0.005, -15.0, 15.0); // PID with EMU to maintain a roll value of 0 when pitch is less more than 10 °, else progressinve in the +30° -30° range.
			}

			Rol_PID.Compute();
			Pit_PID.Compute();
			ail = Rol_Output;
			elev = Pit_Output;

		}

		//from -1;+1 range to 2000 - 4000
		uint16_t microRudder = 1500 + 500 * (int)rudder;
		int microElev = 1500 + 500 * (int)elev;
		int microAil1 = 1500 + 500 * (int)ail;
		int microAil2 = microAil1; // both servos ar in the same direction to generate roll
		
		//throttle
		setServoChannel(SERVO_THROTTLE, throttle*2000.0 + 2000.0);

		setServoChannel(SERVO_AIL_LEFT, ail*1000.0 + 3000.0);
		setServoChannel(SERVO_AIL_RIGHT, -ail*1000.0 + 3000.0);

		setServoChannel(SERVO_ELEV, elev*1000.0 + 3000.0);

		setServoChannel(SERVO_RUDD, rudder*1000.0 + 3000.0);
	}


	return;

}

// this function enalble Pinchange interrupt on the 6 first pins of portC.
void AttachPortC() {
	// set pins as input
	DDRC &= ~B00111111;
	//disable pullups
	PORTC &= ~B00111111;
	//enable 6 pins on the same vector C = mask 1
	PCMSK1 |= B00111111;
	// enable vector
	PCICR |= B00000010;
}

/* 
This is called every time one pin changes on the port C
The channels are pulsed one by one in order, with a 45 µs delay between each pulse
we wait for the first pin to pulse and then increment the channel number.
There is no error detection, the pins have to be pulsed in order. the cycle is reset when the first pin pulses.
*/
ISR(PCINT1_vect) {
	uint16_t timer = TCNT1;
	uint8_t newChan0 = PINC & 1;

	if (newChan0 & (~chan0)) {
		//rising edge of first channel.
		channel = 0;
		up = false;
		lastTimeAxisRead = millis(); // mark time of last read. to detect when we lose signal.
	}
	if (channel <6)
	{
		// waiting for a rising edge on pin
		if (up) {
			//falling edge, write value to channel.
			
			unAxisInShared[channel] = (timer - riseTime); // rightshift for ticks to micros ( div 2)
			bUpdateFlagsShared |= _BV(channel);
			channel++;
		}
		else
			riseTime = timer;
		up = !up;
	}
	chan0 = newChan0;
}

void ServosBegin()
{
	// initialize servo width
	// throttle
	ServoPulseWidths[0] = SERVO_MIN_TICKS;

	// other channels are centered
	ServoCurrentChannel = 1;
	while (ServoCurrentChannel < SERVOS_COUNT)
	{
		ServoPulseWidths[ServoCurrentChannel] = SERVO_DEF_TICKS;

		ServoCurrentChannel++;
	}
	// with ServoCurrentChannel at SERVOS_COUNT, the first servo to trigger is the channel 0

	//TODO write to port in the begin function.
	// set pins as outputs
	SERVOS_DDR |= B11111100;
	//write low
	SERVOS_PORT &= ~B11111100;

	TCNT1 = 0;              // clear the timer count 

							// Initilialise Timer1
	TCCR1A = 0;             // normal counting mode
	TCCR1B = 2;     // set prescaler of 8 = 1 tick = 0.5us (see ATmega328 datasheet pgs. 134-135)

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
	// we check if we just passed the delay, then we can reset the pin number
	if (ServoCurrentChannel >= SERVOS_COUNT)
	{
		// reset our current servo/output channel to 0
		ServoCurrentChannel = 0;

		// no pin to set low, we are in the delay
	}
	else
	{
		// set previous pin low
		SERVOS_PORT ^= (SERVOS_STARTMASK << (ServoCurrentChannel - 1));
	}

	// ----setting the pin High only on real channels
	if (ServoCurrentChannel <(SERVOS_COUNT - 1))
		SERVOS_PORT |= (SERVOS_STARTMASK << ServoCurrentChannel);



	// set the duration of the output pulse
	OCR1A = TCNT1 + ServoPulseWidths[ServoCurrentChannel];

	// done with this channel so move on.
	ServoCurrentChannel++;
}

ISR(__vector_default) {}