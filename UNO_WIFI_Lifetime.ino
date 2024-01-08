/*
This Script will log and write times when we saw a start signal on pin D3
and a stop signal on pin SDA. It will also print the time-difference of the
two signals.
It connects to the GPS on pins D0 and D1 (RX and TX)
The PPS signal must enter at pin D4
Pin restrictons arise from internal connections to timers on the board. 

We use the only two timers (TCB0 and TCB1) that can record external events to 
60ns precision. Timer TCB1 is used for the start pulses and TCB0 is used for
the stop pulses. TCB2 keeps the time between PPS events. The PPS signal we 
only react to every 4 microseconds, as it uses a regular ISR() and not the 
event system with the timers. (Where we also only react every 4 microseconds,
but know the time when the flag was raised). 

Each time we get a PPS signal, we recompute the current clockspeed. 
This is not totally acurate, as we record the pulse with 
4000ns/62.5ns = 64 clock-cycles precision.
However the variance in the number of clock-cycles in a second is larger than this,
so we still improve over not taking this into account at all. 
*/


#include <Adafruit_GPS.h>

#define mySerial Serial1 //TX,RX
Adafruit_GPS GPS(&mySerial);

//To track state:
bool startup=true; 
bool logging=true;
bool Ready=false;

//Pinning:
// Pin D3 as input for pulses (start)
// Pin SDA as input for pulses (stop)
// Pin D4 as PPS pin (PC6)

//variables
volatile unsigned long nanosecond;
volatile unsigned long second;
int minute;
int hour;

volatile unsigned long cyclesInSecond; //clockcycles we had last second

volatile unsigned long overflowCounter = 0; //Counts overflows in clock (every 4 ms this hapens)
volatile float clkSpeed=62.5;//clockspeed in ns

volatile unsigned long EventNanosec; //record time of event
volatile unsigned long EventSec;

volatile bool Start = false; //For the differences
volatile bool Stop = false;
volatile unsigned long CounterAtStart=0;
volatile unsigned long CounterAtStop=0;

unsigned long Difference=0;
unsigned long maxTimeDiff=25000; //in ns
unsigned long minTimeDiff=0; //in ns


void setup() {
  
  Serial.begin(38400);
  GPS.begin(9600);//Start connection to GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //Tell GPS we only need basic data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  Serial.println("Waiting for GPS Fix");

  // set up clock counter:
  noInterrupts();
  TCB2.CTRLB = 0; // Use timer compare mode
  TCB2.EVCTRL = 0; //Periodic interrupt mode
  TCB2.CCMP = 65535; // set TOP (max is 2^16)
  TCB2.INTCTRL = 1; // Enable the interrupt
  TCB2.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Use Timer as clock, enable timer


  //Set up interrupt from signal 1 (start):
  PORTF.DIRCLR = PIN5_bm; //pin PF5
  PORTF.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_RISING_gc;
  PORTMUX.TCBROUTEA = 2; //use alternative pin PF5 = D3
  
  //Connect the event-system:
  EVSYS.CHANNEL5 = 0x4D; //set channel 5 to use PF5 to get events
  EVSYS.USERTCB1 = 6; //user is TCB1, connect to channel 5=6-1
  
  TCB1.CTRLB = 2; // Use COE mode
  TCB1.EVCTRL = 1; //enable input capture event
  TCB1.INTCTRL = 1; // Enable the interrupt on capture event
  TCB1.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Use Timer as clock, enable timer

  
  //Set up interrupt from signal 2 (stop):
  PORTA.DIRCLR = PIN2_bm; //pin PA2
  PORTA.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_RISING_gc;
  EVSYS.CHANNEL0 = 0x42; //set channel 1 to use PA2 to get events
  EVSYS.USERTCB0 = 1; //user is TCB0, connect to channel 1=2-1

  //Set up clock for pin 2 - Has to be TCB0 to get right pin available
  TCB0.CTRLB = 2; // Use COE mode
  TCB0.EVCTRL = 1; //enable input capture event
  TCB0.INTCTRL = 1; // Enable the interrupt on capture event
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // Use Timer as clock, enable timer


  //Set up Interrupt from PPS:
  PORTC.DIRCLR = PIN6_bm; //pin PC6
  PORTC.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_RISING_gc;
  
  interrupts();
  
}

void loop() {
  //main loop
  if(startup) {
    Startup();
  } else if (logging) {
    Normal();
  } else {
    //Do nothing
  }
  
}

void Startup(){
  //Run this until we have time from GPS  
  
  char c = GPS.read(); //read GPS module
  if (GPS.newNMEAreceived()) {//If we received something, parse it
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (((GPS.day != 0) and (Ready==false)) and (GPS.fix==true)){ //The GPS sent us a time, so get ready to start!
    nanosecond=0;
    minute=GPS.minute; //save time we got
    second=GPS.seconds;
    hour=GPS.hour;

    Serial.print("GPS Fix obtained at;");
    printTime(second,nanosecond,true);
    Serial.println(";");
    Ready=true; // Go into normal mode when next second ticks
  }
  
}

void Normal(){
  //Normal operating loop
  
  CheckPulse(); //Check if we need to print about an event
  
  StopLoggingCheck(); //Check if we should stop

  delay(10);
}


void CheckPulse(){
  //Check if we had an event 
  if ((Start==true) and (Stop ==true)){
    Difference=(CounterAtStop-CounterAtStart)*clkSpeed;
    if (Difference < maxTimeDiff and Difference > minTimeDiff){
      Serial.print("Lifetime;");
      printTime(EventSec,EventNanosec,false);
      if (Difference<1000000){Serial.print(0);}
      if (Difference<100000){Serial.print(0);}
      if (Difference<10000){Serial.print(0);}
      if (Difference<1000){Serial.print(0);}
      if (Difference<100){Serial.print(0);}
      if (Difference<10){Serial.print(0);}
      Serial.print(Difference);
      Serial.println(";");
    }
    Start=false;
    Stop=false;
  }
}

ISR(TCB1_INT_vect){
  //Interrupt ISR for the start signal
  CounterAtStart = TCB1.CCMP;
  Start=true;
  Stop=false;
  EventNanosec = (overflowCounter * TCB2.CCMP + CounterAtStart)*clkSpeed; //#cycles times clockspeed (since second ticked over)
  EventSec = second;
  TCB1.INTFLAGS = TCB_CAPT_bm; //clear flag
}

ISR(TCB0_INT_vect){
  //Interrupt ISR for the stop signal
  CounterAtStop = TCB0.CCMP;
  if (Start==true){Stop=true;} //We only raise flag if in correct order
  TCB0.INTFLAGS = TCB_CAPT_bm; //clear flag
}


unsigned long NanosecondsNow(){
  return (overflowCounter * TCB1.CCMP + TCB1.CNT)*clkSpeed;
}


ISR(PORTC_PORT_vect){
  //called every sec by GPS
  cyclesInSecond = overflowCounter * TCB2.CCMP + TCB2.CNT; //How many cycles passed since last PPS event
  clkSpeed=1e9/cyclesInSecond; //Recompute clockspeed (in ns)
  TCB2.CNT=0; //reset counter
  TCB1.CNT=0;
  TCB0.CNT=0;
  overflowCounter=0; //reset overflow, we entered new second
  second =  second + 1; //One second passed
  if (Ready==true and startup==true){
    startup=false; // we can enter normal mode
  }
  PORTC.INTFLAGS = 64;
}

ISR(TCB2_INT_vect){ //Call when we reach TOP
  TCB1.CNT=TCB2.CNT; //keep others in synch
  TCB0.CNT=TCB2.CNT;
  
  overflowCounter+=1; //We overflowed
  TCB2.INTFLAGS = TCB_CAPT_bm; //clear flag
}

void StopLoggingCheck(){
  //This we call to check if we were told to stop
  if (Serial.available() > 0) {
    // read the incoming byte:
    char message = Serial.read();
    if (message=='s'){
      Serial.print("Stopping logging session;");
      nanosecond=NanosecondsNow();
      printTime(second,nanosecond,true);
      logging=false;
    }
   }
}

void printTime(unsigned long sec,unsigned long nanosec, bool printDate){
  //Prints the time, and perhaps the date also
  //Idea is to never increment min and hr variables, instead we compute what we need to print from sec and nanosec
  int PrintNanosecond=nanosec%1000;
  int PrintMicrosecond=(nanosec%1000000)/1000;
  int PrintMillisecond=(nanosec%1000000000)/1000000;
  int PrintSecond=(sec+nanosec/1000000000)%60;
  int PrintMinute=(minute+(sec+nanosec/1000000000)/60)%60;
  int PrintHour=hour+(minute+(sec+nanosec/1000000000)/60)/60;

  //print in chosen format:
  if (PrintHour < 10) { Serial.print('0'); }
  Serial.print(PrintHour, DEC); Serial.print(':');
  if (PrintMinute < 10) { Serial.print('0'); }
  Serial.print(PrintMinute, DEC); Serial.print(':');
  if (PrintSecond < 10) { Serial.print('0'); }
  Serial.print(PrintSecond, DEC); Serial.print('.');
  
  if (PrintMillisecond < 10) {
    Serial.print("00");
  } else if (PrintMillisecond > 9 && PrintMillisecond < 100) {
    Serial.print("0");
  }
  Serial.print(PrintMillisecond); Serial.print(':');
  
  if (PrintMicrosecond < 10) {
    Serial.print("00");
  } else if (PrintMicrosecond > 9 && PrintMicrosecond < 100) {
    Serial.print("0");
  }
  Serial.print(PrintMicrosecond); Serial.print(':');

  if (PrintNanosecond < 10) {
    Serial.print("00");
  } else if (PrintNanosecond > 9 && PrintNanosecond < 100) {
    Serial.print("0");
  }
  Serial.print(PrintNanosecond); Serial.print(':');
  
  if (printDate){
    Serial.print(";Date:");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.print(GPS.year, DEC);
  } else {
    Serial.print(";");
  }
}

ISR(PORTF_PORT_vect){ // called by pin event
  PORTF.INTFLAGS = 32;//clear Flag
}
ISR(PORTA_PORT_vect){ // called by pin event
  PORTA.INTFLAGS = 4;//clear Flag
}
