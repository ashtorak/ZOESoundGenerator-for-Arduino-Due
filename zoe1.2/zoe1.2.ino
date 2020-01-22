/*
 * This sketch reads the pedal position and motor RPM from the ZOE diagnostic bus
 * and synthesizes an engine sound based on these inputs by running through a 
 * sample table with a timed interrupt at a fixed rate. The pitch is shifted by
 * skipping or repeating a sample from the wave table. To do this fast, 32 bit fixed point
 * calculations are used (not sure how necessary this even is). The sound synthesis 
 * is based on http://rcarduino.blogspot.com/2012/12/arduino-due-dds-part-1-sinewaves-and.html
 * The CAN bus read out has been taken from an older CanZE dongle software for the DUE,
 * which is no longer supported anymore on http://canze.fisch.lu
 */

#include "porscheIdle.h"
#include "porsche3750.h"
#define SAMPLES_PER_CYCLE 84803 // numbers of samples from include file
#define SAMPLES_PER_CYCLE_FIXEDPOINT (SAMPLES_PER_CYCLE<<15) //bit shift needs to be adjusted according to max. number of samples

#include "bang.h"
#define SAMPLES_BANG sizeof(tableBang)/sizeof(tableBang[0]) // automatically get table size (sizeof() returns bytes)
#define SAMPLES_BANG_FIXEDPOINT (SAMPLES_BANG<<15) //bit shift needs to be adjusted according to max. number of samples
boolean bang = 0;

// sample volume applied at run time with float multiplier
// caution: if the volume is too high you will get distortion
const float volumeIdlingConst = 2.0;
float volumeIdling = volumeIdlingConst;
const float volumeRevingConst = 1.0;
float volumeReving = volumeRevingConst;
float revingOff;
const float volumeBangConst = 1.5;
float volumeBang = volumeBangConst;

// run the sound modulation with a fixed interval
int timer = millis();
int dtSoundModulator = 10; // in ms

// simulate engine sound for testing with buttons on digital inputs
boolean simulateInput = 1;

// pitch is shifted by running faster or slower through the wave table
int pitchBase = 22000; //base wave table cycling speed
// divide rpm by this number to have good pitch shifting
float rpmToPitch = 2000.0;

// the phase accumulator points to the current sample in our wavetable
uint32_t ulPhaseAccumulator = 0;
uint32_t ulPhaseAccumulatorBang = 0;
// the phase increment controls the rate at which we move through the wave table
// higher values = higher frequencies
volatile uint32_t ulPhaseIncrement = pitchBase;

// The Due DAC accepts 12 bit unsigned values, so 0...4096, zero output = 2048
// However, wave table values are 16 bit. So we shift it by 4 bits to get to 16.
uint32_t ulOutput = 2048<<4;
// Please note that DAC output range is actually from 0.55 V to 2.75 V only.

//---------------------
// can bus stuff below
//---------------------

// load the SPI library (needed for the CAN, SD & LCD library)
#include <SPI.h>

// load the CAN library
#include "variant.h"
#include <due_can.h>   

#include "Structs.h"

CAN_FRAME* dataArray = 0;
int dataArraySize = 0;

CAN_FRAME EMPTY;

ISO_MESSAGE isoMessage;

int T = millis();

// read buffer
String readBuffer = "";
//String filter = "";

// for reading out bus values
int byte1 = 0;
int byte2 = 0;
int elecRPM = 0;
int pedal = 0;

// for sound modulation
int RPM = 0;
int pedalCounter = 0;
int pedalCounterBang = 0;

//---------------------
// switches below
//---------------------

//switch on/off loop functions
const int switchPin = 53;
//simulate pedal
const int pedalPinLow = 52;
const int pedalPinMed = 50;
const int pedalPinHigh = 48;

void setup()
{  
  //init EMPTY frame
  EMPTY.length=8;
  for(int i=0; i<EMPTY.length; i++)
    EMPTY.data.bytes[i]=0;
  
  // initialise the serial connection
  Serial.begin(9600);
    
  // Initialize CAN0, Set the proper baud rates here
  Can0.begin(CAN_BPS_500K);
  
  setFilter(0);

  // setup audio
      
  /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC4);

  /* we want wavesel 01 with RC */
  TC_Configure(/* clock */TC1,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
  TC_SetRC(TC1, 1, 2*238); // sets interrupt timer: 238 for 44.1 Khz, 2*238 for 22.05 kHz
  TC_Start(TC1, 1);
 
  // enable timer interrupts on the timer
  TC1->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;
 
  /* Enable the interrupt in the nested vector interrupt controller */
  /* TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number (=(1*3)+1) for timer1 channel1 */
  NVIC_EnableIRQ(TC4_IRQn);
  
  // this is a cheat - enable the DAC
  analogWrite(DAC1,0);

  // caution: pinMode has to be set after Can0.begin() !!
  pinMode(switchPin, INPUT);
  pinMode(pedalPinLow, INPUT);
  pinMode(pedalPinMed, INPUT);
  pinMode(pedalPinHigh, INPUT);
}

void loop()
{
  if(digitalRead(switchPin)) 
  {
    CAN_FRAME incoming;
  
    if (Can0.available() > 0) 
    {
      Can0.read(incoming); 
       
      processFrame(incoming);
    }
    
    readIncoming(); //serial commands
  
    // adjust sound with fixed interval according to dtSoundModulator in ms
    if(millis()-timer>dtSoundModulator)
    {
      soundModulator();
      timer=millis();
    }
  }
}

void soundModulator()
{
  if(elecRPM<1 && pedal>1) // allow reving with pedal when car is not moving
  {
    pedalCounter+=(pedal*dtSoundModulator); //the counter will be added to the RPM
  }
  else if(pedal<1 && pedalCounter>0 && elecRPM<1) // rev down slowly when pedal is not pushed
  {
    pedalCounter-=dtSoundModulator*44;
    if(pedalCounter<100) pedalCounter=0;
  }
  else if(pedalCounter>0 && elecRPM>1) // stop pedal reving when car is moving
  {
    pedalCounter-=dtSoundModulator*elecRPM/10;
    if(pedalCounter<100) pedalCounter=0;
  }
 
  RPM=elecRPM+pedalCounter/35;

  // adjust idling volume, cut-off is last number in formula
  volumeIdling = volumeIdlingConst/(1+pow(2.71828,(0.04*(RPM-222))));

  // adjust reving cut-off
  revingOff = 1/(1+pow(2.71828,(0.04*(RPM-6000))));
  if(pedalCounter>1) revingOff = 1; // for going really high with pedal :)

  // adjust reving volume
  volumeReving = (0.5+pedal/260.0)*volumeRevingConst/(1+pow(2.71828,(-0.04*(RPM-222))))*revingOff;

  // increase pitch by RPM always starts at base pitch
  ulPhaseIncrement = pitchBase*(1+RPM/(rpmToPitch));

  // add a bang
  pedalCounterBang += (pedal-40)*dtSoundModulator;
  if(pedalCounterBang<0) pedalCounterBang = 0;
  else if(pedalCounterBang>40000)
  {
    if(!bang && RPM>4500 && pedal<1) 
    {
      bang = 1;
      pedalCounterBang = 0;
    }
  }

  // simulate a simple pedal input
  if(simulateInput) 
  {
      if(digitalRead(pedalPinLow)) pedal=10;
      else if(digitalRead(pedalPinMed)) pedal=64;
      else if(digitalRead(pedalPinHigh)) pedal=128;
      else pedal=0;
   }
}

// interrupt function for DAC output, which is a loop started in setup()
void TC4_Handler()
{
  if(digitalRead(switchPin))
  {
    // We need to get the status to clear it and allow the interrupt to fire again
    TC_GetStatus(TC1, 1);
   
    ulPhaseAccumulator += ulPhaseIncrement;   // 32 bit phase increment, see below
    
    // if the phase accumulator over flows - we have been through one cycle at the current pitch,
    // now we need to reset the grains ready for our next cycle
    if(ulPhaseAccumulator > SAMPLES_PER_CYCLE_FIXEDPOINT)
    {
     ulPhaseAccumulator -= SAMPLES_PER_CYCLE_FIXEDPOINT;
    }

    if(bang)  
    {
      ulPhaseAccumulatorBang += pitchBase;
      if(ulPhaseAccumulatorBang > SAMPLES_BANG_FIXEDPOINT)
      {
       ulPhaseAccumulatorBang -= SAMPLES_BANG_FIXEDPOINT;
       bang = 0;
      }
    }
    
    // synthesize the current sample with 16 bit wave table samples
    
    ulOutput = 2048<<4; // zero offset for DAC output
    
    ulOutput += volumeIdling*(tablePorscheIdle[ulPhaseAccumulator>>15]); // bit shift depends on sample number
    ulOutput += volumeReving*(tablePorsche3750[ulPhaseAccumulator>>15]);
    if(bang) ulOutput += volumeBang*(tableBang[ulPhaseAccumulatorBang>>15]);
    
    ulOutput=ulOutput>>4; // get it back down to 12 bit for DAC
    
    if(ulOutput>4096) ulOutput = 4096;
    if(ulOutput<0) ulOutput = 0;
    
    // we cheated and user analogWrite to enable the dac, but here we want to be fast so
    // write directly 
    dacc_write_conversion_data(DACC_INTERFACE, ulOutput);
  }
}

void processFrame(CAN_FRAME &frame) 
{      
  String idHex = String(frame.id,HEX);

  if(idHex=="186") // position of accelerator pedal, from bit 40 to 48, range 0...125
  {        
    pedal = frame.data.bytes[5];
  }
  else if(idHex=="1f8") // elecRPM, from bit 40 up to and including 50, range 0...1200 (multiply by 10 for rpm)
  {
    byte1 = frame.data.bytes[5]; // bit 40 to 47
    byte2 = frame.data.bytes[6]; // bit 48 to 55
    elecRPM = (byte1<<3 | byte2>>5)*10; // combine 0111 1111 1000 and 0111    
  }
}
   
void readIncoming()
{
  if(Serial.available())
  {
    char ch = Serial.read();
    if(ch=='\n' || ch=='\r')
    {
      if(readBuffer!="")
      {
        processCommand(readBuffer);
        readBuffer="";        
      }
    }
    else
    {
      readBuffer+=ch;
    }
  }  
}

COMMAND decodeCommand(String &input)
{
  COMMAND result;

  // trim whitespaces
  input.trim();

  // stop if input is empty
  if(input.length()==0) return result;
  
  // the first letter is the command
  result.cmd = input.charAt(0);
  input.remove(0,1); 

  // if there is something more,
  if(input.length()!=0)
  {
    // get the ID
    char ch;
    String id = "";
    do
    {
      ch=input.charAt(0);
      if(ch!=',') id+=ch;
      input.remove(0,1); 
    }
    while(input.length()!=0 && ch!=',');
    result.id=hexToDec(id);
  }
  
  // if there is something more,
  if(input.length()!=0)
  {
      // get the REQUEST
      char ch;
      String request = "";
      do
      {
        ch=input.charAt(0);
        if(ch!=',') request+=ch;
        input.remove(0,1); 
      }
      while(input.length()!=0 && ch!=',');
      for(int i=0; i<request.length(); i+=2)
      {
        result.request[result.requestLength]=hexToDec(request.substring(i,i+2));
        result.requestLength++;
      }
  }

  // if there is something more,
  if(input.length()!=0)
  {
      // get the REPLY
      char ch;
      String reply = "";
      do
      {
        ch=input.charAt(0);
        if(ch!=',') reply+=ch;
        input.remove(0,1); 
      }
      while(input.length()!=0 && ch!=',');
      for(int i=0; i<reply.length(); i+=2)
      {
        result.reply[result.replyLength]=hexToDec(reply.substring(i,i+2));
        result.replyLength++;
      }
  }
  return result;
}

void processCommand(String &line)
{
  COMMAND command = decodeCommand(line);
  
  if(command.cmd=='d')
  {
    rpmToPitch-=10;
    Serial.println("rpmToPitch: "+String(rpmToPitch)+"\r\n");
  }
  else if(command.cmd=='u')
  {
    rpmToPitch+=10;
    Serial.println("rpmToPitch: "+String(rpmToPitch)+"\r\n");
  }
}

void setFilter(int filter)
{
  //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames

  //extended
  for (filter = 0; filter < 3; filter++) {
    Can0.setRXFilter(filter, 0, 0, true);
    //Can1.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
    Can0.setRXFilter(filter, 0, 0, false);
    //Can1.setRXFilter(filter, 0, 0, false);
  }   
}

String getHex(int num)
{
  String stringOne =  String(num, HEX); 
  if(stringOne.length()<2) stringOne="0"+stringOne;
  return stringOne;
}

unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}
