  /*
 * This sketch reads the pedal position and motor RPM from the ZOE diagnostic bus
 * and synthesizes an engine sound based on these inputs by running through a 
 * sample table with a timed interrupt at a fixed rate. The pitch is shifted by
 * skipping or repeating a sample from the wave table. The sound synthesis 
 * is based on http://rcarduino.blogspot.com/2012/12/arduino-due-dds-part-1-sinewaves-and.html
 * The CAN bus read out has been taken from an older CanZE dongle software for the DUE,
 * which is no longer supported anymore on http://canze.fisch.lu
 * The sample readout from SD card is based on the Groovuino library.
 */

// simulate engine sound for testing with buttons on digital inputs
boolean simulateInput = 0;

#include <SdFat.h>
#include "sampler.h"

SdFat sd;
const int samplerCount = 6; // 1 for idling, 4 for reving, 1 for a bang
Sampler sampler[samplerCount];
int sampleSet = 0;

// main sample volume is set when loading a car (the following are default values)
// it represents the max volume
// caution: if the volume is too high you will get distortion
float volumeIdlingMain = 1.0;
float volumeRevingMain = 1.0;
float volumeBangMain = 1.0;
// these volume floats will be changed by the sound modulator in the main loop
float volumeIdling = 1.0;
float volumeRevingLow = 1.0;
float volumeRevingMid = 1.0;
float volumeRevingHigh = 1.0;
float volumeRevingMax = 1.0;
float volumeBang = 1.0;
float volumePedal = 1.0;
float volumePedalBefore = 0.0;
float volumePedalTimeConstant = 50.0; // in ms

float revMod;
float revIdleRPM;
float revLowRPM;
float revMidRPM;
float revHighRPM;
float revMaxRPM;
float RPMCutOff;
float crossRPMIdleLow;
float crossRPMLowMid;
float crossRPMMidHigh;
float crossRPMHighMax;
float crossRPMRange; // +/-
int gear = 1;

// run the sound modulation with an interval
int timer = millis();
int dt = 0;
int dtSoundModulator = 10; // calculate the sound every x milliseconds

// for testing via Serial bus
float serialVar = 0.0;

// The Due DAC uses 12 bit unsigned values, so 0...4096, zero output = 2048
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
int elecRPM = 0; // range 0...12000
int pedal = 0; // range 0...250 corresponds to 0...100%

// for sound modulation
int RPM = 0;
int pedalCounter = 0;
int pedalCounterBang = 0;

//---------------------
// pin definitions below
//---------------------

//switch on/off loop functions
const int switchPin = 53;
//simulate pedal
const int pedalPinLow = 52;
const int pedalPinMed = 50;
const int pedalPinHigh = 48;
const int sampleSetPin = 46;
// pin for SD reader has to be 10 with SDFat library on Due
const int chipSelectPin = 10;


void setup()
{ 
  // initialise the serial connection
  Serial.begin(9600);
  Serial.println("setup start");

  //init EMPTY frame
  EMPTY.length=8;
  for(int i=0; i<EMPTY.length; i++)
    EMPTY.data.bytes[i]=0;
  
  // Initialize CAN0, Set the proper baud rates here
  Can0.begin(CAN_BPS_500K);
  
  setFilter(0);


  // setup SD card and samples
  // SD Pin depends on board, max. speed depends on card reader and connection (empirical)
  sd.begin(chipSelectPin, SD_SCK_MHZ(8));
  
  for(int i=0; i<samplerCount; i++)
  {
    sampler[i].init();
  }

  selectCar(sampleSet);
  
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

  // setup pins
  // caution: pinMode has to be set after Can0.begin() !!
  pinMode(switchPin, INPUT);
  pinMode(pedalPinLow, INPUT);
  pinMode(pedalPinMed, INPUT);
  pinMode(pedalPinHigh, INPUT);

  Serial.println("setup finished");
}

void selectCar(int number)
{
  NVIC_DisableIRQ(TC4_IRQn);
    
  for(int i=0; i<samplerCount; i++)
  {
    if(sampler[i].playSample) sampler[i].sstop();
    sampler[i].sloop(false);
  }

  if(number==2)
  {
    sampler[0].load("porscheIdle.wav");
    sampler[1].load("porsche3000.wav");
    sampler[2].load("porsche3750.wav");
    sampler[3].load("porsche5500.wav");
    sampler[4].load("porsche7000.wav");
    revMod = 0.8;
    revIdleRPM = 1500*revMod;
    revLowRPM = 3000*revMod;
    revMidRPM = 3750*revMod;
    revHighRPM = 5500*revMod;
    revMaxRPM = 7000*revMod;
    volumeIdlingMain = 1.5;
    volumeRevingMain = 1.2;
    volumeBangMain = 2.0;
    
    setRPMCrossPoints();
    playAndLoop(0,4);
  }
  else if(number==3)
  {
    sampler[0].load("cobraIdle.wav");
    sampler[1].load("cobra2886.wav");
    sampler[2].load("cobra3209.wav");
    sampler[3].load("cobra4360.wav");
    sampler[4].load("cobra6823.wav");
    revMod = 0.8;
    revIdleRPM = 1500*revMod;
    revLowRPM = 2886*revMod;
    revMidRPM = 3209*revMod;
    revHighRPM = 4360*revMod;
    revMaxRPM = 6823*revMod;
    volumeIdlingMain = 2.0;
    volumeRevingMain = 1.1;
    volumeBangMain = 2.0;
    
    setRPMCrossPoints();
    playAndLoop(0,4);
  }
  else if(number==4)
  {
    sampler[0].load("porscheIdle.wav");
    sampler[1].load("porsche3000.wav");
    sampler[2].load("porsche3750.wav");
    sampler[3].load("porsche5500.wav");
    sampler[4].load("porsche7000.wav");
    revMod = 0.5;
    revIdleRPM = 1500*revMod;
    revLowRPM = 3000*revMod;
    revMidRPM = 3750*revMod;
    revHighRPM = 5500*revMod;
    revMaxRPM = 7000*revMod;
    volumeIdlingMain = 1.5;
    volumeRevingMain = 1.2;
    volumeBangMain = 2.0;
    
    setRPMCrossPoints();
    playAndLoop(0,4);
  }
  else if(number==1)
  {
    sampler[3].load("horseGallop1.wav");
    sampler[4].load("horseGallop2.wav");
    revMod = 1.0;
    revHighRPM = 1000*revMod;
    revMaxRPM = 2000*revMod;
    volumeIdlingMain = 1.0;
    volumeRevingMain = 1.2;
    
    setRPMCrossPoints();
    
    playAndLoop(3,4);
  }

  NVIC_EnableIRQ(TC4_IRQn);
}

// set a range of samples to play and loop
void playAndLoop(int i1, int i2)
{
  for(int i=i1; i<=i2; i++) 
  {
    sampler[i].splay();
    sampler[i].sloop(true);
    sampler[i].buffill(); // open file
    sampler[i].buffill(); // fill buffer
  }  
}

void setRPMCrossPoints()
{
  crossRPMLowMid = revLowRPM+(revMidRPM-revLowRPM)/2;
  crossRPMMidHigh = revMidRPM+(revHighRPM-revMidRPM)/2;
  crossRPMHighMax = revHighRPM+(revMaxRPM-revHighRPM)/2;
  crossRPMIdleLow = 200*revMod; // +/-
  RPMCutOff = 11000*revMod;   
}

void loop()
{
  CAN_FRAME incoming;

  if (Can0.available() > 0) 
  {
    Can0.read(incoming); 
     
    processFrame(incoming);
  }
  
  readIncoming(); //serial commands

  // adjust sound with fixed interval according to dtSoundModulator in ms
  dt = millis()-timer;
  if(dt>dtSoundModulator && sampleSet!=0)
  {
    soundModulator();
    timer=millis();
  }

  // fill sampler buffers
  for(int i=0; i<samplerCount; i++)
  {
    if(sampler[i].volume>0.1) sampler[i].buffill();
    // to reduce SD card reads, fill buffer only when volume is high enough
  }
  
  // simulate a simple pedal input
  if(simulateInput) 
  {
    if(digitalRead(pedalPinLow)) pedal=10;
    else if(digitalRead(pedalPinMed)) pedal=64;
    else if(digitalRead(pedalPinHigh)) pedal=128;
    else pedal=0;
  }
   
  if(digitalRead(sampleSetPin)) 
  {
    delay(200);
    sampleSet++;
    if(sampleSet>4) sampleSet = 0;
    selectCar(sampleSet);
    pedalCounter = 0;
  }

}

void soundModulator()
{
  if(elecRPM<1 && sampleSet!=1) // allow reving with pedal when car is not moving
  {
    pedalCounter+=(pedal/2-22)*dt; //the counter will be added to the RPM
    if(pedalCounter>(9*RPMCutOff)) pedalCounter = 9*RPMCutOff; 
    if(pedalCounter<100) pedalCounter=0;
  }
  else if(pedalCounter>0 && elecRPM>1) // stop pedal reving when car is moving
  {
    pedalCounter-=dt*elecRPM/10;
    if(pedalCounter<100) pedalCounter=0;
  }
 
  RPM=elecRPM+pedalCounter/10;

  // adjust volumes

  if(sampleSet==4) // gear switching only for Porsche samples
  {
    if(RPM<(0.5*7000) && gear!=1)
    {
      gear = 1;
      
      revMod = 0.5;
      revLowRPM = 3000*revMod;
      revMidRPM = 3750*revMod;
      revHighRPM = 5500*revMod;
      revMaxRPM = 7000*revMod;
      setRPMCrossPoints();
    }
    else if(RPM>=(0.6*7000) && RPM<(0.7*7000) && gear!=2)
    {
      gear = 2;
      
      revMod = 0.7;
      revHighRPM = 5500*revMod;
      revMaxRPM = 7000*revMod;
      setRPMCrossPoints();
      }
    else if(RPM>=(0.84*7000) && gear!=3)
    {
      gear = 3;
      
      revMod = 0.96;
      revHighRPM = 5500*revMod;
      revMaxRPM = 7000*revMod;
      setRPMCrossPoints();
    }
  }

  if(sampleSet==1) // horse needs special treatment
  {
    volumeIdling = volumeIdlingMain;
    
    if(RPM<revMaxRPM)
    {    
      if(pedal>200) 
      {
        if(!sampler[2].playSample)
        {
          sampler[2].load("horseAngry.wav");
          sampler[2].splay();
          sampler[2].setVolume(volumeIdling);
          sampler[2].setPhaseIncrement(2.0); // 44 kHz samples!
          sampler[1].sstop();
          sampler[0].sstop();
        }
      }
      else if(pedal>99)
      { 
        if(!sampler[1].playSample && !sampler[2].playSample)
        {
          sampler[1].load("horseWhinny.wav");
          sampler[1].splay();
          sampler[1].setVolume(volumeIdling);
          sampler[1].setPhaseIncrement(2.0);
          sampler[2].sstop();
          sampler[0].sstop();
        }
      }
      else if(pedal>1)
      { 
        if(!sampler[0].playSample && !sampler[1].playSample && !sampler[2].playSample)
        {
          sampler[0].load("horseGrunts.wav");
          sampler[0].splay();
          if(elecRPM<1) sampler[0].setVolume(volumeIdling/2);
          else sampler[0].setVolume(volumeIdling/4);
          sampler[0].setPhaseIncrement(2.0);
          sampler[1].sstop();
          sampler[2].sstop();
        }
      } 
    }
      
    if(elecRPM<1)
    {
      volumeRevingHigh = 0;
      volumeRevingMax = 0;
    }
    else
    {
      volumeRevingHigh = volumeRevingMain/(1+pow(2.71828,(0.005*(RPM-(crossRPMHighMax+crossRPMRange)))));
      volumeRevingMax = volumeRevingMain/(1+pow(2.71828,(-0.005*(RPM-(crossRPMHighMax-crossRPMRange)))))/(1+pow(2.71828,(0.04*(RPM-revMaxRPM*1.5))));
    }
      
    sampler[3].setVolume(volumeRevingHigh); // gallop1
    sampler[4].setVolume(volumeRevingMax); // gallop2
    
    // adjust pitch with RPM
    sampler[3].setPhaseIncrement(1.7+RPM/revHighRPM/2);
    sampler[4].setPhaseIncrement(1.0+RPM/revMaxRPM);
  }
  else  // adjustments for normal reving
  {
    // apply a low pass filter on pedal press for volume modulation while keeping lowest volume on 30%
    volumePedal = 0.3+0.7*(volumePedalBefore + (pedal/250.0-volumePedalBefore)*dt/(dt+volumePedalTimeConstant));
    volumePedalBefore = volumePedal;

    volumeIdling = volumeIdlingMain/(1+pow(2.71828,(0.03*(RPM-(crossRPMIdleLow+25)))));
    volumeRevingLow = volumePedal*volumeRevingMain/(1+pow(2.71828,(-0.03*(RPM-(crossRPMIdleLow-25)))))/(1+pow(2.71828,(0.002*(RPM-(crossRPMLowMid+crossRPMRange)))));
    volumeRevingMid = volumePedal*volumeRevingMain/(1+pow(2.71828,(-0.002*(RPM-(crossRPMLowMid-crossRPMRange)))))/(1+pow(2.71828,(0.002*(RPM-(crossRPMMidHigh+crossRPMRange)))));
    volumeRevingHigh = volumePedal*volumeRevingMain/(1+pow(2.71828,(-0.002*(RPM-(crossRPMMidHigh-crossRPMRange)))))/(1+pow(2.71828,(0.002*(RPM-(crossRPMHighMax+crossRPMRange)))));
    volumeRevingMax = volumePedal*volumeRevingMain/(1+pow(2.71828,(-0.002*(RPM-(crossRPMHighMax-crossRPMRange)))))/(1+pow(2.71828,(0.04*(RPM-RPMCutOff))));
    
    sampler[0].setVolume(volumeIdling);
    sampler[1].setVolume(volumeRevingLow);
    sampler[2].setVolume(volumeRevingMid);
    sampler[3].setVolume(volumeRevingHigh);
    sampler[4].setVolume(volumeRevingMax);
    
    // adjust pitch with RPM
    sampler[0].setPhaseIncrement(1.0+RPM/revIdleRPM);
    sampler[1].setPhaseIncrement(0.6+RPM/revLowRPM/2);
    sampler[2].setPhaseIncrement(RPM/revMidRPM);
    sampler[3].setPhaseIncrement(RPM/revHighRPM);
    sampler[4].setPhaseIncrement(RPM/revMaxRPM);
  }
  
  // add a bang
  
  if(pedalCounterBang>55000 && sampleSet!=1) // not for horse
  {
    if(RPM>(4000*revMod) && pedal<1) 
    {
      sampler[5].load("bang01.wav");
      sampler[5].splay();
      volumeBang = volumeBangMain;
      sampler[5].setVolume(volumeBang);
      pedalCounterBang = 0;
    }
  }
  else if(pedalCounterBang>=0) pedalCounterBang += (pedal-99)*dt;
  else pedalCounterBang = 0;
  
}

// interrupt function for DAC output, which is a loop started in setup()
void TC4_Handler()
{
  if(digitalRead(switchPin))
  {
    // We need to get the status to clear it and allow the interrupt to fire again
    TC_GetStatus(TC1, 1);
    
    // synthesize the current output
    
    ulOutput = 2048; // 12 bit zero offset DAC output 

    for(int i=0; i<samplerCount; i++)
    {
      sampler[i].next();
    
      // output will only be returned if splay(true) has been set
      ulOutput += sampler[i].output();
    }
     
    if(ulOutput>4096) ulOutput = 4096;
    
    // we cheated and user analogWrite to enable the dac, but here we want to be fast so
    // write directly 
    dacc_write_conversion_data(DACC_INTERFACE, ulOutput);
  }
}


void processFrame(CAN_FRAME &frame) 
{      
  String idHex = String(frame.id,HEX);

  if(idHex=="186") // position of accelerator pedal, from bit 40 to 48, range 0...250
  {        
    pedal = frame.data.bytes[5];
    //Serial.println("pedal: " + String(pedal));
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
    serialVar-=10;
    Serial.println("serialVar: "+String(serialVar)+"\r\n");
  }
  else if(command.cmd=='u')
  {
    serialVar+=10;
    Serial.println("serialVar: "+String(serialVar)+"\r\n");
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
