#include <SD.h>   // To Avoid Compiler Complaints
#include <SPI.h>  // to avoid another compiler complaint
#include <Wire.h> // and another
#include <ADC.h>
#include <ADC_Module.h>
#include <Audio.h>

void blinkLED(int pin);

const int LEDpin = 13;

AudioAnalyzeFFT1024 FFT1;
AudioAnalyzeFFT1024 FFT2;

AudioInputAnalog adc1(A11, 0);  // Pins on two 
AudioInputAnalog adc2(A13, 1);  // different ADC's


AudioConnection patchCord1(adc1, 0, FFT1, 0);
AudioConnection patchCord2(adc2, 0, FFT2, 0);


void setup()
{ 
  AudioMemory(24);
  Serial.begin(9600);
  Serial.println("Starting...");
  
  pinMode(LEDpin, OUTPUT);
  for(int i = 0; i < 5; i++)
    { blinkLED(LEDpin); }
  
  //FFT1.windowFunction(AudioWindowFlattop1024);
  //FFT2.windowFunction(AudioWindowFlattop1024);
  
  Serial.println("Finished init");
  
}
int counter = 2500000; // for debugging only
void loop() 
{
 
 //blinkLED(LEDpin);
  
 float n;
 int i;
 
  if(FFT2.available())
  {
   Serial.print("FFT: ");
   for(i=0; i<40; i++)
   {
     n = FFT1.read(i);
     if(n >= 0.01)
     {
       Serial.print(n);
       Serial.print(" ");
     }
     else
     {
       Serial.print("  -  ");
     }
   }
   Serial.println();
  }  //else { Serial.println("ADC1 not available..."); }
  
  counter--;
  if ( counter == 0 )
    { Serial.println("Halted.") ; while( 1 ) ; }
 
//  if(FFT2.available())
//  {
//   Serial.print("FFT: ");
//   for(i=0; i<40; i++)
//   {
//     n = FFT2.read(i);
//     if(n >= 0.01)
//     {
//       Serial.print(n);
//       Serial.print(" ");
//     }
//     else
//     {
//       Serial.print("  -  ");
//     }
//   }
//   Serial.println();
//  }  else { Serial.print("ADC2 not available..."); }
  
}

void blinkLED(int ledPin)
{
  digitalWrite(ledPin, HIGH);   // set the LED on
  delay(333);                  // wait for a second
  digitalWrite(ledPin, LOW);    // set the LED off
  delay(333);                  // wait for a second 
}
