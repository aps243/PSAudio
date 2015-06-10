// FFT Test
//
// Compute a 1024 point Fast Fourier Transform (spectrum analysis)
// on audio connected to the Left Line-In pin.  By changing code,
// a synthetic sine wave can be input instead.
//
// The first 40 (of 512) frequency analysis bins are printed to
// the Arduino Serial Monitor.  Viewing the raw data can help you
// understand how the FFT works and what results to expect when
// using the data to control LEDs, motors, or other fun things!
//
// This example code is in the public domain.

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "input_adc2.h"

const int myInput = A2;


// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
//
//AudioInputAnalog2         adc1(A2);           
//AudioInputAnalog2        adc1; 
AudioInputAnalog2         adc1(myInput);           
//AudioAnalyzeFFT256       fft256_1;      
//AudioAnalyzeToneDetect   tone1;          
AudioAnalyzeFFT256    myFFT;
//AudioConnection          patchCord1(adc1, tone1);
AudioConnection          patchCord2(adc1, myFFT);
//AudioConnection          patchCord3(adc2, myFFT);



void setup() {
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(50);

  }

void loop() {
  float n;
  int i;

  if (myFFT.available()) {
    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor
    Serial.print("FFT: ");
    for (i=0; i<40; i++) {
      n = myFFT.read(i);
      if (n >= 0.01) {
        Serial.print(n);
        Serial.print(" ");
      } else {
        Serial.print("  -  "); // don't print "0.00"
      }
    }
    Serial.println();
  }
}


