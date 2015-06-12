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

// ADC Pinout: https://forum.pjrc.com/attachment.php?attachmentid=1793&d=1396800490
// See Updates at the bottom of the page: https://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1


#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
//#include <input_adc2.h>


const int pin1 = A2;
const int pin2 = A13;

// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
//

AudioInputAnalog         adc1(pin1, 1);           
//AudioInputAnalog2         adc2(pin2);  //  Do we want both audio components on the same pin?        

//AudioAnalyzeToneDetect   tone1;          

AudioAnalyzeFFT256    myFFT;
//AudioAnalyzeFFT256    myFFT2;

//AudioConnection          patchCord1(adc1, tone1);

//AudioConnection          patchCord2(adc2, myFFT2);
AudioConnection          patchCord3(adc1, myFFT);



void setup() {
  Serial.begin(9600);
  while( !Serial );
  Serial.println("initialized.");
  AudioMemory(64);
  }

void loop() {
  float n;
  int i;

  if (myFFT.available()) {
    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor
    Serial.print("myFFT 1: ");
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
  } //else { Serial.println("Not available"); }
//  if (myFFT2.available()) {
//    // each time new FFT data is available
//    // print it all to the Arduino Serial Monitor
//    Serial.print("myFFT 2: ");
//    for (i=0; i<40; i++) {
//      n = myFFT2.read(i);
//      if (n >= 0.01) {
//        Serial.print(n);
//        Serial.print(" ");
//      } else {
//        Serial.print("  -  "); // don't print "0.00"
//      }
//    }
//    Serial.println();
//  } else { Serial.println("Not available"); }
//  Serial.println("loop");
}



