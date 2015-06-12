/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef input_adc2_h_
#define input_adc2_h_

#include "AudioStream.h"
#include "DMAChannel.h"
#include <string.h>

//#include <ADC.h>

class AudioInputAnalog2 : public AudioStream
{
public:
        AudioInputAnalog2() : AudioStream(0, NULL) { init(A3); }
        AudioInputAnalog2(uint8_t pin) : AudioStream(0, NULL) { /*serialstatus("test");*/ init(pin); }
        virtual void update(void);
        friend void dma_ch10_isr(void); //cores/teensy3/mk20dx128.c:
	//// 25 DMA channel 9 transfer complete
private:
        static audio_block_t *block_left;
        static uint16_t block_offset;
	static uint16_t dc_average;
        static bool update_responsibility;
	static DMAChannel myDMA;
        //static ADC myADC;
	static void isr(void);
        static void init(uint8_t pin);
       // static void serialstatus(String msg);
};

#endif
