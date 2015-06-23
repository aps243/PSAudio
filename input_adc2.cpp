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

 // Author: Zach Richard

 // The purpose of this library is to enable the second 
 // ADC to be used for simultanious analog audio input.


#include "input_adc2.h"
#include "utility/dspinst.h"

// #include "../ADC/ADC_Module.h"
 uint16_t AudioInputAnalog2::dc_average = 0;


void AudioInputAnalog2::init(uint8_t pin)//, uint8_t other_adc)
{
	ADC *adc = new ADC();
	uint32_t i, sum=0;

	// Configure the ADC and run at least one software-triggered
	// conversion.  This completes the self calibration stuff and
	// leaves the ADC in a state that's mostly ready to use


	// analogReadRes(16);
	// analogReference(INTERNAL); // range 0 to 1.2 volts
	// analogReadAveraging(8);
	// ^^^-Do That-^^^
	// but with the ADC object here
	adc->setResolution(16, ADC_1);
	adc->setReference(ADC_REF_1V2, ADC_1);
	adc->setAveraging(1, ADC_1);
	// and some more stuff
	adc->setConversionSpeed(ADC_VERY_HIGH_SPEED, ADC_1);
	adc->setSamplingSpeed(ADC_VERY_HIGH_SPEED, ADC_1);
	adc->enableInterrupts(ADC_1);


	// Do many normal reads, to start with a nice DC level
	for (i=0; i < 1024; i++) {
		sum += adc->analogRead(pin, ADC_1);
	}
	dc_average = sum >> 10;

	// set the programmable delay block to trigger the ADC at 44.1 kHz
    // values will be converted every 22.6666666661 microseconds.
	// adc->adc1->stopPDB();	-- probably unneseccary
    adc->adc1->startPDB( AUDIO_SAMPLE_RATE ); //frequency in Hz
    adc->startContinuous(pin, ADC_1);

	// We shouldn't need to use the DMA for this......
	//// enable the ADC for hardware trigger and DMA
	// if (adc_to_use == 0)
	// 	ADC0_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;			
	// else
	// 	ADC1_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;

	//// set up a DMA channel to store the ADC data
	// dma.begin(true);

	// dma.TCD->SADDR = (adc_to_use == 0) ? &ADC0_RA : &ADC1_RA;

	// dma.TCD->SOFF = 0;
	// dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
	// dma.TCD->NBYTES_MLNO = 2;
	// dma.TCD->SLAST = 0;
	// dma.TCD->DADDR = analog_rx_buffer;
	// dma.TCD->DOFF = 2;
	// dma.TCD->CITER_ELINKNO = sizeof(analog_rx_buffer) / 2;
	// dma.TCD->DLASTSGA = -sizeof(analog_rx_buffer);
	// dma.TCD->BITER_ELINKNO = sizeof(analog_rx_buffer) / 2;
	// dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

	// if (adc_to_use == 0)
	// 	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
	// else
	// 	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);

	// update_responsibility = update_setup();
	// dma.enable();
	// dma.attachInterrupt(isr);
}

// Necessary Functions for ADC interupts for PDB
/*********************************************************************************************/
// If you enable interrupts make sure to call readSingle() to clear the interrupt.
void AudioInputAnalog2::adc1_isr() {
       adc->adc1->readSingle();
        //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
}

// pdb interrupt is enabled in case you need it.
void pdb_isr(void) {
        PDB0_SC &=~PDB_SC_PDBIF; // clear interrupt
        digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
}
/*********************************************************************************************/

void AudioInputAnalog2::update(void)
{
	audio_block_t *output=NULL;
	unsigned int dc;
	int16_t s, *p, *end;
	uint32_t tmp;
	int i;

	output = allocate();

	//__disable_irq();-- may cause issues since PDB uses an interupt to trigger
	// fill the block -- not sure if this is fast enough.
	// 				  -- PDB may not be available when we try to read from it.
	for (i=0; i<AUDIO_BLOCK_SAMPLES; i++)
	{
		setADCval();
		output->data[i] = getADCval(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative.
	}
	//__enable_irq();

	//remove DC offset -- possibly unnecessary
	dc = dc_average;
	p = output->data;
	end = p + AUDIO_BLOCK_SAMPLES;
	do {
		tmp = (uint16_t)(*p) - (int32_t)dc;
		s = signed_saturate_rshift(tmp, 16, 0);
		*p++ = s;
		dc += s / 12000;  // slow response, remove DC component
	} while (p < end);
	dc_average = dc;
	
	transmit(output);
	release(output);
}