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

#include "input_adc.h"
#include "utility/pdb.h"
#include "utility/dspinst.h"

#define PDB0_CH1C1              *(volatile uint32_t *)0x40036038 // Channel 1 Control Register 1
#define PDB0_CH1S               *(volatile uint32_t *)0x4003603C // Channel 1 Status Register
#define PDB0_CH1DLY0            *(volatile uint32_t *)0x40036040 // Channel 1 Delay 0 Register
#define PDB0_CH1DLY1            *(volatile uint32_t *)0x40036044 // Channel 1 Delay 1 Register


// #include <Serial>// TMP
// Serial.begin(9600); // TMP

DMAMEM static uint16_t analog_rx_buffer[AUDIO_BLOCK_SAMPLES];
audio_block_t * AudioInputAnalog::block_left = NULL;
uint16_t AudioInputAnalog::block_offset = 0;
uint16_t AudioInputAnalog::dc_average = 0;
bool AudioInputAnalog::update_responsibility = false;
DMAChannel AudioInputAnalog::dma(false);
int AudioInputAnalog::adc_to_use = 0;

void AudioInputAnalog::setADC(int adc)
{	
	if ( adc != 1 )
		adc_to_use = 0;
	else
		adc_to_use = 1;
	//Serial.print("ADC to use changed to ");
	//Serial.println(adc_to_use);
}

void AudioInputAnalog::init(uint8_t pin)//, uint8_t other_adc)
{
	uint32_t i, sum=0;

	// Configure the ADC and run at least one software-triggered
	// conversion.  This completes the self calibration stuff and
	// leaves the ADC in a state that's mostly ready to use
	analogReadRes(16);
	analogReference(INTERNAL); // range 0 to 1.2 volts
	analogReadAveraging(8);
	// Actually, do many normal reads, to start with a nice DC level
	for (i=0; i < 1024; i++) {
		sum += analogRead(pin);
	}
	dc_average = sum >> 10;

	// set the programmable delay block to trigger the ADC at 44.1 kHz
	if (adc_to_use == 0) {
		if (!(SIM_SCGC6 & SIM_SCGC6_PDB )
		  || (PDB0_SC & PDB_CONFIG) != PDB_CONFIG
		  || PDB0_MOD != PDB_PERIOD
		  || PDB0_IDLY != 1
		  || PDB0_CH0C1 != 0x0101) {
			SIM_SCGC6 |= SIM_SCGC6_PDB;
			PDB0_IDLY = 1;
			PDB0_MOD = PDB_PERIOD;
			PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
			PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
			PDB0_CH0C1 = 0x0101;
		}
	}
	else
	{
		if (!(SIM_SCGC6 & SIM_SCGC6_PDB )
		  || (PDB0_SC & PDB_CONFIG) != PDB_CONFIG
		  || PDB0_MOD != PDB_PERIOD
		  || PDB0_IDLY != 1
		  || PDB0_CH1C1 != 0x0101) {
		  	SIM_SCGC3 |= (SIM_SCGC3_ADC1);
			SIM_SCGC6 |= SIM_SCGC6_PDB;
			PDB0_IDLY = 1;
			PDB0_MOD = PDB_PERIOD;
			PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
			PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
			PDB0_CH1C1 = 0x0101;
		}
	}


	// enable the ADC for hardware trigger and DMA
	if (adc_to_use == 0)
		ADC0_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;			
	else
		ADC1_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;

	// set up a DMA channel to store the ADC data
	dma.begin(true);

	dma.TCD->SADDR = (adc_to_use == 0) ? &ADC0_RA : &ADC1_RA;

	dma.TCD->SOFF = 0;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
	dma.TCD->NBYTES_MLNO = 2;
	dma.TCD->SLAST = 0;
	dma.TCD->DADDR = analog_rx_buffer;
	dma.TCD->DOFF = 2;
	dma.TCD->CITER_ELINKNO = sizeof(analog_rx_buffer) / 2;
	dma.TCD->DLASTSGA = -sizeof(analog_rx_buffer);
	dma.TCD->BITER_ELINKNO = sizeof(analog_rx_buffer) / 2;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

	if (adc_to_use == 0)
		dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
	else
		dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);

	update_responsibility = update_setup();
	dma.enable();
	dma.attachInterrupt(isr);
}


void AudioInputAnalog::isr(void)
{
	uint32_t daddr, offset;
	const uint16_t *src, *end;
	uint16_t *dest_left;
	audio_block_t *left;

	daddr = (uint32_t)(dma.TCD->DADDR);
	dma.clearInterrupt();

	if (daddr < (uint32_t)analog_rx_buffer + sizeof(analog_rx_buffer) / 2) {
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = (uint16_t *)&analog_rx_buffer[AUDIO_BLOCK_SAMPLES/2];
		end = (uint16_t *)&analog_rx_buffer[AUDIO_BLOCK_SAMPLES];
		if (update_responsibility) AudioStream::update_all();
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = (uint16_t *)&analog_rx_buffer[0];
		end = (uint16_t *)&analog_rx_buffer[AUDIO_BLOCK_SAMPLES/2];
	}
	left = block_left;
	if (left != NULL) {
		offset = block_offset;
		if (offset > AUDIO_BLOCK_SAMPLES/2) offset = AUDIO_BLOCK_SAMPLES/2;
		dest_left = (uint16_t *)&(left->data[offset]);
		block_offset = offset + AUDIO_BLOCK_SAMPLES/2;
		do {
			*dest_left++ = *src++;
		} while (src < end);
	}
}



void AudioInputAnalog::update(void)
{
	audio_block_t *new_left=NULL, *out_left=NULL;
	unsigned int dc, offset;
	int32_t tmp;
	int16_t s, *p, *end;

	//Serial.println("update");

	// allocate new block (ok if NULL)
	new_left = allocate();

	__disable_irq(); 	//	 Do not allow interupts
	offset = block_offset;
	if (offset < AUDIO_BLOCK_SAMPLES) {
		// the DMA didn't fill a block
		if (new_left != NULL) {
			// but we allocated a block
			if (block_left == NULL) {
				// the DMA doesn't have any blocks to fill, so
				// give it the one we just allocated
				block_left = new_left;
				block_offset = 0;
				__enable_irq();
	 			  //Serial.println("fail 1"); 	 // debugging 
			} else {
				// the DMA already has blocks, doesn't need this
				__enable_irq();
				release(new_left);
	 			 //Serial.print("fail 2, offset="); 	// Usefull for 
	 			 //Serial.println(offset);			// debugging 
			}
		} else {
			// The DMA didn't fill a block, and we could not allocate
			// memory... the system is likely starving for memory!
			// Sadly, there's nothing we can do.
			__enable_irq();
	 		 //Serial.println("fail 3 :: DMA didn't allow a block fill!"); 	// More debugging
		}
		return;
	}
	// the DMA filled a block, so grab it and get the
	// new block to the DMA, as quickly as possible
	out_left = block_left;
	block_left = new_left;
	block_offset = 0;
	__enable_irq();

	// find and subtract DC offset....
	// TODO: this may not be correct, needs testing with more types of signals
	dc = dc_average;
	p = out_left->data;
	end = p + AUDIO_BLOCK_SAMPLES;
	do {
		tmp = (uint16_t)(*p) - (int32_t)dc;
		s = signed_saturate_rshift(tmp, 16, 0);
		*p++ = s;
		dc += s / 12000;  // slow response, remove DC component
	} while (p < end);
	dc_average = dc;

	// then transmit the AC data
	transmit(out_left);
	release(out_left);
}
