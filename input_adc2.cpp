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

#include "input_adc2.h"
#include "utility/pdb.h"
#include "utility/dspinst.h"

#define PDB0_CH1C1              *(volatile uint32_t *)0x40036038 // Channel 1 Control Register 1
#define PDB0_CH1S               *(volatile uint32_t *)0x4003603C // Channel 1 Status Register
#define PDB0_CH1DLY0            *(volatile uint32_t *)0x40036040 // Channel 1 Delay 0 Register
#define PDB0_CH1DLY1            *(volatile uint32_t *)0x40036044 // Channel 1 Delay 1 Register


DMAMEM static uint16_t analog_rx_buffer2[AUDIO_BLOCK_SAMPLES];

//#define DMAMEM __attribute__ ((section(".dmabuffers"), used))

audio_block_t * AudioInputAnalog2::block_left = NULL;

/*
*typedef struct audio_block_struct {
*	unsigned char ref_count;
*	unsigned char memory_pool_index;
*	unsigned char reserved1;
*	unsigned char reserved2;
*	int16_t data[AUDIO_BLOCK_SAMPLES];
*} audio_block_t;
*/

uint16_t AudioInputAnalog2::block_offset = 0;
//Class::Var
uint16_t AudioInputAnalog2::dc_average = 0;
bool AudioInputAnalog2::update_responsibility = false;
// class DMAChannel : public DMABaseClass {  //..cores/teensy3/DMAChannel.h
DMAChannel AudioInputAnalog2::myDMA(false);
//ADC AudioInputAnalog2::myADC;

void AudioInputAnalog2::init(uint8_t pin)
{
	//Serial.begin(9600);
	//Serial.println("Entered init");
	uint32_t i, sum=0;

	// Configure the ADC and run at least one software-triggered
	// conversion.  This completes the self calibration stuff and
	// leaves the ADC in a state that's mostly ready to use
	//myADC.setResolution(16);
	//myADC.setReference(INTERNAL); // range 0 to 1.2 volts
	//myADC.setAveraging(8);
	
	analogReadRes(16);
	analogReference(INTERNAL); // range 0 to 1.2 volts
	analogReadAveraging(8);

	// Actually, do many normal reads, to start with a nice DC level
	for (i=0; i < 1024; i++) {
		//sum += myADC.analogRead(pin, 1);
		sum += analogRead(pin);
	}
	dc_average = sum >> 10;
	//Serial.println("finished analogReadAveraging");

//#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_CONT | PDB_SC_PDBIE | PDB_SC_DMAEN)
//#define PDB_SC_TRGSEL(n)		(((n) & 15) << 8)	// Trigger Input Source Select

	// set the programmable delay block to trigger the ADC at 44.1 kHz
	if (!(SIM_SCGC6 & SIM_SCGC6_PDB)
	  || (PDB0_SC & PDB_CONFIG) != PDB_CONFIG
	  || PDB0_MOD != PDB_PERIOD
	  || PDB0_IDLY != 1
	  || PDB0_CH0C1 != 0x0101) {
/*
 TOS = 0x01 | TOS = 0x00 - channels assert when the counter reaches the channel
 delay register plus one peripheral clock cycle after PDB trigger */
//PDB0_CH0C1 |= PDB_C1_EN(0x1) | PDB_C1_TOS(0x1);
/* delete next line if the two-time measurement per PWM period is not required */
//PDB0_CH0C1 |= PDB_C1_EN(0x2) | PDB_C1_TOS(0x2);
//PDB0_CH1C1 |= PDB_C1_EN(0x1) | PDB_C1_TOS(0x1);
//Serial.println("Got here;");
		PDB0_CH0C1 = 0x0101;
/* delete next line if the two-time measurement per PWM period is not required */
//PDB0_CH1C1 |= PDB_C1_EN(0x2) | PDB_C1_TOS(0x2);
/* set the delay value for the channel's corresponding pre-triggers */
//PDB0_CH0DLY0 = 0; /* corresponding ADC0_RA */
//PDB0_CH1DLY0 = 0; /* corresponding ADC1_RA */
/* delete next two lines if the two-time measurement per PWM period is not required */
//PDB0_CH0DLY1 = 1250; /* corresponding ADC0_RB */
//PDB0_CH1DLY1 = 1250; /* corresponding ADC1_RB */
/* PDBEN = 1 - PDB enabled, TRGSEL = 0x8 - FTM0 is a trigger source for PDB,
 LDOK - writing 1 to this bit updates the internal registers*/
//PDB0_SC |= PDB_SC_PDBEN_MASK | PDB_SC_LDOK_MASK; 



		SIM_SCGC6 |= SIM_SCGC6_PDB;
//		SIM_SCGC3 |= SIM_SCGC3_ADC1;
//#define SIM_SCGC6		(*(volatile uint32_t *)0x4004803C) // System Clock Gating Control Register 6
//#define SIM_SCGC6_PDB			((uint32_t)0x00400000)		// PDB Clock Gate Control SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

		PDB0_IDLY = 1;

//#define PDB0_IDLY		(*(volatile uint32_t *)0x4003600C) // Interrupt Delay Register
/*PDB Interrupt Delay
Specifies the delay value to schedule the PDB interrupt. It can be used to schedule an independent
interrupt at some point in the PDB cycle. If enabled, a PDB interrupt is generated, when the counter is
equal to the IDLY. Reading these bits returns the value of internal register that is effective for the current
cycle of the PDB
*/

		PDB0_MOD = PDB_PERIOD;
/*PDB Modulus
Specifies the period of the counter. When the counter reaches this value, it will be reset back to zero. If the
PDB is in Continuous mode, the count begins anew. Reading these bits returns the value of internal
register that is effective for the current cycle of PDB 

*/

		PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;

//#define PDB_SC_LDOK			0x00000001		// Load OK

		PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;

//#define PDB_SC_SWTRIG			0x00010000		// Software Trigger
		//PDB0_CH1C1 = 0x0101;
		PDB0_CH0C1 = 0x0101; //b0000 0000 0000 0001 0000 0001
//#define PDB0_CH0C1		(*(volatile uint32_t *)0x40036010) // Channel n Control Register 1
/*PDB channel's corresponding pre-trigger asserts when the counter reaches the channel delay register
and one peripheral clock cycle after a rising edge is detected on selected trigger input source or
software trigger is selected and SETRIG is written with 1


These bits enable the PDB ADC pre-trigger outputs. Only lower M pre-trigger bits are implemented in this
MCU.
0 PDB channel's corresponding pre-trigger disabled.
1 PDB channel's corresponding pre-trigger enabled.
*/
	}
	//Serial.println("Finished PDB trigger");

	// enable the ADC for hardware trigger and DMA
	//ADC0_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;
	

ADC1_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;
	// set up a DMA channel to store the ADC data
	myDMA.begin(true);
	myDMA.TCD->SADDR = &ADC1_RA;
	myDMA.TCD->SOFF = 0;
	myDMA.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
	myDMA.TCD->NBYTES_MLNO = 2;
	myDMA.TCD->SLAST = 0;
	myDMA.TCD->DADDR = analog_rx_buffer2;
	myDMA.TCD->DOFF = 2;
	myDMA.TCD->CITER_ELINKNO = sizeof(analog_rx_buffer2) / 2;
	myDMA.TCD->DLASTSGA = -sizeof(analog_rx_buffer2);
	myDMA.TCD->BITER_ELINKNO = sizeof(analog_rx_buffer2) / 2;
	myDMA.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
	myDMA.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);
	update_responsibility = update_setup();
	myDMA.enable();
	myDMA.attachInterrupt(isr);

	//Serial.println("Finished DMA setup");
}


void AudioInputAnalog2::isr(void)
{
	uint32_t daddr, offset;
	const uint16_t *src, *end;
	uint16_t *dest_left;
	audio_block_t *left;

	daddr = (uint32_t)(myDMA.TCD->DADDR);
	myDMA.clearInterrupt();

	if (daddr < (uint32_t)analog_rx_buffer2 + sizeof(analog_rx_buffer2) / 2) {
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = (uint16_t *)&analog_rx_buffer2[AUDIO_BLOCK_SAMPLES/2];
		end = (uint16_t *)&analog_rx_buffer2[AUDIO_BLOCK_SAMPLES];
		if (update_responsibility) AudioStream::update_all();
	} else {
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = (uint16_t *)&analog_rx_buffer2[0];
		end = (uint16_t *)&analog_rx_buffer2[AUDIO_BLOCK_SAMPLES/2];
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



void AudioInputAnalog2::update(void)
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
	 			  //Serial.println("ADC1 fail 1"); 	 // debugging 
			} else {
				// the DMA already has blocks, doesn't need this
				__enable_irq();
				release(new_left);
	 			 //Serial.print("ADC1 fail 2, offset="); 	// Usefull for 
	 			 //Serial.println(offset);			// debugging 
			}
		} else {
			// The DMA didn't fill a block, and we could not allocate
			// memory... the system is likely starving for memory!
			// Sadly, there's nothing we can do.
			__enable_irq();
	 		 //Serial.println("ADC1 fail 3 :: DMA didn't allow a block fill!"); 	// More debugging
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



