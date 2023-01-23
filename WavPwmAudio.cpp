// WavPwmAudio is a merger of two codebases:
//
// Basic PWM audio configuation & starting API:
// PicoRecPlayAudio - Raspberry Pi Pico Audio Record/Playbak/WAV File
// Copyright (C) 2022 Jason Birch
//
// DMA audio playback strategy (frees the CPU for other tasks):
// https://vanhunteradams.com/Pico/DAC/DMA_DAC.html
//
// Thanks Jason! Thanks Van!
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


#include <Arduino.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/interp.h"
#include "WavPwmAudio.h"



static int wavDataCh = -1;
static int wavCtrlCh = -1;
static unsigned int pwmSlice = 0;
static short * bufPtr;
io_rw_32* interpPtr;
extern short transferBuffer[TRANSFER_BUFF_SIZE];
extern short sampleBuffer[SAMPLE_BUFF_SIZE];

// WavPwmInit() sets up an interrupt every TRANSFER_WINDOW_SIZE output samples,
// and then we refill the transfer buffer with TRANSFER_BUFF_SIZE more samples,
// which is TRANSFER_WINDOW_SIZE * number of channels.
/* volatile static unsigned int sampleBuffCursor = 0; // index into sampleBuffer[] */
void pwm_int_handler(){
  static unsigned int sampleBuffCursor = 0;
  pwm_clear_irq(0); // otherSliceNum
           
  for (int i=0; i<TRANSFER_BUFF_SIZE; i++){
    transferBuffer[i] = sampleBuffer[sampleBuffCursor++];
    if (sampleBuffCursor == SAMPLE_BUFF_SIZE)
      sampleBuffCursor = 0;
  }
} 

void WavPwmInit(unsigned char GpioPinChannelA)
{
	//////////////////////////////
	// gpio pin setup for PWM
   gpio_set_function(GpioPinChannelA, GPIO_FUNC_PWM);
   gpio_set_function(GpioPinChannelA + 1, GPIO_FUNC_PWM);

	 // gpio_set_drive_strength(GpioPinChannelA, GPIO_DRIVE_STRENGTH_12MA);
	 // gpio_set_drive_strength(GpioPinChannelA + 1, GPIO_DRIVE_STRENGTH_4MA); // no audible difference.  May be a thing for MIDI mode?

	 // Played with these to try to trim overshoot of a square wave, saw no major change
	 // gpio_set_slew_rate(GpioPinChannelA, GPIO_SLEW_RATE_SLOW);
	 // gpio_set_slew_rate(GpioPinChannelA + 1, GPIO_SLEW_RATE_SLOW);

	 ////////////////////////////
	 // Set up two PWM slices

	 // pwmSlice converts samples to PWM audio on gpio pins
   pwmSlice = pwm_gpio_to_slice_num(GpioPinChannelA);
	 //
	 // triggerSlice generates an interrupt in sync with that one,
	 // but is scaled to only once per TRANSFER_BUFF_SIZE samples.
	 // It triggers a refill of the transfer buffer.  It is just
	 // for IRQs & isn't connected to any pins.
	 //
	 // use slice 0 here, which corresponds to gpio 0&1 or 16&17,
	 // which I happen to know I'm not using in V6.

	 // halt them both:
	 pwm_set_enabled(pwmSlice, false);
	 pwm_set_enabled(TRIGGER_SLICE, false);

	 // initialize them:
	 pwm_config pCfg = pwm_get_default_config();
   pwm_config_set_wrap(&pCfg, WAV_PWM_COUNT);
	 pwm_init(pwmSlice, &pCfg, false);
	 pwm_set_irq_enabled(pwmSlice, false);

	 pwm_config tCfg = pwm_get_default_config();
   pwm_config_set_clkdiv_int_frac(&tCfg, (int)TRANSFER_WINDOW_SIZE, 0);
   pwm_config_set_wrap(&tCfg, WAV_PWM_COUNT);
	 pwm_init(TRIGGER_SLICE, &tCfg, false);
	 pwm_set_irq_enabled(TRIGGER_SLICE, true);
	irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_int_handler);
	irq_set_enabled(PWM_IRQ_WRAP, true);


	 // adjust levels:
   pwm_set_both_levels(pwmSlice, 0, 0);
   // pwm_set_both_levels(TRIGGER_SLICE, 0, 0); // not really meaningful but why not?

	 // line them up:
	 pwm_set_counter(pwmSlice, 0);
	 pwm_set_counter(TRIGGER_SLICE, 28);  // as measured on scope.

	 // make them go:
	 //pwm_set_mask_enabled((1<<pwmSlice) | (1<<TRIGGER_SLICE) | pwm_hw->en);
	 // turn on DMA first!
}


unsigned char WavPwmIsPlaying()
{
	// This is not perfect: approx 1 cycle per AUDIO_PERIOD seconds is when the loop control DMA resets this DMA,
	// so this DMA channel could be not-busy at that moment.  Odds of this are approx once in 133 megachances ... ultra low.
	// but likely we can't expect multiple calls to dma_channel_is_busy to solve that, because the DMAs are moving
	// targets. How to do this exactly right?
   return dma_channel_is_busy(wavDataCh);
}



void WavPwmStopAudio()
{
   if (wavDataCh && dma_channel_is_busy(wavDataCh)) {
      dma_channel_abort(wavDataCh);
      dma_channel_abort(wavCtrlCh);
	 }
}



unsigned char WavPwmPlayAudio(short buf[], unsigned int bufLen)
{
   unsigned char Result = false;
   dma_channel_config wavDataChConfig, wavCtrlChConfig;
	 bufPtr = &buf[0];
	 interpPtr = &interp0->base[1];

   if (wavDataCh < 0) { // if uninitialized
		 Serial.println("getting dma");
		 Serial.flush();
     wavDataCh = dma_claim_unused_channel(true);
	 }
   if (wavCtrlCh < 0) { // if uninitialized
		 Serial.println("getting dma");
		 Serial.flush();
     wavCtrlCh = dma_claim_unused_channel(true);
	 }

	 Serial.printf("pwm dma channel %d\n", wavDataCh);
	 Serial.printf("loop dma channel %d\n", wavCtrlCh);
	 Serial.printf("pwm slice num %d\n", pwmSlice);
	 Serial.flush();

  /*********************************************/
 /* Stop playing audio if DMA already active. */
/*********************************************/
   WavPwmStopAudio();

  /****************************************************/
 /* Don't start playing audio if DMA already active. */
/****************************************************/
   if (!dma_channel_is_busy(wavDataCh))
   {
      Result = true;

	//////
	// configure loop DMA channel, which resets the WAV DMA channel start address, then chains to it.
	// ( https://vanhunteradams.com/Pico/DAC/DMA_DAC.html )
	
      wavCtrlChConfig = dma_channel_get_default_config(wavCtrlCh);
      // channel_config_set_irq_quiet(&wavCtrlChConfig, true); // why?
      channel_config_set_read_increment(&wavCtrlChConfig, false);
      channel_config_set_write_increment(&wavCtrlChConfig, false);
      channel_config_set_transfer_data_size(&wavCtrlChConfig, DMA_SIZE_32);
			channel_config_set_chain_to(&wavCtrlChConfig, wavDataCh); // chain to the wav PWM channel when finished.
			dma_channel_configure(
        wavCtrlCh,                          // Channel to be configured
        &wavCtrlChConfig,                                 // The configuration we just created
        &dma_hw->ch[wavDataCh].read_addr,   // Write address (wav PWM channel read address)
        &bufPtr,                   // Read address (POINTER TO AN ADDRESS) ... contains the address that this DMA writes to the other DMA's read-address.
        1,                                  // transfer 32 bits one time.
        false                               // Don't start immediately
			);

  /****************************************************/
 /* Configure state machine DMA from WAV PWM memory. */
/****************************************************/
      wavDataChConfig = dma_channel_get_default_config(wavDataCh);
      // channel_config_set_irq_quiet(&wavDataChConfig, true); // why?
      channel_config_set_read_increment(&wavDataChConfig, true);
      channel_config_set_write_increment(&wavDataChConfig, false);
      channel_config_set_transfer_data_size(&wavDataChConfig, DMA_SIZE_32); // 32 bytes at a time (l & r 16-bit samples)
      channel_config_set_dreq(&wavDataChConfig, pwm_get_dreq(pwmSlice)); // let PWM cycle request transfers
			channel_config_set_chain_to(&wavDataChConfig, wavCtrlCh); // chain to the loop-control channel when finished.
      dma_channel_configure(
					wavDataCh,  // channel to config
					&wavDataChConfig,  // this configuration
					(void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * pwmSlice)),  // write to pwm channel (pwm structures are 0x14 bytes wide)
					//&interp0->base[1],
					buf, 							// read from here (this value will be overwritten if we start the other loop first)
					bufLen / 2, 			// transfer exactly (samples/2) times (cuz 2 samples per transfer)
					false);

  /**********************/
 /* Start WAV PWM DMA. */
/**********************/
      dma_start_channel_mask(1 << wavCtrlCh);

			// start the PWM to generate the DREQ signals for the DMA:
		 pwm_set_mask_enabled((1<<pwmSlice) | (1<<TRIGGER_SLICE) | pwm_hw->en);
   }

   return Result;
}
