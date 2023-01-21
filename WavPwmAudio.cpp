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
#include "WavPwmAudio.h"



static int wavPwmDmaCh = -1;
static int loopCtrlDmaCh = -1;
static unsigned int PwmSliceNum = 0;
static unsigned short * address_pointer;



void WavPwmInit(unsigned char GpioPinChannelA)
{
  /*************************/
 /* Configure PWM output. */
/*************************/
   gpio_set_function(GpioPinChannelA, GPIO_FUNC_PWM);
	 // gpio_set_drive_strength(GpioPinChannelA, GPIO_DRIVE_STRENGTH_12MA);
   gpio_set_function(GpioPinChannelA + 1, GPIO_FUNC_PWM);
	 // gpio_set_drive_strength(GpioPinChannelA + 1, GPIO_DRIVE_STRENGTH_4MA); // no audible difference
   PwmSliceNum = pwm_gpio_to_slice_num(GpioPinChannelA);
   pwm_set_wrap(PwmSliceNum, WAV_PWM_COUNT);
   pwm_set_chan_level(PwmSliceNum, PWM_CHAN_A, 0);
   pwm_set_chan_level(PwmSliceNum, PWM_CHAN_B, 0);
   pwm_set_enabled(PwmSliceNum, true);
}



unsigned char WavPwmIsPlaying()
{
	// This is not perfect: approx 1 cycle per AUDIO_PERIOD seconds is when the loop control DMA resets this DMA,
	// so this DMA channel could be not-busy at that moment.  Odds of this are approx once in 133 megachances ... ultra low.
	// but likely we can't expect multiple calls to dma_channel_is_busy to solve that, because the DMAs are moving
	// targets. How to do this exactly right?
   return dma_channel_is_busy(wavPwmDmaCh);
}



void WavPwmStopAudio()
{
   if (wavPwmDmaCh && dma_channel_is_busy(wavPwmDmaCh)) {
      dma_channel_abort(wavPwmDmaCh);
      dma_channel_abort(loopCtrlDmaCh);
	 }
}



unsigned char WavPwmPlayAudio(unsigned short WavPwmData[])
{
   unsigned char Result = false;
   dma_channel_config wavPwmDmaChConfig, loopCtrlDmaChConfig;
	 address_pointer = &WavPwmData[0];

   if (wavPwmDmaCh < 0) { // if uninitialized
		 Serial.println("getting dma");
		 Serial.flush();
     wavPwmDmaCh = dma_claim_unused_channel(true);
	 }
   if (loopCtrlDmaCh < 0) { // if uninitialized
		 Serial.println("getting dma");
		 Serial.flush();
     loopCtrlDmaCh = dma_claim_unused_channel(true);
	 }

	 Serial.printf("pwm dma channel %d\n", wavPwmDmaCh);
	 Serial.printf("loop dma channel %d\n", loopCtrlDmaCh);
	 Serial.printf("pwm slice num %d\n", PwmSliceNum);
	 Serial.flush();

  /*********************************************/
 /* Stop playing audio if DMA already active. */
/*********************************************/
   // WavPwmStopAudio();

  /****************************************************/
 /* Don't start playing audio if DMA already active. */
/****************************************************/
   if (!dma_channel_is_busy(wavPwmDmaCh))
   {
      Result = true;

	//////
	// configure loop DMA channel, which resets the WAV DMA channel start address, then chains to it.
	// ( https://vanhunteradams.com/Pico/DAC/DMA_DAC.html )
	
      loopCtrlDmaChConfig = dma_channel_get_default_config(loopCtrlDmaCh);
      // channel_config_set_irq_quiet(&loopCtrlDmaChConfig, true); // why?
      channel_config_set_read_increment(&loopCtrlDmaChConfig, false);
      channel_config_set_write_increment(&loopCtrlDmaChConfig, false);
      channel_config_set_transfer_data_size(&loopCtrlDmaChConfig, DMA_SIZE_32);
			channel_config_set_chain_to(&loopCtrlDmaChConfig, wavPwmDmaCh); // chain to the wav PWM channel when finished.
			dma_channel_configure(
        loopCtrlDmaCh,                          // Channel to be configured
        &loopCtrlDmaChConfig,                                 // The configuration we just created
        &dma_hw->ch[wavPwmDmaCh].read_addr,   // Write address (wav PWM channel read address)
        &address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // transfer 32 bits one time.
        false                               // Don't start immediately
			);

  /****************************************************/
 /* Configure state machine DMA from WAV PWM memory. */
/****************************************************/
      wavPwmDmaChConfig = dma_channel_get_default_config(wavPwmDmaCh);
      // channel_config_set_irq_quiet(&wavPwmDmaChConfig, true); // why?
      channel_config_set_read_increment(&wavPwmDmaChConfig, true);
      channel_config_set_write_increment(&wavPwmDmaChConfig, false);
      channel_config_set_transfer_data_size(&wavPwmDmaChConfig, DMA_SIZE_32); // 32 bytes at a time (l & r 16-bit samples)
      channel_config_set_dreq(&wavPwmDmaChConfig, pwm_get_dreq(PwmSliceNum)); // let PWM cycle request transfers
			channel_config_set_chain_to(&wavPwmDmaChConfig, loopCtrlDmaCh); // chain to the loop-control channel when finished.
      dma_channel_configure(
					wavPwmDmaCh,  // channel to config
					&wavPwmDmaChConfig,  // this configuration
					(void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * PwmSliceNum)),  // write to pwm channel (pwm structures are 0x14 bytes wide)
					WavPwmData, 							// begin reading from here (but this will be overwritten if we start the other loop first)
					AUDIO_BUFF_SIZE / 2, 			
					false);


  /**********************/
 /* Start WAV PWM DMA. */
/**********************/
      //dma_hw->ints0 = ( (1 << wavPwmDmaCh) & (1 << loopCtrlDmaCh) );
      //dma_start_channel_mask(1 << wavPwmDmaCh);
      dma_start_channel_mask(1 << loopCtrlDmaCh);
   }

   return Result;
}
