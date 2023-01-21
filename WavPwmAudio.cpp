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
static int pwmDataCh = -1;
static int pwmCtrlCh = -1;
static unsigned int PwmSliceNum = 0;
static short * wavTablePtr, *testBufPtr;
io_rw_32* interpPtr;



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
   return dma_channel_is_busy(wavDataCh);
}



void WavPwmStopAudio()
{
   if (wavDataCh && dma_channel_is_busy(wavDataCh)) {
      dma_channel_abort(wavDataCh);
      dma_channel_abort(wavCtrlCh);
      dma_channel_abort(pwmDataCh);
      dma_channel_abort(pwmCtrlCh);
	 }
}



unsigned char WavPwmPlayAudio(short WavTable[])
{
   unsigned char Result = false;
   dma_channel_config wavDataChConfig, wavCtrlChConfig, pwmDataChConfig, pwmCtrlChConfig;
	 wavTablePtr = &WavTable[0];
	 short testBuf[2];
	 testBufPtr = &testBuf[0];
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
   if (pwmDataCh < 0) { // if uninitialized
		 Serial.println("getting dma");
		 Serial.flush();
     pwmDataCh = dma_claim_unused_channel(true);
	 }
   if (pwmCtrlCh < 0) { // if uninitialized
		 Serial.println("getting dma");
		 Serial.flush();
     pwmCtrlCh = dma_claim_unused_channel(true);
	 }

	 Serial.printf("pwm dma channel %d\n", wavDataCh);
	 Serial.printf("loop dma channel %d\n", wavCtrlCh);
	 Serial.printf("pwm dma channel %d\n", pwmDataCh);
	 Serial.printf("loop dma channel %d\n", pwmCtrlCh);
	 Serial.printf("pwm slice num %d\n", PwmSliceNum);
	 Serial.flush();

  /*********************************************/
 /* Stop playing audio if DMA already active. */
/*********************************************/
   // WavPwmStopAudio();

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
        &wavTablePtr,                   // Read address (POINTER TO AN ADDRESS)
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
      channel_config_set_dreq(&wavDataChConfig, pwm_get_dreq(PwmSliceNum)); // let PWM cycle request transfers
			channel_config_set_chain_to(&wavDataChConfig, wavCtrlCh); // chain to the loop-control channel when finished.
      dma_channel_configure(
					wavDataCh,  // channel to config
					&wavDataChConfig,  // this configuration
					//(void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * PwmSliceNum)),  // write to pwm channel (pwm structures are 0x14 bytes wide)
					&testBuf, // write to
					//&interp0->base[1],
					WavTable, 							// read from here (this value will be overwritten if we start the other loop first)
					AUDIO_BUFF_SIZE / 2, 			
					false);

			//////
			// pwm control channel
      pwmCtrlChConfig = dma_channel_get_default_config(pwmCtrlCh);
      // channel_config_set_irq_quiet(&pwmCtrlChConfig, true); // why?
      channel_config_set_read_increment(&pwmCtrlChConfig, false);
      channel_config_set_write_increment(&pwmCtrlChConfig, false);
      channel_config_set_transfer_data_size(&pwmCtrlChConfig, DMA_SIZE_32);
			channel_config_set_chain_to(&pwmCtrlChConfig, pwmDataCh); // chain to the pwm data channel when finished.
			dma_channel_configure(
        pwmCtrlCh,                          // Channel to be configured
        &pwmCtrlChConfig,                                 // The configuration we just created
        &dma_hw->ch[pwmDataCh].read_addr,   // Write address (pwm PWM channel read address)
        //&wavTablePtr,                   // Read address (POINTER TO AN ADDRESS)
        //&interpPtr,                   // Read address (POINTER TO AN ADDRESS)
        &testBufPtr,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // transfer 32 bits one time.
        false                               // Don't start immediately
			);

			// pwm data channel ...
      pwmDataChConfig = dma_channel_get_default_config(pwmDataCh);
      // channel_config_set_irq_quiet(&pwmDataChConfig, true); // why?
      channel_config_set_read_increment(&pwmDataChConfig, false);
      channel_config_set_write_increment(&pwmDataChConfig, false);
      channel_config_set_transfer_data_size(&pwmDataChConfig, DMA_SIZE_32); // 32 bytes at a time (l & r 16-bit samples)
      channel_config_set_dreq(&pwmDataChConfig, pwm_get_dreq(PwmSliceNum)); // let PWM cycle request transfers
			channel_config_set_chain_to(&pwmDataChConfig, pwmCtrlCh); // chain to the loop-control channel when finished.
      dma_channel_configure(
					pwmDataCh,  // channel to config
					&pwmDataChConfig,  // this configuration
					//&testBuf,  	// write to
					(void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * PwmSliceNum)),  // write to pwm channel (pwm structures are 0x14 bytes wide)
					//WavTable, 							// begin reading from here (but this will be overwritten if we start the other loop first)
					&testBuf, // read from
					//&interp0->base[1], // read from
					AUDIO_BUFF_SIZE / 2, 			
					false);


  /**********************/
 /* Start WAV PWM DMA. */
/**********************/
      //dma_hw->ints0 = ( (1 << wavDataCh) & (1 << wavCtrlCh) );
      dma_start_channel_mask( (1 << wavDataCh) | (1<<pwmDataCh) );
      //dma_start_channel_mask(1 << wavCtrlCh);
   }

   return Result;
}
