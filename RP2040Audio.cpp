// RP2040Audio learns from two codebases:
//
// Basic PWM audio configuation:
// PicoRecPlayAudio - Raspberry Pi Pico Audio Record/Playbak/WAV File
// Copyright (C) 2022 Jason Birch
//
// DMA audio playback strategy (frees the CPU for other tasks):
// https://vanhunteradams.com/Pico/DAC/DMA_DAC.html
//
// Thanks Jason! Thanks Van!
//
//

#include "config.h"

// don't compile this for Teensy:
#ifdef TARGET_RP2040

#include <Arduino.h> // for Serial
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/interp.h"
#include "pins.h"
#include "RP2040Audio.h"

// C++, why you can't just read the header file jeez ...
extern int RP2040Audio::wavDataCh[2];
extern int RP2040Audio::wavCtrlCh[2];
extern unsigned int RP2040Audio::pwmSlice[2];
extern short* RP2040Audio::bufPtr[2];


void setup_interp1_clamp(){
		interp_config cfg = interp_default_config();
    interp_config_set_clamp(&cfg, true);
    interp_config_set_signed(&cfg, true);
    interp_set_config(interp1, 0, &cfg);

		interp1->base[0] = 0 - (WAV_PWM_RANGE / 2);;
    interp1->base[1] = (WAV_PWM_RANGE / 2) -1;
}

// This gets called once at startup to set up both stereo PWMs for both ports
void RP2040Audio::init() {
	/////////////////////////
	// set up interp1 for clamping (used by ISR)
	//
	setup_interp1_clamp();

  ////////////////////////////
  // Set up PWM slices

  // pwmSlice converts samples to PWM audio on gpio pins
  pwmSlice[0] = pwm_gpio_to_slice_num(ring1);
  pwmSlice[1] = pwm_gpio_to_slice_num(ring2);

  // triggerSlice generates an interrupt in sync with that one,
  // but is scaled to only once per TRANSFER_BUFF_SAMPLES samples.
  // It triggers a refill of the transfer buffer.  It is just
  // for IRQs & isn't connected to any pins.
  //
  // use slice 0 here, which corresponds to gpio 0&1 or 16&17,
  // which I happen to know I'm not using.

  // halt:
	pwm_set_enabled(pwmSlice[0], false);
	pwm_set_enabled(pwmSlice[1], false);
  pwm_set_enabled(TRIGGER_SLICE, false);


  // initialize:
  pwm_config pCfg;
  for (int i = 0; i < 2; i++) {
    pCfg = pwm_get_default_config();
    pwm_config_set_wrap(&pCfg, WAV_PWM_COUNT);
    pwm_init(pwmSlice[i], &pCfg, false);
    pwm_set_irq_enabled(pwmSlice[i], false);
  }

  pwm_config tCfg = pwm_get_default_config();
  pwm_config_set_clkdiv_int_frac(&tCfg, (int)TRANSFER_WINDOW_XFERS, 0);
  pwm_config_set_wrap(&tCfg, WAV_PWM_COUNT);
  pwm_init(TRIGGER_SLICE, &tCfg, false);
  pwm_set_irq_enabled(TRIGGER_SLICE, true);
  irq_set_enabled(PWM_IRQ_WRAP, true);


  // line them up & adjust levels
  for (int i = 0; i < 2; i++) {
		pwm_set_both_levels(pwmSlice[i], 0, 0);
    pwm_set_counter(pwmSlice[i], 0);
	}
  pwm_set_counter(TRIGGER_SLICE, 28);  // as measured on scope.

  // don't make them go yet. Turn on DMA first!
  //pwm_set_mask_enabled((1<<pwmSlice) | (1<<TRIGGER_SLICE) | pwm_hw->en);
}


bool RP2040Audio::isPlaying(unsigned char port) {
  // This is not perfect: approx 1 cycle per AUDIO_PERIOD seconds is when the loop control DMA resets this DMA,
  // so this DMA channel could be not-busy at that moment.  Odds of this are approx once in 133 megachances ... ultra low.
  // but likely we can't expect multiple calls to dma_channel_is_busy to solve that, because the DMAs are moving
  // targets. How to do this exactly right?
  return dma_channel_is_busy(wavDataCh[port]);
}


void RP2040Audio::pause(unsigned char port) {
  if (wavDataCh[port] >= 0 && dma_channel_is_busy(wavDataCh[port])) {
    dma_channel_abort(wavDataCh[port]);
    dma_channel_abort(wavCtrlCh[port]);
    pwm_set_enabled(pwmSlice[port], false);
  }
}

void RP2040Audio::play(unsigned char port) {
  dma_channel_config wavDataChConfig, wavCtrlChConfig;
  bufPtr[port] = &(transferBuffer[port][0]);

  if (wavDataCh[port] < 0) {  // if uninitialized
    Serial.println("getting dma");
    Serial.flush();
    wavDataCh[port] = dma_claim_unused_channel(true);
  }
  if (wavCtrlCh[port] < 0) {  // if uninitialized
    Serial.println("getting dma");
    Serial.flush();
    wavCtrlCh[port] = dma_claim_unused_channel(true);
  }

  Serial.printf("pwm dma channel %d\n", wavDataCh[port]);
  Serial.printf("loop dma channel %d\n", wavCtrlCh[port]);
  Serial.printf("pwm slice num %d\n", pwmSlice[port]);
  Serial.flush();

  /*********************************************/
  /* Stop playing audio if DMA already active. */
  /*********************************************/
  this->pause(port);

  /****************************************************/
  /* Don't start playing audio if DMA already active. */
  /****************************************************/
  if (!dma_channel_is_busy(wavDataCh[port])) {

    //////
    // configure loop DMA channel, which resets the WAV DMA channel start address, then chains to it.
    // ( https://vanhunteradams.com/Pico/DAC/DMA_DAC.html )

    wavCtrlChConfig = dma_channel_get_default_config(wavCtrlCh[port]);
    channel_config_set_read_increment(&wavCtrlChConfig, false);
    channel_config_set_write_increment(&wavCtrlChConfig, false);
    channel_config_set_transfer_data_size(&wavCtrlChConfig, DMA_SIZE_32);
    channel_config_set_chain_to(&wavCtrlChConfig, wavDataCh[port]);  // chain to the wav PWM channel when finished.
    dma_channel_configure(
      wavCtrlCh[port],                         // Channel to be configured
      &wavCtrlChConfig,                        // The configuration we just created
      &dma_hw->ch[wavDataCh[port]].read_addr,  // Write address (wav PWM channel read address)
      &bufPtr[port],                                 // Read address (POINTER TO AN ADDRESS) ... contains the address that this DMA writes to the other DMA's read-address.
      1,                                       // transfer 32 bits one time.
      false                                    // Don't start immediately
    );

    /****************************************************/
    /* Configure state machine DMA from WAV PWM memory. */
    /****************************************************/
    wavDataChConfig = dma_channel_get_default_config(wavDataCh[port]);
    channel_config_set_read_increment(&wavDataChConfig, true);
    channel_config_set_write_increment(&wavDataChConfig, false);
    channel_config_set_transfer_data_size(&wavDataChConfig, DMA_SIZE_32);     // 32 bytes at a time (l & r 16-bit samples)
    channel_config_set_dreq(&wavDataChConfig, pwm_get_dreq(pwmSlice[port]));  // let PWM cycle request transfers
    channel_config_set_chain_to(&wavDataChConfig, wavCtrlCh[port]);           // chain to the loop-control channel when finished.
    dma_channel_configure(
      wavDataCh[port],                                                  // channel to config
      &wavDataChConfig,                                                 // this configuration
      (void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * pwmSlice[port])),  // write to pwm channel (pwm structures are 0x14 bytes wide)
      transferBuffer[port],                                                   // read from here (this value will be overwritten if we start the other loop first)
      TRANSFER_BUFF_SAMPLES / 2,                                           // transfer exactly (samples/2) times (cuz 2 samples per transfer)
      false);

    /**********************/
    /* Start WAV PWM DMA. */
    /**********************/
    dma_start_channel_mask(1 << wavCtrlCh[port]);

    // start the PWM to generate the DREQ signals for the DMA:
    pwm_set_mask_enabled((1 << pwmSlice[port]) | (1 << TRIGGER_SLICE) | pwm_hw->en);
  }
}


// constructor/initalizer cuz c++ is weird about this
RP2040Audio::RP2040Audio() {
  wavDataCh[0] = wavDataCh[1] = -1;
  wavCtrlCh[0] = wavCtrlCh[1] = -1;
  pwmSlice[0] = pwmSlice[1] = 0;
}

extern volatile uint32_t iVolumeLevel;

// init() sets up an interrupt every TRANSFER_WINDOW_XFERS output samples,
// then this ISR refills the transfer buffer with TRANSFER_BUFF_SAMPLES more samples,
// which is TRANSFER_WINDOW_XFERS * number of channels.
void RP2040Audio::ISR_play() {
  static unsigned int sampleBuffCursor = 0;
  pwm_clear_irq(TRIGGER_SLICE);

  for (int i = 0; i < TRANSFER_BUFF_SAMPLES; i++) {

    // Since amplitude can go over max, use interpolator #1 in clamp mode
		// to hard-limit the signal.
    interp1->accum[0] = 
											(short)( 
													(long) (
														sampleBuffer[sampleBuffCursor++]
														 * iVolumeLevel  // scale numerator (can be from 0 to more than WAV_PWM_RANGE
														)
                         / WAV_PWM_RANGE      // scale denominator (TODO right shift here? or is the compiler smart?)
                         )
      ;
			// TODO: set up interp0 to perform this add?
		transferBuffer[0][i] = transferBuffer[1][i] = interp1->peek[0] + (WAV_PWM_RANGE / 2);  // shift to positive

    if (sampleBuffCursor == SAMPLE_BUFF_SAMPLES)
      sampleBuffCursor = 0;
  }
}

// Fancier version, time-scaling a 1hz sample:
extern volatile float sampleCursorInc[4];
void RP2040Audio::ISR_test() {
  static float sampleBuffCursor[4] = {0,0,0,0};
  pwm_clear_irq(TRIGGER_SLICE);

	for (uint8_t port = 0; port<2; port++) {
		for (int i = 0; i < TRANSFER_BUFF_SAMPLES; i+=2) {
			for (uint8_t chan = 0; chan < 2; chan++){
				uint8_t pc = (port<<1)+chan;

				transferBuffer[port][i+chan] = sampleBuffer[(unsigned int)sampleBuffCursor[pc]] ;
						// + (WAV_PWM_RANGE / 2);  // not shifting! we expect a positive-weighted sample (true flag passed to fillWithSine)

				// advance cursor:
				sampleBuffCursor[pc] += sampleCursorInc[pc];
				while (sampleBuffCursor[pc] >= SAMPLE_BUFF_SAMPLES)
					sampleBuffCursor[pc] -= SAMPLE_BUFF_SAMPLES;
			}
		}
	}

}

//////////
//
// These basic utils generate signals in the sampleBuffer.
// In every case it's signed values between -(WAV_PWM_RANGE/2) 
// and WAV_PWM_COUNT
//
// fill buffer with white noise (signed)
void RP2040Audio::fillWithNoise(){
	randomSeed(666);
	for(int i=0; i<SAMPLE_BUFF_SAMPLES; i++){
		sampleBuffer[i] = random(WAV_PWM_RANGE) - (WAV_PWM_RANGE / 2);
	}
}

// fill buffer with sine waves
void RP2040Audio::fillWithSine(uint count, bool positive){
	const float twoPI = 6.283;
	const float scale = (WAV_PWM_RANGE) / 2;

	for (int i=0; i<SAMPLE_BUFF_SAMPLES; i+= AUDIO_CHANNELS){
		for(int j=0;j<AUDIO_CHANNELS; j++)
			sampleBuffer[i + j] = (int) (scale
					* sin( (float)i * count / (float)SAMPLE_BUFF_SAMPLES * twoPI ) 
				 ) + (positive ? scale : 0) ; // shift sample to positive? (so the ISR routine doesn't have to)
	}
}

// fill buffer with square waves
void RP2040Audio::fillWithSquare(uint count){
	for (int i=0; i<SAMPLE_BUFF_SAMPLES; i+= AUDIO_CHANNELS)
		for(int j=0;j<AUDIO_CHANNELS; j++)
		 if ((i*count)%SAMPLE_BUFF_SAMPLES < (SAMPLE_BUFF_SAMPLES / 2)){ 
			 sampleBuffer[i + j] = (WAV_PWM_RANGE)/ 2;
		 } else {
			 sampleBuffer[i + j] = 0 - ((WAV_PWM_RANGE) / 2);
		 }
}

// fill buffer with sawtooth waves running negative to positive
// (Still slightly buggy ...)
void RP2040Audio::fillWithSaw(uint count){
	const float twoPI = 6.283;
	const float scale = (WAV_PWM_RANGE) / 2;

	for (int i=0; i<SAMPLE_BUFF_SAMPLES; i+= AUDIO_CHANNELS){
		for(int j=0;j<AUDIO_CHANNELS; j++)
			sampleBuffer[i + j] = (int) 

				// i 																																// 0 -> SAMPLE_BUFF_SAMPLES-1
				// i / (SAMPLE_BUFF_SAMPLES - 1) 																			// 0 -> 1
				// i * WAV_PWM_RANGE / (SAMPLE_BUFF_SAMPLES -1) 												// 0 -> WAV_PWM_RANGE
				// (i * count) * WAV_PWM_RANGE / (SAMPLE_BUFF_SAMPLES -1) 							// 0 -> count*WAV_PWM_RANGE
				(i * count * WAV_PWM_RANGE / (SAMPLE_BUFF_SAMPLES -1) ) % WAV_PWM_RANGE // 0 -> WAV_PWM_RANGE, count times
					- (WAV_PWM_RANGE / 2); // shift to 50% negative
	}
}

// PWM tuning utility
// One would call this over and over again in a main loop.
void RP2040Audio::tweak() {
	char c;

	static int position = 0;

	static int step = 50;

	if (Serial.available()) {
		c = Serial.read();
		if (c == '+') {
			// advance
			for (int x = 0; x < step; x++) {
				pwm_advance_count(0);
			}
			position += step;
			Serial.println(position);
		} else if (c == '-') {
			// retard
			for (int x = 0; x < step; x++) {
				pwm_retard_count(0);
			}
			position -= step;
			Serial.println(position);
		} else if (c == '*') {
			// increase step size
			step = step * 2;
			Serial.print('*');
			Serial.println(step);
		} else if (c == '/') {
			// decrease step size
			step = step / 2;
			Serial.print('*');
			Serial.println(step);
		} else {
			Serial.print(c);
		}
	}
}


#endif // TARGET_RP2040
