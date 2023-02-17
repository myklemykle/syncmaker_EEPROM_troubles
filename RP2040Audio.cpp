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
extern short* RP2040Audio::bufPtr;


// This gets called once at startup to set up both stereo PWMs for both ports
void RP2040Audio::init() {
  ////////////////////////////
  // Set up PWM slices

  // pwmSlice converts samples to PWM audio on gpio pins
  pwmSlice[0] = pwm_gpio_to_slice_num(ring1);
  pwmSlice[1] = pwm_gpio_to_slice_num(ring2);

  // triggerSlice generates an interrupt in sync with that one,
  // but is scaled to only once per TRANSFER_BUFF_SIZE samples.
  // It triggers a refill of the transfer buffer.  It is just
  // for IRQs & isn't connected to any pins.
  //
  // use slice 0 here, which corresponds to gpio 0&1 or 16&17,
  // which I happen to know I'm not using in V6.

  // halt:
  // TODO: needed? was chasing a bug with this, fixed elsewhere ...
  for (int i = 0; i < 2; i++)
    pwm_set_enabled(pwmSlice[i], false);
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
  pwm_config_set_clkdiv_int_frac(&tCfg, (int)TRANSFER_WINDOW_SIZE, 0);
  pwm_config_set_wrap(&tCfg, WAV_PWM_COUNT);
  pwm_init(TRIGGER_SLICE, &tCfg, false);
  pwm_set_irq_enabled(TRIGGER_SLICE, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, &RP2040Audio::ISR);
  irq_set_enabled(PWM_IRQ_WRAP, true);


  // adjust levels:
  pwm_set_both_levels(pwmSlice[0], 0, 0);

  // line them up:
  for (int i = 0; i < 2; i++)
    pwm_set_counter(pwmSlice[i], 0);
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
  if (wavDataCh[port] && dma_channel_is_busy(wavDataCh[port])) {
    dma_channel_abort(wavDataCh[port]);
    dma_channel_abort(wavCtrlCh[port]);
    pwm_set_enabled(pwmSlice[port], false);
  }
}

void RP2040Audio::play(unsigned char port) {
  dma_channel_config wavDataChConfig, wavCtrlChConfig;
  bufPtr = &transferBuffer[0];

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
    // channel_config_set_irq_quiet(&wavCtrlChConfig, true); // why?
    channel_config_set_read_increment(&wavCtrlChConfig, false);
    channel_config_set_write_increment(&wavCtrlChConfig, false);
    channel_config_set_transfer_data_size(&wavCtrlChConfig, DMA_SIZE_32);
    channel_config_set_chain_to(&wavCtrlChConfig, wavDataCh[port]);  // chain to the wav PWM channel when finished.
    dma_channel_configure(
      wavCtrlCh[port],                         // Channel to be configured
      &wavCtrlChConfig,                        // The configuration we just created
      &dma_hw->ch[wavDataCh[port]].read_addr,  // Write address (wav PWM channel read address)
      &bufPtr,                                 // Read address (POINTER TO AN ADDRESS) ... contains the address that this DMA writes to the other DMA's read-address.
      1,                                       // transfer 32 bits one time.
      false                                    // Don't start immediately
    );

    /****************************************************/
    /* Configure state machine DMA from WAV PWM memory. */
    /****************************************************/
    wavDataChConfig = dma_channel_get_default_config(wavDataCh[port]);
    // channel_config_set_irq_quiet(&wavDataChConfig, true); // why?
    channel_config_set_read_increment(&wavDataChConfig, true);
    channel_config_set_write_increment(&wavDataChConfig, false);
    channel_config_set_transfer_data_size(&wavDataChConfig, DMA_SIZE_32);     // 32 bytes at a time (l & r 16-bit samples)
    channel_config_set_dreq(&wavDataChConfig, pwm_get_dreq(pwmSlice[port]));  // let PWM cycle request transfers
    channel_config_set_chain_to(&wavDataChConfig, wavCtrlCh[port]);           // chain to the loop-control channel when finished.
    dma_channel_configure(
      wavDataCh[port],                                                  // channel to config
      &wavDataChConfig,                                                 // this configuration
      (void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * pwmSlice[port])),  // write to pwm channel (pwm structures are 0x14 bytes wide)
      transferBuffer,                                                   // read from here (this value will be overwritten if we start the other loop first)
      TRANSFER_BUFF_SIZE / 2,                                           // transfer exactly (samples/2) times (cuz 2 samples per transfer)
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

void RP2040Audio::ISR() {
  // WavPwmInit() sets up an interrupt every TRANSFER_WINDOW_SIZE output samples,
  // and then we refill the transfer buffer with TRANSFER_BUFF_SIZE more samples,
  // which is TRANSFER_WINDOW_SIZE * number of channels.
  /* volatile static unsigned int sampleBuffCursor = 0; // index into sampleBuffer[] */
  static unsigned int sampleBuffCursor = 0;
  pwm_clear_irq(TRIGGER_SLICE);

  for (int i = 0; i < TRANSFER_BUFF_SIZE; i++) {

    // TODO: use interps to reduce cpu load here? clamp or blend?
		// TODO: clamp somehow!
    transferBuffer[i] = (sampleBuffer[sampleBuffCursor++]
                         * interp0->accum[1]  // scale numerator
                         / WAV_PWM_RANGE      // scale denominator (TODO right shift here? or is the compiler smart?)
                         )
                        + (WAV_PWM_RANGE / 2)  // shift to positive
      ;

    if (sampleBuffCursor == SAMPLE_BUFF_SIZE)
      sampleBuffCursor = 0;
  }
}

//////////
// fill buffer with white noise (signed)
void RP2040Audio::fillWithNoise(){
  randomSeed(666);
  for(int i=0; i<SAMPLE_BUFF_SIZE; i++){
    sampleBuffer[i] = random(WAV_PWM_RANGE) - (WAV_PWM_RANGE / 2);
  }
}

// fill buffer with sine waves
void RP2040Audio::fillWithSine(uint count){
  const float twoPI = 6.283;
  const float scale = (WAV_PWM_RANGE) / 2;

  for (int i=0; i<SAMPLE_BUFF_SIZE; i+= AUDIO_CHANNELS){
    for(int j=0;j<AUDIO_CHANNELS; j++)
      sampleBuffer[i + j] = (int) (scale
          * sin( (float)i * count / (float)SAMPLE_BUFF_SIZE * twoPI ) 
         );
  }
}

// fill buffer with square waves
void RP2040Audio::fillWithSquare(uint count){
  for (int i=0; i<SAMPLE_BUFF_SIZE; i+= AUDIO_CHANNELS)
    for(int j=0;j<AUDIO_CHANNELS; j++)
     if ((i*count)%SAMPLE_BUFF_SIZE < (SAMPLE_BUFF_SIZE / 2)){ 
       sampleBuffer[i + j] = (WAV_PWM_RANGE)/ 2;
     } else {
       sampleBuffer[i + j] = 0 - ((WAV_PWM_RANGE) / 2);
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
