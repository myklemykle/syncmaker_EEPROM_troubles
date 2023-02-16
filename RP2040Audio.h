// TODO: license
// (See WavPwmAudio.cpp)
//

#ifndef __WAV_PWM_AUDIO_H
#define __WAV_PWM_AUDIO_H


// //#define WAV_SAMPLE_RATE          22048 // mult of 16
//
// // WAV_PWM_COUNT = mcu frequency / sample rate = how many possible PWM values (bit depth basically)
// // TODO: poll the mcu to know its clock frequency? this assumes 133mhz:
// #define WAV_PWM_COUNT            (133000000 / WAV_SAMPLE_RATE)

#define WAV_PWM_SCALE 1                             // the tradeoff btwn bit depth & sample rate. 1 = 10 bit, 2 = 11 bit ... \
                                                    // 10-bit audio has the advantage that the sample rate is up at 130khz, \
                                                    // which means that the HF noise is really getting well-suppressed. \
                                                    // With 12-bit audio, that noise is getting down into the almost-audible \
                                                    // spectrum, which many amplifiers will amplify.  Really I'm not happy \
                                                    // with the output filter performance of V6, tho this won't matter much \
                                                    // because our main output will be white noise. =) \
                                                    // PDM could also improve this, if more sample resolution was needed.
#define WAV_PWM_COUNT ((1024 * WAV_PWM_SCALE) - 1)  // the PWM counter's setting
#define WAV_PWM_RANGE ((1024 * WAV_PWM_SCALE))
#define WAV_SAMPLE_RATE (133000000 / WAV_PWM_RANGE)

// void WavPwmInit();
// unsigned char WavPwmIsPlaying(unsigned char port);
// void WavPwmStopAudio(unsigned char port);
// unsigned char WavPwmPlayAudio(short sampleBuf[], unsigned int sampleBufLen, unsigned char port);

// Sample buffer: 2 channels, because PWM outputs want to be stereo
// (It's remarkable how long a white noise sample has to be before you can't detect some
// looping artifact.  Longer than 2 seconds, for sure.)
#define AUDIO_CHANNELS 2
#define AUDIO_PERIOD 1
#define SAMPLE_SIZE 2  //bytes

// Core1 scales samples from the sample buffer into this buffer,
// while DMA transfers from this buffer to the PWM.
#define TRANSFER_WINDOW_SIZE 8
#define TRANSFER_BUFF_SIZE TRANSFER_WINDOW_SIZE* AUDIO_CHANNELS  // size in uint_16s
//#define SAMPLE_BUFF_SIZE 	( TRANSFER_WINDOW_SIZE * (320 / WAV_PWM_SCALE) )
//
// that's fine for a waveform, but for noise we need a much larger buffer:
#define SAMPLE_BUFF_SIZE (TRANSFER_WINDOW_SIZE * 10000) // about 1.6 secs at WAV_PWM_SCALE = 1

// Here is a spare pwm slice that we can make a timer from:
#define TRIGGER_SLICE 0

class RP2040Audio {
public:
  static short transferBuffer[TRANSFER_BUFF_SIZE];
  static short sampleBuffer[SAMPLE_BUFF_SIZE];

  RP2040Audio();
  static void ISR();
  void init();  // allocate & configure PWM and DMA for both ports
  // void play(short buf[], unsigned int bufLen, unsigned char port); // turn on PWM & DMA
  void play(unsigned char port);   // turn on PWM & DMA
  void pause(unsigned char port);  // halt PWM & DMA
  bool isPlaying(unsigned char port);
	void fillWithNoise();
	void fillWithSine(uint count);
	void fillWithSquare(uint count);
  void tweak();  // adjust the trigger pulse. for debugging purposes only. reads from Serial.
	// TODO:
	// void sleep()
	// void wake()

private:
  static int wavDataCh[2];
  static int wavCtrlCh[2];
  static unsigned int pwmSlice[2];
  static short* bufPtr;
  io_rw_32* interpPtr;
  unsigned short volumeLevel = 0;
};



#endif
