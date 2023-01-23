// TODO: license
// (See WavPwmAudio.cpp)

#ifndef __WAV_PWM_AUDIO_H
#define __WAV_PWM_AUDIO_H


// //#define WAV_SAMPLE_RATE          22048 // mult of 16
//
// // WAV_PWM_COUNT = mcu frequency / sample rate = how many possible PWM values (bit depth basically)
// // TODO: poll the mcu to know its clock frequency? this assumes 133mhz:
// #define WAV_PWM_COUNT            (133000000 / WAV_SAMPLE_RATE)

#define WAV_PWM_SCALE 1 // the tradeoff btwn bit depth & sample rate. 1 = 10 bit, 2 = 11 bit ...
#define WAV_PWM_COUNT ((1024 * WAV_PWM_SCALE) - 1) 
//#define WAV_PWM_COUNT 8191 // 13-bit 
//#define WAV_PWM_COUNT 4095 // 12-bit 
//#define WAV_PWM_COUNT 2047 // 11-bit 
//#define WAV_PWM_COUNT 1023 // 10-bit 
#define WAV_SAMPLE_RATE  (133000000 / WAV_PWM_COUNT) // approx 32471hz for 12-bit.

void WavPwmInit(unsigned char GpioPinChannelA);
unsigned char WavPwmIsPlaying();
void WavPwmStopAudio();
unsigned char WavPwmPlayAudio(short sampleBuf[], unsigned int sampleBufLen);


// Sample buffer: 2 channels for 2 seconds @ 22050 Hz Samples/Second.
// (It's remarkable how long a white noise sample has to be before you can't detect some
// looping artifact.  Longer than 2 seconds, for sure.)
#define AUDIO_CHANNELS                 2
#define AUDIO_PERIOD                   1
// #define SAMPLE_BUFF_SIZE                (AUDIO_CHANNELS * WAV_SAMPLE_RATE * AUDIO_PERIOD)
#define SAMPLE_SIZE 									2 //bytes

// Core1 will use interpolator to scale samples from the sample buffer into this buffer,
// and then DMA will transfer from this buffer to the PWM.
#define TRANSFER_WINDOW_SIZE  				32
#define TRANSFER_BUFF_SIZE  					TRANSFER_WINDOW_SIZE * AUDIO_CHANNELS // 64 samples
//#define SAMPLE_BUFF_SIZE 	( TRANSFER_WINDOW_SIZE * 1010 ) // 32320
//#define SAMPLE_BUFF_SIZE 	( TRANSFER_WINDOW_SIZE * 40 )
#define SAMPLE_BUFF_SIZE 	( TRANSFER_WINDOW_SIZE * (80 / WAV_PWM_SCALE) )

// otherwise unused pwm slice that we can make a timer from:
#define TRIGGER_SLICE 0

#endif

