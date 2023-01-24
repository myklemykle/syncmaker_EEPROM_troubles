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
												// 10-bit audio has the advantage that the sample rate is up at 130khz,
												// which means that the HF noise is really getting well-suppressed.
												// With 12-bit audio, that noise is getting down into the almost-audible
												// spectrum, which many amplifiers will amplify.  Really I'm not happy
												// with the output filter performance of V6, tho this won't matter much
												// because our main output will be white noise. =)
												// PDM could also improve this, if more sample resolution was needed.
#define WAV_PWM_COUNT ((1024 * WAV_PWM_SCALE) - 1) 
#define WAV_SAMPLE_RATE  (133000000 / WAV_PWM_COUNT) 

void WavPwmInit(unsigned char GpioPinChannelA);
unsigned char WavPwmIsPlaying();
void WavPwmStopAudio();
unsigned char WavPwmPlayAudio(short sampleBuf[], unsigned int sampleBufLen);


// Sample buffer: 2 channels for 2 seconds @ 22050 Hz Samples/Second.
// (It's remarkable how long a white noise sample has to be before you can't detect some
// looping artifact.  Longer than 2 seconds, for sure.)
#define AUDIO_CHANNELS                 2
#define AUDIO_PERIOD                   1
#define SAMPLE_SIZE 									2 //bytes

// Core1 will use interpolator to scale samples from the sample buffer into this buffer,
// and then DMA will transfer from this buffer to the PWM.
#define TRANSFER_WINDOW_SIZE  				8
#define TRANSFER_BUFF_SIZE  					TRANSFER_WINDOW_SIZE * AUDIO_CHANNELS // 64 samples
#define SAMPLE_BUFF_SIZE 	( TRANSFER_WINDOW_SIZE * (320 / WAV_PWM_SCALE) )

// The current test code creates one wave across the sample buffer, so SAMPLE_BUFF_SIZE 
// determines pitch, so I need to somehow keep it constant when changing other variables
// if I want to A/B.
//
// Unfortunately there's a tiny bit of signal glitch when we reach the end of the txBuf;
// might just be the noise created by core1 running the IMU.  Though we were supposed
// to get good isolation from PS noise with that seperate analog supply ... it's
// an open question what causes it, but there it is.  V8 may be better.
//
// Anyway there's two ways to clean that up. Making the txWin shorter drives it
// above 16khz, or making it longer drives it below 30hz.  The former costs
// more CPU (and power), the latter costs more memory.  ATM i'm doing the former,
// with txWin of 8 samples at sample rate of 130khz, that noise is 
// at 16250hz. Sorry, dogs ...
//
// (Were it not for the 2022 chip shortage, I'd just use a DAC.)

// Here is a spare pwm slice that we can make a timer from:
#define TRIGGER_SLICE 0

#endif

