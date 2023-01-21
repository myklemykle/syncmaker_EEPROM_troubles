// TODO: license
// (See WavPwmAudio.cpp)

#ifndef __WAV_PWM_AUDIO_H
#define __WAV_PWM_AUDIO_H


#define WAV_SAMPLE_RATE          22050
#define WAV_PWM_COUNT            (125000000 / WAV_SAMPLE_RATE)


void WavPwmInit(unsigned char GpioPinChannelA);
unsigned char WavPwmIsPlaying();
void WavPwmStopAudio();
unsigned char WavPwmPlayAudio(unsigned short WavPwmData[]);


// Audio buffer 2 channels for 2 seconds @ 22050 Hz Samples/Second.
// (It's remarkable how long a white noise sample has to be before you can't detect some
// looping artifact.  Longer than 2 seconds, for sure.)
#define AUDIO_CHANNELS                 2
#define AUDIO_PERIOD                   1
//#define AUDIO_BUFF_SIZE                (AUDIO_CHANNELS * WAV_SAMPLE_RATE * AUDIO_PERIOD)
#define AUDIO_BUFF_SIZE                (AUDIO_CHANNELS * WAV_SAMPLE_RATE * AUDIO_PERIOD)


#endif

