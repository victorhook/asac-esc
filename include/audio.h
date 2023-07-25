#ifndef AUDIO_H
#define AUDIO_H

#include "stdint.h"

#define FREQ_A4 (float) 440.00
#define FREQ_B4 (float) 493.88
#define FREQ_C4 (float) 261.63
#define FREQ_D4 (float) 293.66
#define FREQ_E4 (float) 329.63
#define FREQ_F4 (float) 349.23
#define FREQ_G4 (float) 392.00

#define FREQ_A5 (float) 880.00
#define FREQ_B5 (float) 987.77
#define FREQ_C5 (float) 523.25
#define FREQ_D5 (float) 587.33
#define FREQ_E5 (float) 659.25
#define FREQ_F5 (float) 698.46
#define FREQ_G5 (float) 783.99

#define TONE_A ( (float) ((float) 1000000.0 / FREQ_A5) )
#define TONE_B ( (float) ((float) 1000000.0 / FREQ_B5) )
#define TONE_C ( (float) ((float) 1000000.0 / FREQ_C5) )
#define TONE_D ( (float) ((float) 1000000.0 / FREQ_D5) )
#define TONE_E ( (float) ((float) 1000000.0 / FREQ_E5) )
#define TONE_F ( (float) ((float) 1000000.0 / FREQ_F5) )
#define TONE_G ( (float) ((float) 1000000.0 / FREQ_G5) )


typedef struct
{
    float    tone;
    uint32_t on_duration_ms;
    uint32_t off_duration_ms;
} note_t;


void play_melody(const note_t notes[], const uint16_t nbr_of_notes);


#endif /* AUDIO_H */
