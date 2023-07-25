#include "audio.h"
#include "asac_esc.h"


#define AUDIO_THROTTLE_LEVEL 2048


/*
    Plays a single tone for a given amount of time.
    Note that "tone" is really a value of: 1000000.0 / frequency
*/
static inline void play_tone(const float tone, const uint32_t duration_ms)
{
    uint32_t t0 = micros();

    // We'll let the current go through the motor for this amount of time,
    // in order to let the rotor commutate.
    uint16_t high_time_us = 100;

    while ((micros() - t0) < (duration_ms * 1000))
    {
        // Disable interrupts, in case input signal interrupt occurs here and
        // changes the throttle value.
        disable_interrupts();
        state.throttle = AUDIO_THROTTLE_LEVEL;
        commutate();
        enable_interrupts();
        state.direction *= -1;

        sleep_us(high_time_us);
        all_outputs_low();
        sleep_us(tone - high_time_us);
    }
}


void play_melody(const note_t notes[], const uint16_t nbr_of_notes)
{
    // Save throttle and direction before we start, since we will alter them
    uint16_t throttle = state.throttle;
    direction_t direction = state.direction;

    for (int i = 0; i < nbr_of_notes; i++)
    {
        note_t* note = &notes[i];
        play_tone(note->tone, note->on_duration_ms);
        sleep_ms(note->off_duration_ms);
    }

    // Restore throttle and diretions
    state.throttle = throttle;
    state.direction = direction;
    reset_commutation_state();
}