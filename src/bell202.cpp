#include <Arduino.h>
#include "dds.h"
#include "bell202.h"


#define T_BIT ((unsigned int)(SAMPLERATE/BITRATE))

bool Bell202::encoderStart() {
  sending = true;
  dds->setFrequency(0);
  dds->on();
  return true;
}
void Bell202::encoderStop() {
  sending = false;
  dds->setFrequency(0);
  dds->off();
}

void Bell202::encoderSetTone(bool tone) {
  currentTone = tone;
  if(tone == 0) {
    PORTD |= _BV(7);
    dds->setFrequency(AFSK_SPACE);
  } else {
    PORTD &= ~_BV(7);
    dds->setFrequency(AFSK_MARK);
  }  
}

template <typename T, int size>
class FastRing {
private:
  T ring[size];
  uint8_t position;
public:
  FastRing(): position(0) {}
  inline void write(T value) {
    ring[(position++) & (size-1)] = value;
  }
  inline T read() const {
    return ring[position & (size-1)];
  }
  inline T readn(uint8_t n) const {
    return ring[(position + (~n+1)) & (size-1)];
  }
};
// Create a delay line that's half the length of the bit cycle (-90 degrees)
FastRing<uint8_t,(T_BIT/2)> delayLine;

// Handle the A/D converter interrupt (hopefully quickly :)
bool Bell202::decoderProcess(int8_t curr_sample) {
  // Run the same through the phase multiplier and butterworth filter
  iir_x[0] = iir_x[1];
  iir_x[1] = ((int8_t)delayLine.read() * curr_sample) >> 2;
  iir_y[0] = iir_y[1];
  iir_y[1] = iir_x[0] + iir_x[1] + (iir_y[0] >> 1) + (iir_y[0]>>3) + (iir_y[0]>>5);
  
  // Place this ADC sample into the delay line
  delayLine.write(curr_sample);

  // Shift the bit into place based on the output of the discriminator
  sampled_bits <<= 1;
  sampled_bits |= (iir_y[1] > 0) ? 1 : 0;  
  
  // If we found a 0/1 transition, adjust phases to track
  if(EDGE_FOUND(sampled_bits)) {
    if(curr_phase < PHASE_THRES)
      curr_phase += PHASE_INC;
    else
      curr_phase -= PHASE_INC;
  }
  
  // Move ahead in phase
  curr_phase += PHASE_BIT;
  
  // If we've gone over the phase maximum, we should now have some data
  if(curr_phase >= PHASE_MAX) {
    curr_phase %= PHASE_MAX;
    found_bits <<= 1;
    
    // If we have 3 bits or more set, it's a positive bit
    register uint8_t bits = sampled_bits & 0x07;
    if(bits == 0x07 || bits == 0x06 || bits == 0x05 || bits == 0x03) {
      found_bits |= 1;
    }
    
    // pass back to encoder
    return true;
  }
  return false;
}
  

#define AFSK_ADC_INPUT 2
void Bell202::decoderStart() {
  /*  ASSR &= ~(_BV(EXCLK) | _BV(AS2));

  // Do non-inverting PWM on pin OC2B (arduino pin 3) (p.159).
  // OC2A (arduino pin 11) stays in normal port operation:
  // COM2B1=1, COM2B0=0, COM2A1=0, COM2A0=0
  // Mode 1 - Phase correct PWM
  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~(_BV(COM2B0) | _BV(COM2A1) | _BV(COM2A0)) |
           _BV(WGM21) | _BV(WGM20);
  // No prescaler (p.162)
  TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21))) | _BV(CS20) | _BV(WGM22);
  
  OCR2A = pow(2,COMPARE_BITS)-1;
  OCR2B = 0;*/
  
  
  // This lets us use decoding functions that run at the same reference
  // clock as the DDS.
  // We use ICR1 as TOP and prescale by 8
  // Note that this requires the DDS to be started as well
  TCCR1A = 0;
  TCCR1B = _BV(CS11) | _BV(WGM13) | _BV(WGM12);
  ICR1 = ((F_CPU / 8) / 9600) - 1; //TODO: get the actual refclk from dds
  // NOTE: should divider be 1 or 8?
  ADMUX = _BV(REFS0) | _BV(ADLAR) | AFSK_ADC_INPUT; // Channel AFSK_ADC_INPUT, shift result left (ADCH used)
  DDRC &= ~_BV(AFSK_ADC_INPUT);
  PORTC &= ~_BV(AFSK_ADC_INPUT);
  DIDR0 |= _BV(AFSK_ADC_INPUT); // disable input buffer for ADC pin
  ADCSRB = _BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0);
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2); // | _BV(ADPS0);
}

void Bell202::start(DDS *dds) {
  afskEnabled = true;
  encoderSetDDS(dds);
  decoderStart();
}
