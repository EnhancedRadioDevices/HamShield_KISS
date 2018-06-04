#ifndef _BELL202_H_
#define _BELL202_H_

/*
 * bell202.h
 * The Bell202 class implements an AFSK transceiver at the bit level.
 * All encoding and framing should be done at a higher level (e.g. HDLC and AX.25)
 * 
 * See http://destevez.net/2016/06/kiss-hdlc-ax-25-and-friends/
 */

#include <Arduino.h>
#include <DDS.h>
#include <avr/pgmspace.h>

#define SAMPLERATE 9600
#define BITRATE    1200

#define SAMPLEPERBIT (SAMPLERATE / BITRATE)

#define RX_FIFO_LEN 16

#define AFSK_RX_FIFO_LEN 16

#define BIT_DIFFER(bitline1, bitline2) (((bitline1) ^ (bitline2)) & 0x01)

#define EDGE_FOUND(bitline)            BIT_DIFFER((bitline), (bitline) >> 1)

#define T_BIT ((unsigned int)(SAMPLERATE/BITRATE))

#define PHASE_BIT 8
#define PHASE_INC 1

#define PHASE_MAX (SAMPLEPERBIT * PHASE_BIT)
#define PHASE_THRES (PHASE_MAX / 2)

#define AFSK_SPACE 2200
#define AFSK_MARK  1200

#define ADC_OFFSET 83

class Bell202 {
private:
  volatile bool afskEnabled;
public:
  Bell202() {
    sending = false;
  }
  volatile inline bool enabled() { return afskEnabled; }
  volatile inline bool isSending() volatile { 
      return sending; 
  }

  void encoderSetDDS(DDS *d) { dds = d; }
  //int16 getReferenceClock() { return dds.getReferenceClock(); }
  bool encoderStart();
  void encoderStop();
  void encoderSetTone(bool tone);
  bool encoderGetTone() { return currentTone; }
  
  void decoderStart();
  bool decoderRead();
  bool decoderProcess(int8_t);
  
  void start(DDS *);

  uint8_t found_bits;
  
private:
  volatile bool sending;
  byte currentTone : 1;
  //byte bitClock;
  DDS *dds;
  
  int16_t iir_x[2];
  int16_t iir_y[2];
  int8_t curr_phase;
  uint8_t sampled_bits;
  
};


#endif /* _BELL202_H_ */
