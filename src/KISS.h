#ifndef _KISS_H_
#define _KISS_H_

#include <HamShield.h>
#include "AX25.h"

#define KISS_FEND 0xC0
#define KISS_FESC 0xDB
#define KISS_TFEND 0xDC
#define KISS_TFESC 0xDD

class KISS {
public:
  KISS(Stream *_io, HamShield *h, DDS *d, AX25 *a) : io(_io), radio(h), dds(d), ax25(a) {}
  bool read();
  void writePacket(AX25::Packet *);
  void loop();
private:
  Stream *io;
  HamShield *radio;
  DDS *dds;
  AX25 *ax25;
};

#endif /* _KISS_H_ */
