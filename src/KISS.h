#ifndef _KISS_H_
#define _KISS_H_

#include <HamShield.h>
#include "packet.h"

#define KISS_FEND 0xC0
#define KISS_FESC 0xDB
#define KISS_TFEND 0xDC
#define KISS_TFESC 0xDD

class KISS {
public:
  KISS(Stream *_io, HamShield *h, DDS *d, AFSK *a) : io(_io), radio(h), dds(d), afsk(a) {}
  bool read();
  void writePacket(AFSK::Packet *);
  void loop();
private:
  Stream *io;
  HamShield *radio;
  DDS *dds;
  AFSK *afsk;
};

#endif /* _KISS_H_ */
