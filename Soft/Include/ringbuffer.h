#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "stdint.h"
/* http://qaru.site/questions/88096/how-do-you-implement-a-circular-buffer-in-c */

typedef struct
{
  uint8_t arr[10];
}
bufStruct;

void initBuff();
int putDataIntoBuff(bufStruct c);
int getDataFromBuff(bufStruct *pc);

#endif // RINGBUFFER_H
