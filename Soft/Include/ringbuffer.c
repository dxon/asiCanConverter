#include "ringbuffer.h"

#define BUFSIZE 256
volatile bufStruct buf[BUFSIZE];
volatile bufStruct *pIn, *pOut, *pEnd;
volatile uint32_t full;

// init
void initBuff()
{
    pIn = pOut = buf;       // init to any slot in buffer
    pEnd = &buf[BUFSIZE];   // past last valid slot in buffer
    full = 0;               // buffer is empty
}

// add to ring buffer
int putDataIntoBuff(bufStruct c)
{
    if (pIn == pOut  &&  full)
    {
        return 0;           // buffer overrun
    }

    *pIn++ = c;             // insert c into buffer
    if (pIn >= pEnd)        // end of circular buffer?
        pIn = buf;          // wrap around

    if (pIn == pOut)       // did we run into the output ptr?
      {
        full = 1;           // can't add any more data into buffer
      }
    
    return 1;               // all OK
}

// get from ring buffer
int getDataFromBuff(bufStruct *pc)
{
    if (pIn == pOut  &&  !full)
    {
        return 0;           // buffer empty  FAIL
    }

    *pc = *pOut++;              // pick up next data to be returned
    
    if (pOut >= pEnd)       // end of circular buffer?
    {
        pOut = buf;         // wrap around
    }

    full = 0;               // there is at least 1 slot
    
    return 1;               // *pc has the data to be returned
}

void clearBuff()
{
    pOut = pIn = 0;

    initBuff();
}