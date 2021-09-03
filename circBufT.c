// *******************************************************
// 
// circBufT.c
//
// Support for a circular buffer of uint32_t values on the 
// Tiva processor.
// P.J. Bones UCECE
// Last modified:  15.8.2020
// 
// *******************************************************
#include "main.h"
#include "circBufT.h"

// *******************************************************
// writeCircBuf: insert entry at the current windex location,
//              advance windex, modulo (buffer size).
// *******************************************************
void writeCircBuf(circBuf_t *buffer, uint32_t entry) {
    *(buffer->data + buffer->windex) = entry;
    buffer->windex++;
    if (buffer->windex >= buffer->size)
       buffer->windex = 0;
}

// *******************************************************
// readCircBuf: return entry at the current rindex location,
//              advance rindex, modulo (buffer size). The function deos not check
//              if reading has advanced ahead of writing.
// *******************************************************
uint32_t readCircBuf(circBuf_t *buffer) {
    uint32_t entry;
    //get the data at the current read index
    entry = buffer->data[buffer->rindex];
    // Advance the read index
    buffer->rindex++;
    if (buffer->rindex >= buffer->size)
       buffer->rindex = 0;
    return entry;
}

