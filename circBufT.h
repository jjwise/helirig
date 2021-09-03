#ifndef CIRCBUFT_H_
#define CIRCBUFT_H_

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

// *******************************************************

/* Type Definitions -----------------------------------------*/
// Buffer structure
typedef struct {
    uint32_t size;      // Number of entries in buffer
    uint32_t windex;    // index for writing, mod(size)
    uint32_t rindex;    // index for reading, mod(size)
    uint32_t *data;     // pointer to the data
} circBuf_t;
/*--------------------------------------------------------------*/

/* Function prototypes -----------------------------------------*/

// *******************************************************
// writeCircBuf: insert entry at the current windex location,
// advance windex, modulo (buffer size).
void writeCircBuf (circBuf_t *buffer, uint32_t entry);

// *******************************************************
// readCircBuf: return entry at the current rindex location,
// advance rindex, modulo (buffer size). The function deos not check
// if reading has advanced ahead of writing.
uint32_t readCircBuf (circBuf_t *buffer);

/*--------------------------------------------------------------*/

#endif /*CIRCBUFT_H_*/
