#ifndef LOOPVARIABLES_H
#define LOOPVARIABLES_H

#include <CircularBuffer.h>

/////////////////////////////
// Some utils for handling loop variables, which are very short CircularBuffers
// for comparing this loop's value to the previous loop's value.
// The most recent value will be in cb[0], the previous in cb[1], etc.
////////////////////////////
// init the buffer with one value in all positions:
#define CBINIT(cb, val) \
  while (!cb.isFull()) cb.push(val)

// change the current value without touching the previous value:
#define CBSET(cb, val) \
  { \
    cb.shift(); \
    cb.unshift(val); \
  }

// update the value (current becomes previous)
#define CBPUSH(cb, val) (cb.unshift(val))

// update the current value to match the previous value
#define CBNEXT(cb) (cb.unshift(cb[0]))

// get the current value
#define CBGET(cb, val) (cb[0])

// test: current value is less than previous
#define CBFELL(cb) (cb[0] < cb[1])  // also works for bools in C++ because TRUE = 1 and FALSE = 0

// test: current value is below val, and previous value was above it
#define CBFELLTHRU(cb, val) (cb[0] <= val && cb[1] > val)

// test: current value is more than previous
#define CBROSE(cb) (cb[0] > cb[1])

// test: current value is above val, and previous value was below it
#define CBROSETHRU(cb, val) (cb[0] > val && cb[1] <= val)

// test: value changed (current != previous)
#define CBDIFF(cb) (cb[0] != cb[1])






#endif
