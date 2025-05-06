#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>

typedef struct {
    int32_t *buffer;       // Pointer to the buffer array
    uint16_t size;         // Maximum size of the FIFO
    uint16_t head;         // Index for the next insertion
    uint16_t count;        // Number of elements currently in the FIFO
    int64_t sum;           // Sum of all elements in the FIFO
} fifo_t;

// Function prototypes
void fifo_init(fifo_t *fifo, int32_t *buffer, uint16_t size);
void fifo_push(fifo_t *fifo, int32_t value);
int32_t fifo_get_average(const fifo_t *fifo);

#endif // FIFO_H