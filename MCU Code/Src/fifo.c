#include "fifo.h"

void fifo_init(fifo_t *fifo, int32_t *buffer, uint16_t size) {
    fifo->buffer = buffer;
    fifo->size = size;
    fifo->head = 0;
    fifo->count = 0;
    fifo->sum = 0;
}

void fifo_push(fifo_t *fifo, int32_t value) {
    if (fifo->count == fifo->size) {
        // FIFO is full, remove the oldest value
        int32_t removed_value = fifo->buffer[fifo->head];
        fifo->sum -= removed_value;
    } else {
        fifo->count++;
    }

    // Add the new value
    fifo->sum += value;
    fifo->buffer[fifo->head] = value;

    // Update the head index
    fifo->head = (fifo->head + 1) % fifo->size;
}

int32_t fifo_get_average(const fifo_t *fifo) {
    if (fifo->count == 0) {
        return 0; // Avoid division by zero
    }
    return (int32_t)(fifo->sum / fifo->count);
}