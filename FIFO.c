#include <FIFO.h>

void FIFO_push(struct FIFO *fifo, uint8_t data)
{
  fifo->data[fifo->head] = data;
  fifo->head = (fifo->head + 1) % 10;
  fifo->count++;
}
uint8_t FIFO_pop(struct FIFO *fifo)
{
  uint8_t data = fifo->data[fifo->tail];
  fifo->tail = (fifo->tail + 1) % 10;
  fifo->count--;
  return data;
}
uint8_t FIFO_count(struct FIFO *fifo)
{
  return fifo->count;
}
