#include <stdint.h>

struct FIFO {
  uint8_t data[10];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
};
void FIFO_push(struct FIFO *FIFO, uint8_t data);
uint8_t FIFO_pop(struct FIFO *FIFO);
uint8_t FIFO_count(struct FIFO *FIFO);

