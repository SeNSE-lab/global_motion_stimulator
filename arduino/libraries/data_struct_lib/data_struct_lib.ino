#define CIRCULAR_BUFFER_OVERFLOW 0
#define CIRCULAR_BUFFER_FILLED   1

// Circular buffer for storing data.
typedef struct circular_buffer_int {
    volatile int *data;
    volatile unsigned int fill_next;
    volatile unsigned int sent_last;
    volatile unsigned int buffer_length;
} circular_buffer_int;

typedef struct sampled_time {
    volatile unsigned long start_time_us;
    volatile unsigned long end_time_us;
    volatile unsigned int first_element_index;
    volatile unsigned int last_element_index;
    volatile unsigned long sampling_frequency;
    volatile int first_element;
    volatile int last_element;
} sampled_time;

circular_buffer_int * initialize_ring_buffer(unsigned int buffer_length) {

    circular_buffer_int *user_buffer;
    user_buffer = (circular_buffer_int *)malloc(sizeof(circular_buffer_int));
    user_buffer->buffer_length = buffer_length;
    user_buffer->fill_next = 0;
    user_buffer->sent_last = 0;
    user_buffer->data = (int *)malloc(buffer_length * sizeof(int));
    // set data to 0.
    for (int data_element=0; data_element<buffer_length; data_element++)
        user_buffer->data[data_element] = 0;

    return user_buffer;
}

int insert_data_buffer(circular_buffer_int *user_buffer, int data) {

    user_buffer->data[user_buffer->fill_next] = data;
    user_buffer->fill_next = (user_buffer->fill_next + 1)% \
                              user_buffer->buffer_length;

    if (user_buffer->fill_next == user_buffer->sent_last)
         return CIRCULAR_BUFFER_OVERFLOW;
    else
         return CIRCULAR_BUFFER_FILLED;
}

boolean buffer_has_data(circular_buffer_int *user_buffer) {

    return (user_buffer->sent_last != user_buffer->fill_next);

}

int grab_data_element(circular_buffer_int *user_buffer) {

    int data = -1;   // default no data marker.
    if (buffer_has_data(user_buffer)) {
        data = user_buffer->data[user_buffer->sent_last];
        user_buffer->sent_last = (user_buffer->sent_last + 1) % \
                                 user_buffer->buffer_length;
    }
    return data;
}

int read_last_element(circular_buffer_int *user_buffer) {

    unsigned int last_filled = (user_buffer->fill_next - 1) % \
                                user_buffer->buffer_length;

    return user_buffer->data[last_filled];
}

// utility to find out remaining SRAM
int free_sram ()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}