#define CH_COUNT 2
#include "main.h"

struct Frame {
    float fdata[CH_COUNT];
    //char tail[4];
};

// Structure to hold float as bytes
typedef union {
    float f;
    uint8_t bytes[4];
} float_union_t;

void send_float_binary(float data);
	void send_tail();
void send_frame(struct Frame frame);
void send_to_serial(char *ptr);