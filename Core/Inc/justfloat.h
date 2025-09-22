#define CH_COUNT 2
#include "main.h"

struct Frame {
	float fdata[CH_COUNT];
	unsigned char tail[4];
};

// Structure to hold float as bytes
typedef union {
	float f;
	uint8_t bytes[4];
} float_union_t;

void send_float_binary(float data);
void send_tail(void);
void send_frame(const struct Frame *frame);
void send_frame_data(const float *data);
void send_to_serial(const char *ptr);

// 数据解析函数
int parse_frame(uint8_t *buffer, int buffer_len, struct Frame *frame);
int find_frame_start(uint8_t *buffer, int buffer_len);
int validate_frame_tail(uint8_t *buffer, int offset);

// 便捷函数
void init_frame(struct Frame *frame);
void create_frame(struct Frame *frame, float *data);