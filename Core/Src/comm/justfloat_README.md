# JustFloat协议实现

## 概述
JustFloat是一个简单的二进制数据传输协议，用于在STM32和上位机之间传输浮点数据。

## 数据格式
```c
#define CH_COUNT 2  // 通道数量

struct Frame {
    float fdata[CH_COUNT];        // 小端浮点数组
    unsigned char tail[4];        // 帧尾 {0x00, 0x00, 0x80, 0x7f}
};
```

## 数据示例
发送4个曲线的数据格式：
```
bf 10 59 3f b1 02 95 3e 57 a6 16 be 7b 4d 7f bf 00 00 80 7f
```

## API函数

### 发送函数
- `void send_float_binary(float data)` - 发送单个浮点数
- `void send_tail()` - 发送帧尾
- `void send_frame(struct Frame frame)` - 发送完整帧

### 解析函数
- `int parse_frame(uint8_t *buffer, int buffer_len, struct Frame *frame)` - 解析接收缓冲区中的帧
- `int find_frame_start(uint8_t *buffer, int buffer_len)` - 查找帧开始位置
- `int validate_frame_tail(uint8_t *buffer, int offset)` - 验证帧尾

### 便捷函数
- `void init_frame(struct Frame *frame)` - 初始化帧结构体
- `void create_frame(struct Frame *frame, float *data)` - 创建包含数据的帧

## 使用示例

### 发送数据
```c
// 方法1：使用便捷函数
struct Frame frame;
float data[CH_COUNT] = {1.23f, 4.56f};
create_frame(&frame, data);
send_frame(frame);

// 方法2：手动创建
struct Frame frame;
init_frame(&frame);
frame.fdata[0] = 1.23f;
frame.fdata[1] = 4.56f;
send_frame(frame);
```

### 接收数据
```c
uint8_t rx_buffer[256];
int buffer_pos = 0;

// 接收数据到缓冲区...

// 解析帧
struct Frame received_frame;
int next_pos = parse_frame(rx_buffer, buffer_pos, &received_frame);

if (next_pos > 0) {
    // 成功解析到数据
    float ch1 = received_frame.fdata[0];
    float ch2 = received_frame.fdata[1];
    
    // 移除已处理的数据
    memmove(rx_buffer, &rx_buffer[next_pos], buffer_pos - next_pos);
    buffer_pos -= next_pos;
}
```

## 注意事项
1. 浮点数使用小端字节序
2. 帧尾固定为 `{0x00, 0x00, 0x80, 0x7f}`
3. 通道数量由 `CH_COUNT` 宏定义
4. 解析函数会查找帧尾来确定帧边界
5. 缓冲区中可能包含多个帧，解析函数会返回下一帧的位置
