
```shell
arm-none-eabi-gdb build/Debug/RaspVisionCar.elf -q -ex "target extended-remote 192.168.232.107:3333" -ex "monitor reset halt" -ex "load" -ex "monitor reset" -ex "detach" -ex "quit"
```

使用socat启动 TCP 服务器（监听在 3444 端口），并将所有来自 TCP 的连接数据转发到串口 /dev/ttyS0（通常是 GPIO 上的串口），同时将串口接收到的数据发送给所有已连接的 TCP 客户端。

sudo socat TCP-LISTEN:3444,reuseaddr,fork FILE:/dev/ttyAMA0,b115200,raw,echo=0