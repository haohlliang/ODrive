#ifndef __INTERFACE_UART_HPP
#define __INTERFACE_UART_HPP

#ifdef __cplusplus
#include "fibre/protocol.hpp"
//修改 增加LCD头文件
#include <HIMI/lcd.hpp> // XBoard

extern StreamSink* uart4_stream_output_ptr;

extern "C" {
#endif

#include <cmsis_os.h>

extern osThreadId uart_thread;
extern const uint32_t stack_size_uart_thread;

void start_uart_server(void);

#ifdef __cplusplus
}
#endif

#endif // __INTERFACE_UART_HPP
