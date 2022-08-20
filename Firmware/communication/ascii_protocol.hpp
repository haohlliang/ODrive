#ifndef __ASCII_PROTOCOL_H
#define __ASCII_PROTOCOL_H


/* Includes ------------------------------------------------------------------*/
#include <fibre/protocol.hpp>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fibre/introspection.hpp>
//修改 增加LCD头文件
#include <HIMI/lcd.hpp> // XBoard
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink& response_channel);

//修改 添加 extern Introspectable root_obj;
extern Introspectable root_obj;


#endif /* __ASCII_PROTOCOL_H */
