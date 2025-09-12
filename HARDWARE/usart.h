#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"

// Data structures for communication
typedef struct {
    uint8_t head1;
    uint8_t head2;
    // Add other fields as needed
    uint8_t bcc;
} ReportDataSend_t;

typedef struct {
    uint8_t head1;
    uint8_t head2;
    // Add other fields as needed
    uint8_t bcc;
} ReportDataRecv_t;

extern ReportDataSend_t ReportSendpack;
extern ReportDataRecv_t ReportRecvpack;

void usart1_init(uint32_t baud);
void report_to_ros(void);
uint8_t Calculate_BBC(const uint8_t* checkdata, uint16_t datalen);

#endif /* __USART_H */