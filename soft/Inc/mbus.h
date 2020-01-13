/*************************************************************************
 Title:    mbus.h - C Library for MBUS data link layer serial communications
 protocol
 Author:   Nicholas Morrow <nickhudspeth@gmail.com>
 File:     mbus.h
 Software: STM32Fxxx_HAL_Driver, CMSIS-CORE
 Hardware: STM32Fxxx
 License:  The MIT License (MIT)

 DESCRIPTION:


 USAGE:

 //// EXAMPLE USAGE FOR RECEIVING AND VERIFYING DATA ////

 mbus_frame_t *it_mbus_frame;
 mbus_frame_t *sys_mbus_frame;
 uint8_t new_frame_flag = 0;
 uint8_t new_cmd_flag = 0;
 char uart_incoming_char;
 unsigned char sys_cmd_buffer[256] = {'\0'};


 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 if(mbus_frame_assemble(it_mbus_frame, uart_incoming_char)){
 new_frame_flag = 1;
 }
 // Re-enable UART interrupt
 if (HAL_UART_Receive_IT(&huart1, &uart_incoming_char, 1)
 != HAL_OK) {
 _Error_Handler(__FILE__, __LINE__);
 }
 }

 void ProcessCmdBuffer(void) {
 mbus_frame_clear(sys_mbus_frame);
 *sys_mbus_frame = *it_mbus_frame;
 mbus_frame_clear(it_mbus_frame);
 mbus_frame_verify_wr(sys_mbus_frame);
 memcpy(sys_cmd_buffer, sys_mbus_frame->payload, sys_mbus_frame->len);
 new_cmd_flag = 1;
 new_frame_flag = 0;
 if (HAL_UART_Receive_IT(&huart1, &uart_incoming_char, 1)
 != HAL_OK) {
 _Error_Handler(__FILE__, __LINE__);
 }
 }

 size_t application_checksum_func(uint8_t *data, uint8_t len){
 uint8_t len32 = (len / 4) + (len % 4);
 return HAL_CRC_Calculate(&hcrc, (uint32_t *)data, len32);
 }

 uint8_t application_uart_ready_func(void){
 if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){
 return 0;
 }
 return 1;
 }

 size_t application_uart_tx_func(uint8_t *buf, uint8_t len){
 return HAL_UART_Transmit_IT(&huart1, buf, len);
 }

 int main(void){
 mbus_cfg_t mbus_config;
 mbus_config.mbus_hw_calc_checksum = application_checksum_func;
 mbus_config.mbus_hw_ready = application_uart_ready_func;
 mbus_config.mbus_hw_transmit = application_uart_tx_func;
 if(mbus_init(mbus_config) != 0){
 _Error_Handler(__FILE__, __LINE__);
 }


 while(1){
 if(new_frame_flag){
 ProcessCmdBuffer();
 }
 if(new_cmd_flag){
 sparse_Exec(parser, sys_cmd_buffer);
 new_cmd_flag = 0;
 }
 }
 }


 //// EXAMPLE USAGE FOR ENCAPSULATING AND TRANSMITTING DATA ////

 int main(void){
 mbus_config_t mbus_config;
 mbus_config.mbus_hw_calc_checksum = application_checksum_func;
 mbus_config.mbus_hw_ready = application_uart_ready_func;
 mbus_config.mbus_hw_transmit = application_uart_tx_func;
 if(mbus_init(mbus_config) != 0){
 _Error_Handler(__FILE__, __LINE__);
 }

 // Encapsulate string "Hello, World!" and send to device with address 54
 mbus_frame_t *f = mbus_frame_assemble(54, "Hello, World!");

 // The frame will be destroyed automatically after transmission.
 mbus_frame_transmit(f);
 }


 NOTES:

 TO-DO:
 Add option to cdxbot_config_t to allow user to specify whether checksum
 should
 be calculated using hardware or software. Add CRC calculation algorithm to
 mbus_calculate_checksum() to perform checksum calculation if software
 calculation
 option is selected. See algorithm used in STMicro's AN4187 for reference.


 LICENSE:
 Copyright (C) 2018 Pathogen Systems, Inc. dba Crystal Diagnostics

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to
 deal in the Software without restriction, including without limitation the
 rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

 *************************************************************************/
#ifndef MBUS_H_
#define MBUS_H_
/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "userdefs.h"
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#ifndef PORT_CALLOC
#error PORT_CALLOC must be defined in "userdefs.h"
#endif
#ifndef PORT_FREE
#error PORT_FREE must be defined in "userdefs.h"
#endif

#define MBUS_BROADCAST_ID 0x00       /* ASCII NUL */
#define MBUS_FRAME_START 0x02        /* ASCII STX */
#define MBUS_FRAME_END 0x03          /* ASCII ETX */
#define MBUS_RETRANSMIT_REQUEST 0x05 /* ASCII ENQ */
#define MBUS_ACK 0x06                /* ASCII ACK */

#define MBUS_FRAME_TYPE_ACK 0x06    /* Return ACK frame type */
#define MBUS_FRAME_TYPE_SINGLE 0x01 /* Message sending by single frame */
#define MBUS_FRAME_TYPE_START_MULTIPLE 0x02 /*START OF MULTIPLE FRAME          \
                                               TRANSMIT*/
#define MBUS_FRAME_TYPE_BODY_MULTIPLE 0x03  /* BODY OF MULTIPLE FRAME          \
                                               TRANSMIT*/
#define MBUS_FRAME_TYPE_END_MULTIPLE                                           \
    0x04 /* END OF MULTIPLE FRAME           \ \ \ \ \                                                                             \
            TRANSMIT*/

#define MBUS_TX_PENDING 0x00
#define MBUS_TX_SUCCESS 0x01
#define MBUS_TX_TIMEOUT 0x02

#define MBUS_RX_DATA_NOT_READY 0x00
#define MBUS_RX_DATA_READY 0x01

#define MBUS_FRAME_READY 1
#define MBUS_FRAME_NOT_READY 0

#define MBUS_POOL_SIZE_MINIMAL 5
#define MBUS_POOL_SIZE_DEFAULT MBUS_POOL_SIZE_MINIMAL
#define MBUS_LATENCY_DEFAULT                                                   \
    100 /* Allowable time (ms) for downstream device to ingest data, process \ \
           \                                                                   \
           \ \                                                                             \
           \ \ \                                                                             \
           \ \ \ \                                                                             \
           message and issue ACK */

#define MBUS_PAYLOAD_LENGTH                                                    \
    248 /* Set to 248 so that struct total size is 256 and aligned to bus \ \  \
           \                                                                   \
           \ \                                                                             \
           \ \ \                                                                             \
           width */
#define MBUS_FRAME_LENGTH_MAX 256 /* Maximum length of MBUS? frame */
#define MBUS_FRAME_LENGTH_ACK 8   /* Length of MBUS ACK frame */

#define MBUS_NULL_CFG_CHECK(cfg)                                               \
    {                                                                          \
        if (cfg == NULL) {                                                     \
            /** Can't call the cfg error handler since cfg is NULL, \ \ \ \ \  \
             * \                                                               \
             * but we can call the library default. */                         \
            mbus_default_error_handler(                                        \
                "Null pointer passed as configuration structure parameter.",   \
                __FILE__, __LINE__);                                           \
        }                                                                      \
    }

typedef struct {
    uint8_t checksum;   /* 32-bit CRC % 255 */
    uint8_t frame_type; /* Frame type like ACK, MULTIPLE FRAME/msg ... */
    uint8_t dst_id;     /* Destination address for frame */
    uint8_t src_id;     /* Source address for frame */
    uint8_t msn;        /* Message sequence number */
    uint8_t len;        /* Payload length (bytes) */
    unsigned char payload[MBUS_PAYLOAD_LENGTH]; /* Data payload */
} mbus_frame_t;

typedef struct {
    mbus_frame_t frame;
    bool allocated;
} mbus_pool_element_t;

typedef struct {
    uint32_t mbus_device_id; /* Bus ID for this device */
    size_t (*mbus_hw_transmit)(
        uint8_t *, uint16_t);    /* User-defined hardware transmit function*/
    bool (*mbus_hw_ready)(void); /* User-defined function to report when
     hardware is ready to transmit */
    size_t (*mbus_hw_calc_checksum)(
        uint8_t *,
        uint8_t); /* User-defined hardware checksum calculation function */
    uint32_t (*mbus_hw_systick)(void);      /* User-defined systick function */
    void (*mbus_hw_reset_peripheral)(void); /** User-defined peripheral
                                                 * reset function. */
    void (*mbus_user_error_handler)(
        char *errstring, char *FILE,
        unsigned int LINE); /** User-defined runtime error handler*/
    mbus_frame_t
        last_transmitted_frame; /* Last frame sent - stored in case of
                                                                                retransmit request */
    uint8_t msn_counter;        /* Accumulator for message sequence number */
    uint8_t msn_last;           /* Sequence number of last message received */
    uint8_t pool_size;  /* Size (in number of frames) of statically-allocated
      memory pool */
    uint32_t baud_rate; /** Clock speed of RX/TX hardware
     *(used for calculating timeouts) */
    uint32_t tx_frame_timeout; /* Time to wait for an acknowledge packet to be
                                  received */
    uint8_t tx_max_retries;    /* Number of times to attempt to retry a frame
                                  transmission */
    mbus_pool_element_t
        *mbus_mem_pool; /* Memory pool for quick allocation of new frames */
    volatile uint8_t txbuf[sizeof(mbus_frame_t) + 2]; /* Temporary buffer with
     guaranteed lifetime for
     outgoing messages */
    char *rx_buffer;
    uint32_t rx_data_len;
    volatile bool last_message_acknowledged;
    uint8_t tx_status; /* Status of transmit message*/
    uint8_t rx_status;
    mbus_frame_t *it_mbus_frame;
    mbus_frame_t *sys_mbus_frame;
    bool tx_busy;
    bool mbus_enable;

} mbus_config_t;

/***********************    FUNCTION PROTOTYPES    ***********************/
uint8_t mbus_calculate_checksum(mbus_config_t *cfg, mbus_frame_t *frame);
uint8_t mbus_frame_assemble(mbus_config_t *cfg, const unsigned char c);
uint8_t mbus_frame_assemble_from_buffer(mbus_config_t *cfg,
                                        unsigned char *buff);
void mbus_frame_clear(mbus_frame_t *frame);
mbus_frame_t *mbus_frame_create(mbus_config_t *cfg, uint8_t frame_type,
                                uint8_t dst_id, char *data,
                                uint8_t data_length);
void mbus_frame_destroy(mbus_config_t *cfg, mbus_frame_t *frame);
mbus_frame_t *mbus_frame_new(mbus_config_t *cfg);
void mbus_frame_retransmit(mbus_config_t *cfg);
void mbus_frame_transmit(mbus_config_t *cfg, mbus_frame_t *frame);
mbus_frame_t *mbus_get_frame_from_pool(mbus_config_t *cfg);
mbus_config_t *mbus_init(uint32_t id, uint32_t baud);
int mbus_set_device_id(mbus_config_t *cfg, uint32_t id);
int mbus_frame_send(mbus_config_t *cfg, mbus_frame_t *frame);
int mbus_data_send(mbus_config_t *cfg, uint8_t id, char *data, uint32_t len);
int mbus_frame_receive(mbus_config_t *cfg, mbus_frame_t *frame);
int mbus_data_receive(mbus_config_t *cfg, char *buf);
uint32_t mbus_data_is_available(mbus_config_t *cfg);
int mbus_frame_verify(mbus_config_t *cfg, mbus_frame_t *frame);
void mbus_default_error_handler(char *errstring, char *FILE, unsigned int LINE);

#endif /* MBUS_H_ */
