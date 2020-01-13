/************************************************************************
 Title:	  mbus.c - C Library for MBUS data link layer serial
 communications protocol
 Author:   Nicholas Morrow <nickhudspeth@gmail.com>
 File:     mbus.c
 Software: STM32Fxxx_HAL_Driver, CMSIS-CORE
 Hardware: STM32Fxxx
 License:  The MIT License (MIT)
 Usage:    Refer to the header file mbus.h for a description of the routines.
 See also example test_mbus.c, if available.
 LICENSE:
 Copyright (C) 2018 Nicholas Morrow

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
 ************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include "mbus.h"
/**********************    GLOBAL VARIABLES    ***********************/
mbus_config_t _mbus_config;

/* Debug variables */
volatile int32_t mbus_frames_allocated = 0;
volatile uint32_t mbus_frames_lost = 0;
volatile uint32_t mbus_frames_received = 0;
volatile mbus_frame_t *last_allocated_frame_address = NULL;
volatile uint32_t frames_since_missed_checksum = 0;
volatile uint32_t fsmc_accumulator = 0;

/*******************    FUNCTION IMPLEMENTATIONS    ********************/

uint8_t mbus_calculate_checksum(mbus_config_t *cfg, mbus_frame_t *frame) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (frame == NULL) {
        cfg->mbus_user_error_handler("Null pointer passed as frame argument.",
                                     __FILE__, __LINE__);
    }
    if (cfg->mbus_hw_calc_checksum == NULL) {
        cfg->mbus_user_error_handler("No valid function registered for "
                                     "cfg->mbus_hw_calc_checksum.",
                                     __FILE__, __LINE__);
    }
    /** The next line requires some explanation. Since all elements of
     * mbus_frame_t before the payload
     * are of type uint8_t, we calculate the checksum starting with the second
     * element of the struct and
     * continue to the end of the payload.
     * */
    uint8_t ret = (uint8_t)cfg->mbus_hw_calc_checksum(&(frame->frame_type),
                                                      4 + frame->len) %
                  255;
    return ret;
}

uint8_t mbus_frame_assemble_from_buffer(mbus_config_t *cfg,
                                        unsigned char *buf) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (buf == NULL) {
        cfg->mbus_user_error_handler("Null pointer passed as buf argument.",
                                     __FILE__, __LINE__);
        return -1;
    }
    /** if mbus_enable is false, just copy all buffer to rx_buff */
    if (cfg->mbus_enable == false) {
        /** Because we don't know how many bytes received to buf,
         * copy all the contain in the buf to rx_buf,
         * assume max length of message is the max length of 1 cdxframe
         */
        // Reallocate rx_buffer to the MBUS_FRAME_LENGTH_MAX
        cfg->rx_buffer = PORT_REALLOC(cfg->rx_buffer, MBUS_FRAME_LENGTH_MAX);
        if (cfg->rx_buffer == NULL) {
            /** Error: Insufficient memory to allocate array */
            return -1;
        }
        memcpy(cfg->rx_buffer, buf, MBUS_FRAME_LENGTH_MAX);
        cfg->rx_data_len = MBUS_FRAME_LENGTH_MAX;
        cfg->rx_status = MBUS_RX_DATA_READY;
        memset(buf, 0, MBUS_FRAME_LENGTH_MAX);
        return MBUS_FRAME_READY;
    }

    /** mbus assemble frame from buffer */
    int i = 0;
    int res = 0;
    while (1) {
        res = mbus_frame_assemble(cfg, buf[i]);
        if (res == MBUS_FRAME_READY) {
            return MBUS_FRAME_READY;
        }
        i++;
        if (i >= MBUS_FRAME_LENGTH_MAX) {
            return MBUS_FRAME_NOT_READY;
        }
    }
}

uint8_t mbus_frame_assemble(mbus_config_t *cfg, const unsigned char c) {
    MBUS_NULL_CFG_CHECK(cfg);
    static uint8_t mode = 0;
    static uint8_t pidx = 0;
    mbus_frame_t *frame = cfg->it_mbus_frame;

    /** Ignore all received char without START FRAME char*/
    if (mode == 0 && c != MBUS_FRAME_START) {
        return MBUS_FRAME_NOT_READY;
    }

    /** If in IDLE mode and received a START (0x02), move on to received
     * process*/
    if (mode == 0 && c == MBUS_FRAME_START) {
        mbus_frame_clear(frame);
        mode = 1;
        return MBUS_FRAME_NOT_READY;
    }

    switch (mode) {
    case 0:
        /** Waiting for start frame. This case should never be executed */
        break;
    case 1:
        /** Start frame was last character received. This is the checksum */
        frame->checksum = (uint8_t)c;
        mode = 2;
        break;
    case 2:
        /** Checksum was the last character received. This is frame_type.
         */
        frame->frame_type = (uint8_t)c;
        mode = 3;
        break;
    case 3:
        /** Frame_type was the last character received. This is the destination
         * id.
         */
        frame->dst_id = (uint8_t)c;
        mode = 4;
        break;
    case 4:
        /** Destination id was the last character received. This is the source
         * id.
         */
        frame->src_id = (uint8_t)c;
        mode = 5;
        break;
    case 5:
        /** Source id was the last character received. This is the message
         * sequence
         * number. */
        frame->msn = (uint8_t)c;
        mode = 6;
        break;
    case 6:
        /** Message sequence number was the last character received. This is the
         * payload length. */
        frame->len = (uint8_t)c;
        if (frame->len > MBUS_PAYLOAD_LENGTH) {
            mode = 0;
            break;
        }
        pidx = 0;
        mode = 7;
        break;
    case 7:
        /** Receive (frame->len) char into payload. */
        if (pidx < frame->len) {
            frame->payload[pidx] = c;
            pidx++;
            break;
        } else {
            /** Check for MBUS_FRAME_END character */
            mode = 0;
            if (frame->frame_type != MBUS_FRAME_TYPE_ACK) {
                mbus_frames_received++;
            }
            if (c != MBUS_FRAME_END) {
                return MBUS_FRAME_NOT_READY;
            }
            /** This is a new frame. Swap the buffers to ensure data
             * consistency*/
            mbus_frame_clear(cfg->sys_mbus_frame);
            *(cfg->sys_mbus_frame) = *(cfg->it_mbus_frame);
            mbus_frame_clear(cfg->it_mbus_frame);
            if (mbus_frame_verify(cfg, cfg->sys_mbus_frame) == 0) {
                mbus_frame_receive(cfg, cfg->sys_mbus_frame);
                return MBUS_FRAME_READY;
            }
        }
    }
    return MBUS_FRAME_NOT_READY;
}

void mbus_frame_clear(mbus_frame_t *frame) {
    if (frame == NULL) {
        mbus_default_error_handler("Null pointer passed as frame argument.",
                                   __FILE__, __LINE__);
        return;
    }
    frame->checksum = 0;
    frame->frame_type = 0;
    frame->dst_id = 0;
    frame->src_id = 0;
    frame->msn = 0;
    frame->len = 0;
    memset(frame->payload, 0, sizeof(frame->payload));
}

mbus_frame_t *mbus_frame_create(mbus_config_t *cfg, uint8_t frame_type,
                                uint8_t dst_id, char *data,
                                uint8_t data_length) {
    MBUS_NULL_CFG_CHECK(cfg);
    mbus_frame_t *ret = mbus_get_frame_from_pool(cfg);
    if (ret == NULL) {
        cfg->mbus_user_error_handler(
            "Could not allocate frame from memory pool.", __FILE__, __LINE__);
        return NULL;
    }

    last_allocated_frame_address = ret;
    mbus_frames_allocated++;

    memcpy(ret->payload, data, data_length);
    ret->frame_type = frame_type;
    ret->dst_id = dst_id;
    ret->src_id = cfg->mbus_device_id;
    if (frame_type == MBUS_FRAME_TYPE_ACK) {
        ret->msn = cfg->msn_last;
    } else {
        ret->msn = ++(cfg->msn_counter);
    }
    ret->len = data_length;
    ret->checksum = mbus_calculate_checksum(cfg, ret);

    return ret;
}

void mbus_frame_destroy(mbus_config_t *cfg, mbus_frame_t *frame) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (frame == NULL) {
        cfg->mbus_user_error_handler("Null pointer passed as frame argument.",
                                     __FILE__, __LINE__);
        return;
    }
    /** We have to scan through the memory pool to find
     * the matching mbus_mem_pool entry so that we can
     * appropriately mark the frame as having been deallocated.
     */
    for (uint8_t i = 0; i < cfg->pool_size; i++) {
        if (frame == &(cfg->mbus_mem_pool[i].frame)) {
            mbus_frame_clear(&(cfg->mbus_mem_pool[i].frame));
            cfg->mbus_mem_pool[i].allocated = false;
        }
    }
    mbus_frames_allocated--;
}

mbus_frame_t *mbus_frame_new(mbus_config_t *cfg) {
    MBUS_NULL_CFG_CHECK(cfg);
    mbus_frame_t *ret = mbus_get_frame_from_pool(cfg);
    if (ret == NULL) {
        cfg->mbus_user_error_handler(
            "Could not allocate frame from memory pool.", __FILE__, __LINE__);
        return NULL;
    }
    last_allocated_frame_address = ret;
    mbus_frames_allocated++;

    return ret;
}

void mbus_frame_retransmit(mbus_config_t *cfg) {
    MBUS_NULL_CFG_CHECK(cfg);
    mbus_frame_t *frame = mbus_frame_new(cfg);
    if (frame == NULL) {
        cfg->mbus_user_error_handler(
            "Could not allocate frame from memory pool.", __FILE__, __LINE__);
        return;
    }
    *frame = cfg->last_transmitted_frame;
    mbus_frame_transmit(cfg, frame);
}

void mbus_frame_transmit(mbus_config_t *cfg, mbus_frame_t *frame) {
    MBUS_NULL_CFG_CHECK(cfg);
    /** (START + STOP) + (checksum + frame_type + dst_id + src_id + msn + len) +
     * payload */
    // Check for NULL frame transmit
    if (frame == NULL) {
        cfg->mbus_user_error_handler("Null pointer passed as frame argument.",
                                     __FILE__, __LINE__);
        return;
    }
    while (cfg->tx_busy == 1) {
        ;
    }
    cfg->tx_busy = 1;
    uint16_t size = 2 + 6 + frame->len;
    /** Wait until previous frame has been transmitted and hardware
     * is ready to send */
    size_t time_start = cfg->mbus_hw_systick(), time = 0;
    while (!cfg->mbus_hw_ready()) {
        time = cfg->mbus_hw_systick() - time_start;
        if (time >= cfg->tx_frame_timeout) {
            if (cfg->mbus_hw_reset_peripheral != NULL) {
                cfg->mbus_hw_reset_peripheral();
            }
            time_start = cfg->mbus_hw_systick();
        };
    }

    cfg->txbuf[0] = MBUS_FRAME_START;
    cfg->txbuf[1] = frame->checksum;
    cfg->txbuf[2] = frame->frame_type;
    cfg->txbuf[3] = frame->dst_id;
    cfg->txbuf[4] = frame->src_id;
    cfg->txbuf[5] = frame->msn;
    cfg->txbuf[6] = frame->len;
    memmove((void *)((cfg->txbuf) + 7), (void *)(&(frame->payload)),
            frame->len);
    cfg->txbuf[7 + frame->len] = MBUS_FRAME_END;
    cfg->last_message_acknowledged = false;
    cfg->mbus_hw_transmit((uint8_t *)&(cfg->txbuf), size);
    if (frame->frame_type != MBUS_FRAME_TYPE_ACK) {
        cfg->tx_status = MBUS_TX_PENDING;
        cfg->last_transmitted_frame = *frame;
    }

    mbus_frame_destroy(cfg, frame);
    cfg->tx_busy = 0;
}

int mbus_frame_verify(mbus_config_t *cfg, mbus_frame_t *frame) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (frame == NULL) {
        cfg->mbus_user_error_handler("Null pointer passed as frame argument.",
                                     __FILE__, __LINE__);
        return -2;
    }
    /** Verify checksum for the received frame. */
    if (frame->checksum != mbus_calculate_checksum(cfg, frame)) {
        mbus_frames_lost++;
        return -1;
    }
    /** Verify destination ID matches internal device ID*/
    if ((frame->dst_id != cfg->mbus_device_id) &&
        (frame->dst_id != MBUS_BROADCAST_ID)) {
        /** The received message was not intended for this device and should be
         * ignored.*/
        return -2;
    }
    /** If it's an ACK frame and msn equal the last transmit frame -> change the
     * TX_PENDING status */
    if (frame->frame_type == MBUS_ACK) {
        if (cfg->last_transmitted_frame.msn == frame->msn) {
            cfg->tx_status = MBUS_TX_SUCCESS;
        }
        return 1;
    }

    /** Verify msn number :*/
    /**    1. if rx_buff is full means data is not pull out, mbus cannot
       received more data.
                       -> just return 3 to ignore the frame. */
    if (cfg->rx_status == MBUS_RX_DATA_READY) {
        return 3;
    }

    /**    2. Reply an ACK for a good frame received. */
    int res = 0; // new data frame need process.
    if (frame->msn == cfg->msn_last) {
        mbus_frames_lost++;
        res = 2; // Ignore the frame.
    }
    cfg->msn_last =
        frame->msn; // Update last msn to avoid replicated data received.
    mbus_frame_t *f =
        mbus_frame_create(cfg, MBUS_FRAME_TYPE_ACK, frame->src_id, NULL, 0);
    mbus_frame_transmit(cfg, f);
    return res;
}

mbus_frame_t *mbus_get_frame_from_pool(mbus_config_t *cfg) {
    MBUS_NULL_CFG_CHECK(cfg);
    for (uint8_t i = 0; i < cfg->pool_size; i++) {
        if (cfg->mbus_mem_pool[i].allocated == 0) {
            cfg->mbus_mem_pool[i].allocated = true;
            return &(cfg->mbus_mem_pool[i]).frame;
        }
    }
    return (mbus_frame_t *)NULL;
}

mbus_config_t *mbus_init(uint32_t id, uint32_t baud) {
    mbus_config_t *cfg;
    /* Sanity check for variables */
    assert(id != MBUS_BROADCAST_ID);

    cfg = (mbus_config_t *)PORT_CALLOC(1, sizeof(mbus_config_t));
    if (cfg != NULL) {
        cfg->mbus_device_id = id;
        cfg->baud_rate = baud;
        /** Initialize memory pool */
        cfg->pool_size = MBUS_POOL_SIZE_DEFAULT;
        cfg->mbus_mem_pool = (mbus_pool_element_t *)PORT_CALLOC(
            cfg->pool_size, sizeof(mbus_pool_element_t));
        /** Allocate space on the heap for the rx_buffer.
         * This will be resized later with realloc()*/
        cfg->rx_buffer = (char *)PORT_CALLOC(1, sizeof(char));
        cfg->it_mbus_frame = mbus_frame_new(cfg);
        cfg->sys_mbus_frame = mbus_frame_new(cfg);
        cfg->tx_busy = 0;
        /** Set default the number of retries */
        cfg->tx_max_retries = 10;
        /** Calculate default tx_frametimeout using MBUS_FRAME_LENGTH_MAX,
         * 8 data bits + 1 stop bit * 1000 to convert to ms */
        cfg->tx_frame_timeout =
            MBUS_LATENCY_DEFAULT +
            ((8 + 1) * (MBUS_FRAME_LENGTH_MAX + MBUS_FRAME_LENGTH_ACK) * 1000) /
                cfg->baud_rate;
        cfg->mbus_user_error_handler = mbus_default_error_handler;
        cfg->mbus_enable = true;
    }
    return cfg;
}

int mbus_set_device_id(mbus_config_t *cfg, uint32_t id) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (cfg->mbus_device_id == MBUS_BROADCAST_ID) {
        cfg->mbus_user_error_handler(
            "Cannot initialize device with ID equal to MBUS_BROADCAST_ID.",
            __FILE__, __LINE__);
        return -1;
    } else {
        cfg->mbus_device_id = id;
        return 0;
    }
}

/** This function block until we have the result of tx */
int mbus_frame_send(mbus_config_t *cfg, mbus_frame_t *frame) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (cfg->mbus_hw_systick == NULL) {
        cfg->mbus_user_error_handler("No valid function registered for "
                                     "cfg->mbus_hw_systick.",
                                     __FILE__, __LINE__);
        return -1;
    }

    uint32_t time_start = cfg->mbus_hw_systick(), time = 0;
    uint8_t num_retries = 0;

    mbus_frame_transmit(cfg, frame);

    while (cfg->tx_status == MBUS_TX_PENDING) {
        time = cfg->mbus_hw_systick() - time_start;
        if (time >= cfg->tx_frame_timeout) {
            mbus_frame_retransmit(cfg);
            time_start = cfg->mbus_hw_systick();
            num_retries++;
        }
        if (num_retries > cfg->tx_max_retries) {
            cfg->tx_status = MBUS_TX_TIMEOUT;
            break;
        }
    }
    if (cfg->tx_status == MBUS_TX_SUCCESS) {
        return 0;
    } else {
        return -1;
    }
}

int mbus_data_send(mbus_config_t *cfg, uint8_t id, char *data, uint32_t len) {
    MBUS_NULL_CFG_CHECK(cfg);
    /** Don't use mbus, go directly to uart transmit. */
    if (cfg->mbus_enable == false) {
        while (cfg->tx_busy == 1) {
            ;
        }
        cfg->tx_busy = 1;
        /** Wait until previous frame has been transmitted and hardware
         * is ready to send */
        size_t time_start = cfg->mbus_hw_systick(), time = 0;
        while (!cfg->mbus_hw_ready()) {
            time = cfg->mbus_hw_systick() - time_start;
            if (time >= cfg->tx_frame_timeout) {
                if (cfg->mbus_hw_reset_peripheral != NULL) {
                    cfg->mbus_hw_reset_peripheral();
                }
                time_start = cfg->mbus_hw_systick();
            };
        }
        cfg->mbus_hw_transmit(data, len);
        cfg->tx_busy = 0;
        return 0;
    }

    /** Use mbus to transmit if mbus_enable is true */
    mbus_frame_t *frame;
    if (len <= MBUS_PAYLOAD_LENGTH) {
        /** Single frame transmit */
        frame = mbus_frame_create(cfg, MBUS_FRAME_TYPE_SINGLE, id, data, len);
        return mbus_frame_send(cfg, frame);
    } else {
        /** Multiple frame transmit */
        uint32_t n_frames = len / MBUS_PAYLOAD_LENGTH;
        uint32_t rem = len % MBUS_PAYLOAD_LENGTH;

        uint32_t offset = 0;
        /** Assemble and transmit all full-length frames */
        for (int i = 0; i < n_frames; i++) {
            if (rem == 0 && i == n_frames - 1) {
                frame = mbus_frame_create(cfg, MBUS_FRAME_TYPE_END_MULTIPLE, id,
                                          data + offset, MBUS_PAYLOAD_LENGTH);
                if (mbus_frame_send(cfg, frame) != 0)
                    return -1;
                offset += MBUS_PAYLOAD_LENGTH;
            } else {
                if (i == 0) {
                    frame = mbus_frame_create(
                        cfg, MBUS_FRAME_TYPE_START_MULTIPLE, id, data + offset,
                        MBUS_PAYLOAD_LENGTH);
                } else {
                    frame = mbus_frame_create(
                        cfg, MBUS_FRAME_TYPE_BODY_MULTIPLE, id, data + offset,
                        MBUS_PAYLOAD_LENGTH);
                }

                if (mbus_frame_send(cfg, frame) != 0)
                    return -1;
                offset += MBUS_PAYLOAD_LENGTH;
            }
        }
        if (rem == 0) {
            return 0;
        }
        /** Assemble and transmit last partial-length frame. */
        frame = mbus_frame_create(cfg, MBUS_FRAME_TYPE_END_MULTIPLE, id,
                                  data + offset, rem);
        return mbus_frame_send(cfg, frame);
    }
}

int mbus_frame_receive(mbus_config_t *cfg, mbus_frame_t *frame) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (frame == NULL) {
        cfg->mbus_user_error_handler("Null pointer passed as frame argument.",
                                     __FILE__, __LINE__);
        return -1;
    }

    if (cfg->rx_status == MBUS_RX_DATA_READY) {
        return -2;
    }

    if (frame->frame_type == MBUS_FRAME_TYPE_ACK) {
        return 0;
    }

    /** Check for the corrupted multiple frame sending*/
    if (frame->frame_type == MBUS_FRAME_TYPE_SINGLE ||
        frame->frame_type == MBUS_FRAME_TYPE_START_MULTIPLE) {
        cfg->rx_data_len = 0;
    }
    if (frame->frame_type == MBUS_FRAME_TYPE_END_MULTIPLE &&
        cfg->rx_data_len == 0) {
        return -3; // Missing START and BODY of MULTIPLE FRAME
    }
    if (frame->frame_type == MBUS_FRAME_TYPE_BODY_MULTIPLE &&
        cfg->rx_data_len == 0) {
        return -3; // Missing START of MULTIPLE FRAME
    }
    uint32_t new_size = cfg->rx_data_len + frame->len;
    /** Resize the rx buffer to hold the new data */
    cfg->rx_buffer = PORT_REALLOC(cfg->rx_buffer, new_size + 1);
    if (cfg->rx_buffer == NULL) {
        /** Error: Insufficient memory to allocate array */
        return -1;
    }
    /** Copy data from frame payload into rx buffer and update rx_buffer size*/
    memcpy((cfg->rx_buffer) + cfg->rx_data_len, frame->payload, frame->len);
    cfg->rx_data_len = new_size;
    if ((frame->frame_type == MBUS_FRAME_TYPE_SINGLE) ||
        (frame->frame_type == MBUS_FRAME_TYPE_END_MULTIPLE)) {
        /** Null-terminate string */
        cfg->rx_buffer[cfg->rx_data_len] = '\0';
        cfg->rx_status = MBUS_RX_DATA_READY;
    } else {
        cfg->rx_status = MBUS_RX_DATA_NOT_READY;
    }
    return frame->len;
}

int mbus_data_receive(mbus_config_t *cfg, char *buf) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (cfg->rx_status == MBUS_RX_DATA_READY) {
        int bytes = cfg->rx_data_len;
        memcpy(buf, cfg->rx_buffer, bytes);
        /** Update status variables now that data has been read */
        cfg->rx_data_len = 0;
        cfg->rx_status = MBUS_RX_DATA_NOT_READY;
        return bytes;
    } else if (cfg->rx_status == MBUS_RX_DATA_NOT_READY) {
        return 0;
    }
    return 0;
}

uint32_t mbus_data_is_available(mbus_config_t *cfg) {
    MBUS_NULL_CFG_CHECK(cfg);
    if (cfg->rx_status == MBUS_RX_DATA_READY) {
        return cfg->rx_data_len;
    } else {
        return 0;
    }
}

void mbus_default_error_handler(char *errstring, char *FILE,
                                unsigned int LINE) {
    while (1) {
        /** Wait here indefinitely. Analyze arguments for source of error. */
        ;
    }
}
