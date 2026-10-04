#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t id;
    uint8_t length;
    uint8_t data[255];
} s_can_frame_t;

typedef void (*s_can_receive_callback_t)(const s_can_frame_t* frame);

void s_can_transmit(const s_can_frame_t* frame);

esp_err_t s_can_set_receive_callback(s_can_receive_callback_t callback);

esp_err_t s_can_init();
