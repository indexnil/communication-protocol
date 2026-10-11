#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "time.h"

#include "s_can.h"

#define CAN_H_GPIO GPIO_NUM_0
#define RECEIVE_FRAME_SLOTS 8

#define BIT_DURATION_US 1000

#define MAX_IDENTICAL_BITS 5
#define MIN_IDLE_BITS 6

static const char* TAG = "s_can";

static TaskHandle_t s_can_receive_task_handle = NULL;
static BaseType_t isr_woke_receive_task = pdFALSE;

static s_can_frame_t receive_frames[RECEIVE_FRAME_SLOTS];
static int receive_frame_slot = 0;

static int frame_id_index = 0;
static int frame_length_index = 0;
static int receive_frame_data_index = 0;
static uint8_t frame_byte = 0;
static uint8_t frame_byte_index = 0;
static int rx_previous_bit = 0;
static int rx_identical_bit_counter = 0;

// On the transmit end
static int tx_previous_bit = 0;
static int tx_identical_bit_counter = 0;

// Frames start with a 1 (this one doesnt have a state), then 0 (start measuring the time), and then 1 again (at this point we calculate the bit duration from the time).
// Frames always end with a 0, this is the FRAME_END state, that then goes back to 1 after one bit duration.
typedef enum {
    NONE,
    FRAME_BEGIN_LOW,
    FRAME_BEGIN_HIGH,
    ID,
    LENGTH,
    DATA,
    FRAME_END,
} frame_receive_state_t;
static frame_receive_state_t frame_receive_state = NONE;

static s_can_receive_callback_t receive_callback = NULL;

static uint64_t last_bit_edge_time;

static uint32_t bit_duration;

typedef enum {
    INVALID_FRAME_END,
    FRAME_BEGIN_SIGNAL_NOISE,
    SIGNAL_NOISE,
    BIT_STUFFING_INCORRECT,
    NUM_ERROR_REASONS,
} frame_error_reason_t;

static inline const char* frame_error_reason_to_string(frame_error_reason_t reason){
    switch (reason){
        case INVALID_FRAME_END:        return "INVALID_FRAME_END";
        case FRAME_BEGIN_SIGNAL_NOISE: return "FRAME_BEGIN_SIGNAL_NOISE";
        case SIGNAL_NOISE:             return "SIGNAL_NOISE";
        case BIT_STUFFING_INCORRECT:   return "BIT_STUFFING_INCORRECT";
        default:                       return "Unknown error";
    }
}
static int frame_errors[NUM_ERROR_REASONS] = {};

static void s_can_transmit_raw_bit(uint32_t level){
    gpio_set_level(CAN_H_GPIO, level);
    esp_rom_delay_us(BIT_DURATION_US);
}

static void s_can_transmit_bit(uint32_t level){
    s_can_transmit_raw_bit(level);
    
    // Bit stuffing first
    if (tx_previous_bit == level){
        tx_identical_bit_counter++;
    } else {
        tx_identical_bit_counter = 1;
        tx_previous_bit = level;
    }

    if (tx_identical_bit_counter == MAX_IDENTICAL_BITS){
        s_can_transmit_raw_bit(!level);
        tx_identical_bit_counter = 1;
        tx_previous_bit = !level;
    }
}

static void s_can_transmit_byte(uint8_t byte){
    for (int i = 0; i < 8; i++){
        // Remember to take the most significant bit first
        s_can_transmit_bit((byte >> (7-i)) & 1);
    }
}

// Yields until everything is transmitted
// For now, assume no one else is transmitting.
void s_can_transmit(const s_can_frame_t* frame){

    // New frame so reset this + we dont want it in the frame start sequence
    tx_identical_bit_counter = 0;
    tx_previous_bit = 0;

    // Frame start sequence
    s_can_transmit_bit(1);
    s_can_transmit_bit(0);
    s_can_transmit_bit(1);

    // Id
    s_can_transmit_byte(frame->id);

    // Length
    s_can_transmit_byte(frame->length);

    // Data
    for (uint8_t i = 0; i < frame->length; i++){
        s_can_transmit_byte(frame->data[i]);
    }

    // We dont want bit stuffing in the frame end sequence
    // (edit) bit stuffing on the frame end is probably fine
    //tx_identical_bit_counter = 0;

    // Frame end
    s_can_transmit_bit(0);

    // Go back to floating now
    gpio_set_level(CAN_H_GPIO, 1);

    // Add the min idle bit gap so that the receiver knows for sure when a new frame starts
    esp_rom_delay_us(MIN_IDLE_BITS * BIT_DURATION_US);

}

esp_err_t s_can_set_receive_callback(s_can_receive_callback_t callback){
    receive_callback = callback;
    return ESP_OK;
}

static void IRAM_ATTR s_can_abort_frame(frame_error_reason_t reason){
    frame_id_index = 0;

    frame_receive_state = NONE;
    frame_errors[reason]++;
}

static void IRAM_ATTR s_can_on_bit(int level){
    if (rx_identical_bit_counter == MAX_IDENTICAL_BITS){
        if (level == rx_previous_bit) {
            s_can_abort_frame(BIT_STUFFING_INCORRECT);
        } else {
            // This bit is a stuffed bit, reset the counter and return
            rx_identical_bit_counter = 1;
            rx_previous_bit = level;
        }
        return;
    }

    if (level == rx_previous_bit) {
        rx_identical_bit_counter++;
    } else {
        rx_identical_bit_counter = 1;
        rx_previous_bit = level;
    }

    switch (frame_receive_state){
        case ID:            
            frame_id_index+=1;

            // Use frame_id_index to detect new frames beginning.
            if (frame_id_index == 1){
                // New frame, reset counters
                frame_length_index = 0;
                frame_byte_index = 0;
                receive_frame_data_index = 0;
                receive_frames[receive_frame_slot].id = 0;

            } else if (frame_id_index == 8){
                frame_id_index = 0; // This one is important to reset to 0 so we can detect a new frame

                frame_receive_state = LENGTH;
                receive_frames[receive_frame_slot].length = 0;
            }

            receive_frames[receive_frame_slot].id = receive_frames[receive_frame_slot].id << 1;
            receive_frames[receive_frame_slot].id |= level;


            break;
        case LENGTH:
            
            receive_frames[receive_frame_slot].length = receive_frames[receive_frame_slot].length << 1;
            receive_frames[receive_frame_slot].length |= level;
            frame_length_index+=1;

            if (frame_length_index == 8){
                if (receive_frames[receive_frame_slot].length > 0){
                    frame_receive_state = DATA;
                } else {
                    frame_receive_state = FRAME_END;
                }
                break;
            }

            break;
        case DATA:            
            frame_byte = frame_byte << 1;
            frame_byte |= level;
            frame_byte_index+=1;

            if (frame_byte_index == 8){
                receive_frames[receive_frame_slot].data[receive_frame_data_index] = frame_byte;
                receive_frame_data_index++;

                frame_byte_index = 0;

                if (receive_frame_data_index >= receive_frames[receive_frame_slot].length){
                    receive_frame_data_index = 0;

                    frame_receive_state = FRAME_END;
                }
            }
            break;
        case FRAME_END:
            if (level == 0){
                frame_receive_state = NONE;
                receive_frame_slot++;
                if (receive_frame_slot >= RECEIVE_FRAME_SLOTS){
                    receive_frame_slot = 0;
                }
                vTaskNotifyGiveFromISR(s_can_receive_task_handle, &isr_woke_receive_task);
            } else {
                // we are probably about to receive a bunch of junk bits after this because the frame didnt end properly
                s_can_abort_frame(INVALID_FRAME_END);
            }
            break;
        default:
            break;
    }
}

static void IRAM_ATTR s_can_on_interrupt(void* arg){
    int level = gpio_get_level(CAN_H_GPIO);

    uint64_t current_time = esp_timer_get_time();

    isr_woke_receive_task = pdFALSE;

    switch (frame_receive_state){
        case NONE: {
            if (level == 0){
                // If the bus is idle for more than an hour this uint32 can overflow
                uint32_t idle = (uint32_t)(current_time-last_bit_edge_time);
                if ((idle + BIT_DURATION_US/2)/BIT_DURATION_US >= MIN_IDLE_BITS){
                    frame_receive_state = FRAME_BEGIN_LOW;
                }
            }
            break;
        }
        case FRAME_BEGIN_LOW: {            
            if (level == 1){
                bit_duration = (uint32_t)(current_time - last_bit_edge_time);
                if (bit_duration != 0){
                    frame_receive_state = FRAME_BEGIN_HIGH;
                } else {
                    frame_receive_state = NONE;
                }
            }
            break;
        } 
        case FRAME_BEGIN_HIGH: {
            if (level == 0){
                frame_receive_state = ID;

                // Take into account if the ID starts high as well.
                uint32_t delta = (uint32_t)(current_time - last_bit_edge_time);
                int bit_count = (delta + bit_duration/2) / bit_duration;
                if (bit_count == 0){
                    // Noise consisting of less than half a bit, might be safe to ignore, but might also not be safe to ignore.
                    s_can_abort_frame(FRAME_BEGIN_SIGNAL_NOISE);
                    break;
                }

                // We've got one high bit at this point, and from now on, s_can_on_bit will update the counter
                rx_identical_bit_counter = 1;
                rx_previous_bit = 1;

                for (int i = 1; i < bit_count; i++){
                    // The level we had previously is the one that matters here
                    s_can_on_bit(1);
                }

            }
            break;
        }
        default: {
            uint32_t delta = (uint32_t)(current_time - last_bit_edge_time);
            int bit_count = (delta + bit_duration/2) / bit_duration;
            if (bit_count == 0){
                // Noise consisting of less than half a bit, might be safe to ignore, but might also not be safe to ignore.
                s_can_abort_frame(SIGNAL_NOISE);
                break;
            }
            for (int i = 0; i < bit_count; i++){
                // The level we had previously is the one that matters here
                s_can_on_bit(!level);
            }

            break;
        }
    }

    last_bit_edge_time = current_time;

    portYIELD_FROM_ISR(isr_woke_receive_task);
}


static void s_can_receive_task(void* arg){
    int read_slot = 0;
    int overflows = 0; // Just for logging
    int receive_count = 0; // Just for logging, includes frames that the interrupt handed us but that got overwritten before the callback had time to run.
    int pending_high_watermark = 0; // Just for logging
    while (true) {
        int pending = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(2000));
        if (pending == 0){
            for (int i = 0; i < NUM_ERROR_REASONS; i++){
                ESP_LOGI(TAG, "%s: %d", frame_error_reason_to_string(i), frame_errors[i]);
            }
            ESP_LOGI(TAG, "No frame in 2s, rx state: %d, bit_duration: %" PRIu32, frame_receive_state, bit_duration);
            continue;
        }
        receive_count++;

        // pending > RECEIVE_FRAME_SLOTS means our data has been overwritten already, >= means that an interrupt can happen at any moment and write over the data for the current callback.
        if (pending < RECEIVE_FRAME_SLOTS){
            // Just threw this in here, should maybe be moved somewhere else
            for (int i = 0; i < NUM_ERROR_REASONS; i++){
                if (frame_errors[i] == 0){
                    continue;
                }
                const char* reason = frame_error_reason_to_string(i);
                ESP_LOGI(TAG, "%s: %d", reason, frame_errors[i]);
            }
            ESP_LOGI(TAG, "Received frame. bit_duration: %" PRIu32 "us | overflows: %d | receive count: %d", bit_duration, overflows, receive_count);
            if (receive_callback != NULL){
                receive_callback(&receive_frames[read_slot]);
            }
        } else {
            // Overflow, skip the callback because the data currently sitting there is intended for a future callback.
            overflows++;
        }

        // Take -1 to subtract the frame we just cleared
        if (pending-1 > pending_high_watermark){
            pending_high_watermark = pending - 1;
        }

        read_slot+=1;
        if (read_slot >= RECEIVE_FRAME_SLOTS){
            read_slot = 0;
        }
    }
}

esp_err_t s_can_init(){
    gpio_config_t can_h_config = {
        .pin_bit_mask = (1ULL << CAN_H_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = false, // External pull up
        .pull_down_en = false,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    gpio_set_level(CAN_H_GPIO, 1); // Is 0 otherwise
    gpio_config(&can_h_config);

    xTaskCreate(s_can_receive_task, "s_can_receive_task", 4096, NULL, 10, &s_can_receive_task_handle);
    if (s_can_receive_task_handle == NULL){
        ESP_LOGE(TAG, "Failed to create s_can_receive_task");
        return ESP_ERR_NO_MEM;
    }

    gpio_install_isr_service(0);
    ESP_ERROR_CHECK(gpio_isr_handler_add(CAN_H_GPIO, s_can_on_interrupt, NULL));

    return ESP_OK;
}