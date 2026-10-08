#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_log.h"

#include "s_can.h"

// GPIO0 is the CAN H line
// Ground is the CAN L line
// Communicate a 0 dragging CAN H Low
// Communicate a 1 by leaving it floating

static const char* TAG = "main";

void on_receive(const s_can_frame_t* frame){
    printf("Frame | Id: %d | Length: %d | Data: ", frame->id, frame->length);
    for (int i = 0; i < frame->length; i++){
        printf("%c", frame->data[i]);
    }
    printf("\n\n");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Waiting for a second at boot...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    s_can_init();
    s_can_set_receive_callback(on_receive);

    // Decide if we should transmit or not by checking GPIO5. If it is held down, transmit.
    gpio_config_t check_config = {
        .pin_bit_mask = (1ULL << GPIO_NUM_5),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&check_config);

    if (gpio_get_level(GPIO_NUM_5) == 0){
        // Transmit!
        
        int transmit_count = 0;
        while (true) {
            // 10ms is the lowest vTaskDelay can go without config changes
            // 34 bits in a full "Hello world!" frame
            vTaskDelay(10 / portTICK_PERIOD_MS);

            //ESP_LOGI(TAG, "Transmitting...");

            char text[] = "Hello world!";

            s_can_frame_t frame = {
                .id = 123,
                .length = strlen(text)+1,
            };
            
            memcpy(frame.data, text, frame.length);

            s_can_transmit(&frame);
            transmit_count++;

            ESP_LOGI(TAG, "Transmit count: %d", transmit_count);
        }
    } else {
        ESP_LOGI(TAG, "Listening...");
    }

}
