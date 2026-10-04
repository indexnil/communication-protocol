#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "s_can.h"

// GPIO0 is the CAN H line
// Ground is the CAN L line
// Communicate a 0 dragging CAN H Low
// Communicate a 1 by leaving it floating

void on_receive(const s_can_frame_t* frame){
    printf("Frame | Id: %d Length: %d Data: ", frame->id, frame->length);
    for (int i = 0; i < frame->length; i++){
        printf("%c", frame->data[i]);
    }
    printf("\n\n");
}

void app_main(void)
{
    printf("Waiting for a second at boot...\n");
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
        
        while (true) {
            printf("Transmitting in 2 seconds!\n");

            vTaskDelay(2000 / portTICK_PERIOD_MS);

            printf("Transmitting...\n");

            char text[] = "Hello world!";

            s_can_frame_t frame = {
                .id = 123,
                .length = strlen(text),
            };
            
            memcpy(frame.data, text, frame.length);

            s_can_transmit(&frame);

            printf("Transmitted!\n");
        }
    } else {
        printf("Listening...\n");
    }

}
