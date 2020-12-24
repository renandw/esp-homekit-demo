#include <stdio.h>
#include <stdlib.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include <button.h>
#include <toggle.h>
#include "ota-api.h"

// The GPIO pin that is connected to RELAY#1 on the board - Lampada 01 (direita)
const int relay_gpio_1 = 2;
// The GPIO pin that is connected to RELAY#2 on the board - Lampada 02 (central)
const int relay_gpio_2 = 12;
// The GPIO pin that is connected to RELAY#3 on the board - Lampada 03 (esquerda)
const int relay_gpio_3 = 13;
// The GPIO pin that is connected to RELAY#4 on the board - Lampada 04 (sala de estar)
const int relay_gpio_4 = 14;
// The GPIO pin that is connected to RELAY#5 on the board - Lampada 05 (sala de jantar)
const int relay_gpio_5 = 16;
// The GPIO pin that is connected to RELAY#5 on the board.
// const int relay_gpio_6 = 16;


// The GPIO pin that is connected to the button1 on the Board.
#define BUTTON_PIN 0
#ifndef BUTTON_PIN
#error BUTTON_PIN is not specified
#endif

// The GPIO pin that is connected to the header on the board(external switch).
#define TOGGLE_PIN_1 4
#ifndef TOGGLE_PIN_1
#error TOGGLE_PIN_1 is not specified
#endif

#define TOGGLE_PIN_2 5
#ifndef TOGGLE_PIN_2
#error TOGGLE_PIN_2 is not specified
#endif




void switch_on_1_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void switch_on_2_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void switch_on_3_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void switch_on_4_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void switch_on_5_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context);
// void switch_on_6_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context);


void relay_write_1(bool on) {
    gpio_write(relay_gpio_1, on ? 0 : 1);
}

void relay_write_2(bool on) {
    gpio_write(relay_gpio_2, on ? 0 : 1);
}

void relay_write_3(bool on) {
    gpio_write(relay_gpio_3, on ? 0 : 1);
}

void relay_write_4(bool on) {
    gpio_write(relay_gpio_4, on ? 0 : 1);
}

void relay_write_5(bool on) {
    gpio_write(relay_gpio_5, on ? 0 : 1);
}

// void relay_write_6(bool on) {
//    gpio_write(relay_gpio_6, on ? 0 : 1);
//}


void reset_configuration_task() {
    //Flash LED first before we start the reset
    printf("Resetting Wifi due to button long at GPIO %2d\n", BUTTON_PIN);
    wifi_config_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Resetting HomeKit Config\n");
    homekit_server_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Restarting\n");
    sdk_system_restart();
    vTaskDelete(NULL);
}

void reset_configuration() {
    printf("Resetting ESP12F configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}

homekit_characteristic_t switch_on_1 = HOMEKIT_CHARACTERISTIC_(
    ON, true, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_1_callback)
);

homekit_characteristic_t switch_on_2 = HOMEKIT_CHARACTERISTIC_(
    ON, true, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_2_callback)
);

homekit_characteristic_t switch_on_3 = HOMEKIT_CHARACTERISTIC_(
    ON, true, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_3_callback)
);

homekit_characteristic_t switch_on_4 = HOMEKIT_CHARACTERISTIC_(
    ON, true, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_4_callback)
);

homekit_characteristic_t switch_on_5 = HOMEKIT_CHARACTERISTIC_(
    ON, true, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_5_callback)
);

// homekit_characteristic_t switch_on_6 = HOMEKIT_CHARACTERISTIC_(
//     ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_6_callback)
// );

void gpio_init() {

    gpio_enable(relay_gpio_1, GPIO_OUTPUT);
    relay_write_1(switch_on_1.value.bool_value);

    gpio_enable(relay_gpio_2, GPIO_OUTPUT);
    relay_write_2(switch_on_2.value.bool_value);

    gpio_enable(relay_gpio_3, GPIO_OUTPUT);
    relay_write_3(switch_on_3.value.bool_value);

    gpio_enable(relay_gpio_4, GPIO_OUTPUT);
    relay_write_4(switch_on_4.value.bool_value);

    gpio_enable(relay_gpio_5, GPIO_OUTPUT);
    relay_write_5(switch_on_5.value.bool_value);

//    gpio_enable(relay_gpio_6, GPIO_OUTPUT);
//    relay_write_6(switch_on_6.value.bool_value);
    gpio_enable(TOGGLE_PIN_1, GPIO_INPUT);
    gpio_enable(TOGGLE_PIN_2, GPIO_INPUT);
    gpio_enable(BUTTON_PIN,  GPIO_INPUT);
}

void switch_on_1_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_1(switch_on_1.value.bool_value);
}

void switch_on_2_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_2(switch_on_2.value.bool_value);
}

void switch_on_3_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_3(switch_on_3.value.bool_value);
}

void switch_on_4_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_4(switch_on_4.value.bool_value);
}

void switch_on_5_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_5(switch_on_5.value.bool_value);
}

// void switch_on_6_callback(homekit_accessory_t *acc, homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
//     relay_write_6(switch_on_6.value.bool_value);
// }

void button_callback(button_event_t event, void* context) {
    switch (event) {
        case button_event_single_press:
          printf("SINGLE PRESS\n");
          switch_on_1.value.bool_value = !switch_on_1.value.bool_value;
          relay_write_1(switch_on_1.value.bool_value);
          homekit_characteristic_notify(&switch_on_1, switch_on_1.value);
          break;
        case button_event_double_press:
          printf("DOUBLE PRESS\n");
          switch_on_2.value.bool_value = !switch_on_2.value.bool_value;
          relay_write_2(switch_on_2.value.bool_value);
          homekit_characteristic_notify(&switch_on_2, switch_on_2.value);
          break;
        case button_event_tripple_press:
          printf("TRIPPLE PRESS\n");
          switch_on_3.value.bool_value = !switch_on_3.value.bool_value;
          relay_write_3(switch_on_3.value.bool_value);
          homekit_characteristic_notify(&switch_on_3, switch_on_3.value);
          break;
        case button_event_long_press:
          printf("LONG PRESS\n");
          reset_configuration();
          break;
        default:
            printf("Unknown button event: %d\n", event);
    }
}

void toggle_callback_1(bool high, void *context) {
    printf("toggle is %s\n", high ? "high" : "low");
    switch_on_4.value.bool_value = !switch_on_4.value.bool_value;
    relay_write_4(switch_on_4.value.bool_value);
    homekit_characteristic_notify(&switch_on_4, switch_on_4.value);
}

void toggle_callback_2(bool high, void *context) {
    printf("toggle is %s\n", high ? "high" : "low");
    switch_on_5.value.bool_value = !switch_on_5.value.bool_value;
    relay_write_5(switch_on_5.value.bool_value);
    homekit_characteristic_notify(&switch_on_5, switch_on_5.value);
}


void light_identify(homekit_value_t _value) {
    printf("Light identify\n");
}

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "5switch");
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "X");
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "1");
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         "Z");
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  "0.0.0");

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
          .id=1,
          .category=homekit_accessory_category_switch,
          .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
            &name,
            &manufacturer,
            &serial,
            &model,
            &revision,
            NULL
        },
        ),

        HOMEKIT_SERVICE(OUTLET, .primary=true, .characteristics=(homekit_characteristic_t*[]){
	    HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada Direita"),
	    &switch_on_1,
      HOMEKIT_CHARACTERISTIC(OUTLET_IN_USE, true),
      &ota_trigger,
            NULL
        }),

        HOMEKIT_SERVICE(OUTLET, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada Central"),
            &switch_on_2,
            HOMEKIT_CHARACTERISTIC(OUTLET_IN_USE, true),
            NULL
        }),

        HOMEKIT_SERVICE(OUTLET, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada Esquerda"),
            &switch_on_3,
            HOMEKIT_CHARACTERISTIC(OUTLET_IN_USE, true),
            NULL
        }),
        NULL,
      }),
      
      HOMEKIT_ACCESSORY(
            .id=2,
            .category=homekit_accessory_category_switch,
            .services=(homekit_service_t*[]){
              HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
              HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
              &name,  
              &manufacturer,
             &serial,
              &model,
              &revision,
              NULL
          },
          ),
        HOMEKIT_SERVICE(OUTLET, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada Sala de Estar"),
            &switch_on_4,
            HOMEKIT_CHARACTERISTIC(OUTLET_IN_USE, true),
            NULL
        }),
        HOMEKIT_SERVICE(OUTLET, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada Sala de Jantar"),
            &switch_on_5,
            HOMEKIT_CHARACTERISTIC(OUTLET_IN_USE, true),
            NULL
        }),
//         HOMEKIT_SERVICE(OUTLET, .characteristics=(homekit_characteristic_t*[]){
//             HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada 6"),
//             &switch_on_6,
//             HOMEKIT_CHARACTERISTIC(OUTLET_IN_USE, true),
//             NULL
//         }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "736-24-212",
    .setupId="7EN2", //ci=8
};

void on_wifi_ready() {
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "Area_Sala-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Area_Sala-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);

    char *serial_value = malloc(13);
    snprintf(serial_value, 13, "%02X%02X%02X%02X%02X%02X", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
    serial.value = HOMEKIT_STRING(serial_value);
}


void user_init(void) {
    uart_set_baud(0, 115200);
    homekit_server_init(&config);
    create_accessory_name();
    gpio_init();


    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                      &model.value.string_value,&revision.value.string_value);
    //c_hash=1; revision.value.string_value="0.0.1"; //cheat line
    config.accessories[0]->config_number=c_hash;
    

    button_config_t config = BUTTON_CONFIG(
        button_active_low,
        .long_press_time = 30000,
        .max_repeat_presses = 3,
    );
    if (button_create(BUTTON_PIN, config, button_callback, NULL)) {
        printf("Failed to initialize button\n");
    }
    if (toggle_create(TOGGLE_PIN_1, toggle_callback_1, NULL)) {
    printf("Failed to initialize toggle 1 \n");
    }
    if (toggle_create(TOGGLE_PIN_2, toggle_callback_2, NULL)) {
    printf("Failed to initialize toggle 2 \n");
    }

    

}
