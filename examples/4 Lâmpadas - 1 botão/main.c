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

// The GPIO pin that is connected to RELAY#1 on the board.
const int relay_gpio_1 = 4;
// The GPIO pin that is connected to RELAY#2 on the board.
const int relay_gpio_2 = 5;
// The GPIO pin that is connected to RELAY#1 on the board.
const int relay_gpio_3 = 13;
// The GPIO pin that is connected to RELAY#2 on the board.
const int relay_gpio_4 = 16;

// The GPIO pin that is connected to the LED on the ESP12F.
const int led_gpio = 2;

// The GPIO pin that is connected to the button on the Board.
#define BUTTON_PIN 0
#ifndef BUTTON_PIN
#error BUTTON_PIN is not specified
#endif



void lightbulb_on_1_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_2_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_3_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_4_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);


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

void led_write(bool on) {
    gpio_write(led_gpio, on ? 0 : 1);
}


void reset_configuration_task() {
    //Flash LED first before we start the reset
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
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

homekit_characteristic_t lightbulb_on_1 = HOMEKIT_CHARACTERISTIC_(
    ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_1_callback)
);

homekit_characteristic_t lightbulb_on_2 = HOMEKIT_CHARACTERISTIC_(
    ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_2_callback)
);

homekit_characteristic_t lightbulb_on_3 = HOMEKIT_CHARACTERISTIC_(
    ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_3_callback)
);

homekit_characteristic_t lightbulb_on_4 = HOMEKIT_CHARACTERISTIC_(
    ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_4_callback)
);


void gpio_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(false);

    gpio_enable(relay_gpio_1, GPIO_OUTPUT);
    relay_write_1(lightbulb_on_1.value.bool_value);

    gpio_enable(relay_gpio_2, GPIO_OUTPUT);
    relay_write_2(lightbulb_on_2.value.bool_value);

    gpio_enable(relay_gpio_3, GPIO_OUTPUT);
    relay_write_3(lightbulb_on_3.value.bool_value);

    gpio_enable(relay_gpio_4, GPIO_OUTPUT);
    relay_write_4(lightbulb_on_4.value.bool_value);

    gpio_enable(BUTTON_PIN,  GPIO_INPUT);
}

void lightbulb_on_1_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_1(lightbulb_on_1.value.bool_value);
}

void lightbulb_on_2_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_2(lightbulb_on_2.value.bool_value);
}

void lightbulb_on_3_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_3(lightbulb_on_3.value.bool_value);
}

void lightbulb_on_4_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_4(lightbulb_on_4.value.bool_value);
}


void button_callback(button_event_t event, void* context) {
    switch (event) {
        case button_event_single_press:
          printf("SINGLE PRESS\n");
          lightbulb_on_1.value.bool_value = !lightbulb_on_1.value.bool_value;
          relay_write_1(lightbulb_on_1.value.bool_value);
          homekit_characteristic_notify(&lightbulb_on_1, lightbulb_on_1.value);
          break;
        case button_event_double_press:
          printf("DOUBLE PRESS\n");
          lightbulb_on_2.value.bool_value = !lightbulb_on_2.value.bool_value;
          relay_write_2(lightbulb_on_2.value.bool_value);
          homekit_characteristic_notify(&lightbulb_on_2, lightbulb_on_2.value);  
          break;
        case button_event_tripple_press:
          printf("TRIPPLE PRESS\n");
          lightbulb_on_3.value.bool_value = !lightbulb_on_3.value.bool_value;
          relay_write_3(lightbulb_on_3.value.bool_value);
          homekit_characteristic_notify(&lightbulb_on_3, lightbulb_on_3.value);  
          break;
        case button_event_long_press:
          printf("LONG PRESS\n");
          lightbulb_on_4.value.bool_value = !lightbulb_on_4.value.bool_value;
          relay_write_4(lightbulb_on_4.value.bool_value);
          homekit_characteristic_notify(&lightbulb_on_4, lightbulb_on_4.value);
            //reset_configuration();
            break;
        default:
            printf("Unknown button event: %d\n", event);
    }
}

void light_identify_task(void *_args) {
    // We identify the ESP by Flashing it's LED.
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    led_write(false);
    vTaskDelete(NULL);
}

void light_identify(homekit_value_t _value) {
    printf("Light identify\n");
    xTaskCreate(light_identify_task, "Light identify", 128, NULL, 2, NULL);
}

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "4 Lâmpadas");
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, NULL);


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
          .id=1,
          .category=homekit_accessory_category_switch,
          .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "renandw"),
            HOMEKIT_CHARACTERISTIC(MODEL, "4chrelay"),
            &name,
            &serial,
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "3.0"),
            HOMEKIT_CHARACTERISTIC(HARDWARE_REVISION, "2.0"),
            NULL
        },
        ),

        HOMEKIT_SERVICE(LIGHTBULB, .primary=true, .characteristics=(homekit_characteristic_t*[]){
	    HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada 1"),
	    &lightbulb_on_1,
            NULL
        }),

        HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada 2"),
            &lightbulb_on_2,
            NULL
        }),

        HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada 3"),
            &lightbulb_on_3,
            NULL
        }),

        HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada 4"),
            &lightbulb_on_4,
            NULL
        }),
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
    homekit_server_init(&config);
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "4Lâmpadas-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "4Lâmpadas-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);

    char *serial_value = malloc(13);
    snprintf(serial_value, 13, "%02X%02X%02X%02X%02X%02X", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
    serial.value = HOMEKIT_STRING(serial_value);
}


void user_init(void) {
    uart_set_baud(0, 115200);
    create_accessory_name();
    wifi_config_init("4Lâmpadas", NULL, on_wifi_ready);
    gpio_init();


    button_config_t config = BUTTON_CONFIG(
        button_active_low,
        .long_press_time = 3000,
        .max_repeat_presses = 3,
    );


    if (button_create(BUTTON_PIN, config, button_callback, NULL)) {
        printf("Failed to initialize button\n");
    }
}
