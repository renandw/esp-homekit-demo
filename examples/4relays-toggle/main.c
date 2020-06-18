#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include <button.h>
#include <toggle.h>

// The GPIO pin that is connected to TRIAC#1 on the board.
const int triac_gpio_1 = 1;
// The GPIO pin that is connected to TRIAC#2 on the board.
const int triac_gpio_2 = 3;
// The GPIO pin that is connected to TRIAC#1 on the board.
const int triac_gpio_3 = 13;
// The GPIO pin that is connected to TRIAC#2 on the board.
const int triac_gpio_4 = 16;

// The GPIO pin that is connected to the LED on the ESP12F.
const int led_gpio = 2;

// The GPIO pin that is connected to the button on the Board.
#define BUTTON_PIN 0
#ifndef BUTTON_PIN
#error BUTTON_PIN is not specified
#endif

// The GPIO pin that is connected to the header on the board(external switch).
#define TOGGLE_PIN_1 5
#ifndef TOGGLE_PIN_1
#error TOGGLE_PIN_1 is not specified
#endif

#define TOGGLE_PIN_2 4
#ifndef TOGGLE_PIN_2
#error TOGGLE_PIN_2 is not specified
#endif

#define TOGGLE_PIN_3 14
#ifndef TOGGLE_PIN_3
#error TTOGGLE_PIN_3 is not specified
#endif

#define TOGGLE_PIN_4 12
#ifndef TOGGLE_PIN_4
#error TOGGLE_PIN_4 is not specified
#endif



void lightbulb_on_1_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_2_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_3_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_4_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);


void triac_write_1(bool on) {
    gpio_write(triac_gpio_1, on ? 0 : 1);
}

void triac_write_2(bool on) {
    gpio_write(triac_gpio_2, on ? 0 : 1);
}

void triac_write_3(bool on) {
    gpio_write(triac_gpio_3, on ? 0 : 1);
}

void triac_write_4(bool on) {
    gpio_write(triac_gpio_4, on ? 0 : 1);
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

    gpio_enable(triac_gpio_1, GPIO_OUTPUT);
    triac_write_1(lightbulb_on_1.value.bool_value);

    gpio_enable(triac_gpio_2, GPIO_OUTPUT);
    triac_write_2(lightbulb_on_2.value.bool_value);

    gpio_enable(triac_gpio_3, GPIO_OUTPUT);
    triac_write_3(lightbulb_on_3.value.bool_value);

    gpio_enable(triac_gpio_4, GPIO_OUTPUT);
    triac_write_4(lightbulb_on_4.value.bool_value);

    gpio_enable(BUTTON_PIN,  GPIO_INPUT);
    gpio_enable(TOGGLE_PIN_1, GPIO_INPUT);
    gpio_enable(TOGGLE_PIN_2, GPIO_INPUT);
    gpio_enable(TOGGLE_PIN_3, GPIO_INPUT);
    gpio_enable(TOGGLE_PIN_4, GPIO_INPUT);
}

void lightbulb_on_1_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    triac_write_1(lightbulb_on_1.value.bool_value);
}

void lightbulb_on_2_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    triac_write_2(lightbulb_on_2.value.bool_value);
}

void lightbulb_on_3_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    triac_write_3(lightbulb_on_3.value.bool_value);
}

void lightbulb_on_4_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    triac_write_4(lightbulb_on_4.value.bool_value);
}


void button_callback(button_event_t event, void* context) {
    switch (event) {
        case button_event_long_press:
            reset_configuration();
            break;
        default:
            printf("Unknown button event: %d\n", event);
    }
}

void toggle_callback_1(bool high, void *context) {
    printf("Toggling TRIAC 1 due to switch at GPIO 3");
    lightbulb_on_1.value.bool_value = !lightbulb_on_1.value.bool_value;
    triac_write_1(lightbulb_on_1.value.bool_value);
    homekit_characteristic_notify(&lightbulb_on_1, lightbulb_on_1.value);
}

void toggle_callback_2(bool high, void *context) {
    printf("Toggling TRIAC 2 due to switch at GPIO 13");
    lightbulb_on_2.value.bool_value = !lightbulb_on_2.value.bool_value;
    triac_write_2(lightbulb_on_2.value.bool_value);
    homekit_characteristic_notify(&lightbulb_on_2, lightbulb_on_2.value);
}

void toggle_callback_3(bool high, void *context) {
    printf("Toggling TRIAC 3 due to switch at GPIO 2");
    lightbulb_on_3.value.bool_value = !lightbulb_on_3.value.bool_value;
    triac_write_3(lightbulb_on_3.value.bool_value);
    homekit_characteristic_notify(&lightbulb_on_3, lightbulb_on_3.value);
}

void toggle_callback_4(bool high, void *context) {
    printf("Toggling TRIAC 4 due to switch at GPIO 16");
    lightbulb_on_4.value.bool_value = !lightbulb_on_4.value.bool_value;
    triac_write_4(lightbulb_on_4.value.bool_value);
    homekit_characteristic_notify(&lightbulb_on_4, lightbulb_on_4.value);
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

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "ESP12F 4CH");

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
          .id=1, 
          .category=homekit_accessory_category_switch, 
          .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "LKAI"),
            HOMEKIT_CHARACTERISTIC(MODEL, "ESP 4CH"),
            &name,
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "mps"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1"),
            HOMEKIT_CHARACTERISTIC(HARDWARE_REVISION, "V2"),
            NULL
        },
        ),

        HOMEKIT_SERVICE(LIGHTBULB, .primary=true, .characteristics=(homekit_characteristic_t*[]){
	    HOMEKIT_CHARACTERISTIC(NAME, "Light 1"),
	    &lightbulb_on_1,
            NULL
        }),
        
        HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Light 2"),
            &lightbulb_on_2,
            NULL
        }),

        HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Light 3"),
            &lightbulb_on_3,
            NULL
        }),

        HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Light 4"),
            &lightbulb_on_4,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "201-01-985"
//    .password = "190-11-978"    //changed tobe valid
//    .password = "111-11-111"    //default easy
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "ESP12F 4CH %02X:%02X:%02X",
            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "ESP12F 4CH %02X:%02X:%02X",
            macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
}


void user_init(void) {
    uart_set_baud(0, 115200);
    create_accessory_name();
    wifi_config_init("4LIGHT", NULL, on_wifi_ready);
    gpio_init();


    button_config_t config = BUTTON_CONFIG(
        button_active_high,
        .long_press_time = 1000,
        .max_repeat_presses = 3,
    );

// 4000 = 4s long press => 15000/3 = 1s medium press
    if (button_create(BUTTON_PIN, config, button_callback, NULL)) {
        printf("Failed to initialize button\n");
    }

    if (toggle_create(TOGGLE_PIN_1, toggle_callback_1, NULL)) {
        printf("Failed to initialize toggle 1 \n");
    }

    if (toggle_create(TOGGLE_PIN_2, toggle_callback_2, NULL)) {
        printf("Failed to initialize toggle 2 \n");
    }

    if (toggle_create(TOGGLE_PIN_3, toggle_callback_3, NULL)) {
        printf("Failed to initialize toggle 3 \n");
    }

    if (toggle_create(TOGGLE_PIN_4, toggle_callback_4, NULL)) {
        printf("Failed to initialize toggle 4 \n");
    }
}