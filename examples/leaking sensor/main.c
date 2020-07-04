#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>
#include <button.h>
#include <led_status.h>

#include "config.h"

#define SENSOR_PIN 5
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif

// The GPIO pin that is connected to the relay on the Sonoff S26
const int relay_gpio = RELAY_GPIO;
// The GPIO pin that is connected to the LED on the Sonoff S26
const int led_gpio = LED_GPIO;
// The GPIO pin that is oconnected to the button on the Sonoff S26
const int button_gpio = BUTTON_GPIO;


// one short blink every 3 seconds
led_status_pattern_t mode_normal = LED_STATUS_PATTERN({100, -2900});
// two short blinks every 3 seconds
led_status_pattern_t mode_connecting_to_wifi = LED_STATUS_PATTERN({100, -100, 100, -2700});
// long blink, long wait
led_status_pattern_t mode_no_wifi_config = LED_STATUS_PATTERN({2000, -2000});
// short blink, long blink, long wait
led_status_pattern_t mode_unpaired = LED_STATUS_PATTERN({100, -100, 800, -1000});

// three short blinks
led_status_pattern_t mode_reset = LED_STATUS_PATTERN({100, -100, 100, -100, 100, -4500});
// three series of two short blinks
led_status_pattern_t mode_identify = LED_STATUS_PATTERN({ 100, -100, 100, -350, 100, -100, 100, -350, 100, -100, 100, -2500});

static led_status_t status;


void button_callback(button_event_t event, void *_context);



void reset_configuration_task() {
    // Flash the LED first before we start the reset
    led_status_signal(status, &mode_reset);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    printf("Resetting Wifi Config\n");
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
    printf("Resetting Sonoff configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}


void button_callback(button_event_t event, void *_context) {
    switch (event) {
        case button_event_long_press:
            reset_configuration();
            break;

        default:
            printf("Unknown button event: %d\n", event);
    }
}


void skimmer_sensor_identify(homekit_value_t _value) {
    printf("Skimmer Control identify\n");
    led_status_signal(status, &mode_identify);
}

homekit_characteristic_t leak_event = HOMEKIT_CHARACTERISTIC_(LEAK_DETECTED, 0);


void skimmer_sensor_task() {
    while (true) {
        gpio_set_pullup(SENSOR_PIN, true, true);
        gpio_set_interrupt(SENSOR_PIN, GPIO_INTTYPE_EDGE_ANY, NULL);
       /* printf(">>>>> %i , PIN: %i ",gpio_read(SENSOR_PIN),SENSOR_PIN); */
        if (gpio_read(SENSOR_PIN)) {
         /*   printf("Libre\n"); */
            homekit_characteristic_notify(&leak_event, HOMEKIT_UINT8(0));
        } else {
            /* printf("Fuga!\n"); */
            homekit_characteristic_notify(&leak_event, HOMEKIT_UINT8(1));
        }

        vTaskDelay(6000 / portTICK_PERIOD_MS);
    }
}

void skimmer_sensor_init() {
    xTaskCreate(skimmer_sensor_task, "Leak Sensor", 256, NULL, 2, NULL);
}

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Controlador de Skimmer");
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, NULL);

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id=1,
        .category=homekit_accessory_category_sensor,
        .services=(homekit_service_t*[]) {
            HOMEKIT_SERVICE(
                ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]) {
                    &name,
                    HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Coderty"),
                    &serial,
                    HOMEKIT_CHARACTERISTIC(MODEL, "CLD1"),
                    HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.3"),
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, skimmer_sensor_identify),
                    NULL
                },
            ),
            HOMEKIT_SERVICE(
                LEAK_SENSOR,
                .primary=true,
                .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Controlador de Skimmer"),
                    &leak_event,
                    NULL
                },
            ),
            NULL
        },
    ),
    NULL
};

void on_homekit_event(homekit_event_t event) {
    if (event == HOMEKIT_EVENT_PAIRING_ADDED) {
        led_status_set(status, &mode_normal);
    } else if (event == HOMEKIT_EVENT_PAIRING_REMOVED) {
        if (!homekit_is_paired()) {
            led_status_set(status, &mode_unpaired);
        }
    }
}

static bool initialized = false;
homekit_server_config_t config = {
    .accessories = accessories,
    .password = ACCESSORY_SETUP_CODE,
    .on_event = on_homekit_event,
};

void on_wifi_config_event(wifi_config_event_t event) {
    if (event == WIFI_CONFIG_CONNECTED && !initialized) {
        if (!initialized) {
            homekit_server_init(&config);

            initialized = true;

            led_status_set(status, homekit_is_paired() ? &mode_normal : &mode_unpaired);
        }
    }
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "Controlador de Skimmer-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Controlador de Skimmer-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
    
    char *serial_value = malloc(13);
    snprintf(serial_value, 13, "%02X%02X%02X%02X%02X%02X", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
    serial.value = HOMEKIT_STRING(serial_value);
}

bool wifi_is_configured() {
    char *ssid = NULL;
    wifi_config_get(&ssid, NULL);
    if (!ssid)
        return false;

    free(ssid);
    return true;
}


void user_init(void) {
    uart_set_baud(0, 115200);
    status = led_status_init(led_gpio, LED_ACTIVE_LEVEL);

   create_accessory_name();

   button_config_t button_config = BUTTON_CONFIG(
       (BUTTON_ACTIVE_LEVEL) ? button_active_high : button_active_low,
       .max_repeat_presses=2,
       .long_press_time=5000,
   );
   if (button_create(button_gpio, button_config, button_callback, NULL)) {
       printf("Failed to initialize button\n");
   }

   wifi_config_init2(WIFI_AP_NAME, WIFI_AP_PASSWORD, on_wifi_config_event);

   led_status_set(status, wifi_is_configured() ? &mode_connecting_to_wifi : &mode_no_wifi_config);
    skimmer_sensor_init();
}
