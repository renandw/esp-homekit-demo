/*
 * Implemenetation of lock mechanism accessory for a magnet lock.
 * When unlocked, it changes relay state (unlocks) for configured period and then
 * changes it back.
 */

#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <etstimer.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include "contact_sensor.h"
#include "button.h"

// The GPIO pin that is connected to a relay
const int relay_gpio = 12;
// The GPIO pin that is connected to a LED
// const int led_gpio = 13;
const int led_gpio = 2;
// The GPIO pin that is connected to a button
// const int button_gpio = 0;

#define BUTTON_PIN 0
#ifndef BUTTON_PIN
#error BUTTON_PIN is not specified
#endif

#define REED_PIN 5
#ifndef REED_PIN
#error REED_PIN is not specified
#endif

// Timeout in seconds to open lock for
const int unlock_period = 5;  // 5 seconds
// Which signal to send to relay to open the lock (0 or 1)
const int relay_open_signal = 1;

void lock_lock();
void lock_unlock();

void button_identify(homekit_value_t _value) {
    printf("Button identify\n");
}

void door_identify(homekit_value_t _value) {
    printf("Door identifying\n");
}

void relay_write(int value) {
    gpio_write(relay_gpio, value ? 1 : 0);
}

void led_write(bool on) {
    gpio_write(led_gpio, on ? 0 : 1);
}

void reset_configuration_task() {
    //Flash the LED first before we start the reset
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

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
    printf("Resetting configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}

void gpio_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(false);

    gpio_enable(relay_gpio, GPIO_OUTPUT);
    relay_write(!relay_open_signal);
}


homekit_characteristic_t button_event = HOMEKIT_CHARACTERISTIC_(PROGRAMMABLE_SWITCH_EVENT, 0);

void button_callback(button_event_t event, void* context) {
    switch (event) {
        case button_event_single_press:
            printf("single press\n");
            homekit_characteristic_notify(&button_event, HOMEKIT_UINT8(0));
            break;
        case button_event_double_press:
            printf("double press\n");
            homekit_characteristic_notify(&button_event, HOMEKIT_UINT8(1));
            break;
        case button_event_long_press:
            printf("long press\n");
            homekit_characteristic_notify(&button_event, HOMEKIT_UINT8(2));
            break;
        default:
            printf("unknown button event: %d\n", event);
    }
}

void lock_identify_task(void *_args) {
    // We identify the Sonoff by Flashing it's LED.
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(false);

    vTaskDelete(NULL);
}

void lock_identify(homekit_value_t _value) {
    printf("Lock identify\n");
    xTaskCreate(lock_identify_task, "Lock identify", 128, NULL, 2, NULL);
}


typedef enum {
    lock_state_unsecured = 0,
    lock_state_secured = 1,
    lock_state_jammed = 2,
    lock_state_unknown = 3,
} lock_state_t;



homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Lock");

homekit_characteristic_t lock_current_state = HOMEKIT_CHARACTERISTIC_(
    LOCK_CURRENT_STATE,
    lock_state_unknown,
);

void lock_target_state_setter(homekit_value_t value);

homekit_characteristic_t lock_target_state = HOMEKIT_CHARACTERISTIC_(
    LOCK_TARGET_STATE,
    lock_state_secured,
    .setter=lock_target_state_setter,
);

void lock_target_state_setter(homekit_value_t value) {
    lock_target_state.value = value;

    if (value.int_value == 0) {
        lock_unlock();
    } else {
        lock_lock();
    }
}

void lock_control_point(homekit_value_t value) {
    // Nothing to do here
}


ETSTimer lock_timer;

void lock_lock() {
    sdk_os_timer_disarm(&lock_timer);

    relay_write(!relay_open_signal);
    led_write(false);

    if (lock_current_state.value.int_value != lock_state_secured) {
        lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
        homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
    }
}

void lock_timeout() {
    if (lock_target_state.value.int_value != lock_state_secured) {
        lock_target_state.value = HOMEKIT_UINT8(lock_state_secured);
        homekit_characteristic_notify(&lock_target_state, lock_target_state.value);
    }

    lock_lock();
}

void lock_init() {
    lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);

    sdk_os_timer_disarm(&lock_timer);
    sdk_os_timer_setfn(&lock_timer, lock_timeout, NULL);
}

void lock_unlock() {
    relay_write(relay_open_signal);
    led_write(true);

    lock_current_state.value = HOMEKIT_UINT8(lock_state_unsecured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);

}


homekit_value_t door_state_getter() {
    printf("Door state was requested (%s).\n", contact_sensor_state_get(REED_PIN) == CONTACT_OPEN ? "open" : "closed");
    return HOMEKIT_UINT8(contact_sensor_state_get(REED_PIN) == CONTACT_OPEN ? 1 : 0);
}

/**
 * The sensor characteristic as global variable.
 **/
homekit_characteristic_t door_open_characteristic = HOMEKIT_CHARACTERISTIC_(CONTACT_SENSOR_STATE, 0,
    .getter=door_state_getter,
    .setter=NULL,
    NULL
);

/**
 * Called (indirectly) from the interrupt handler to notify the client of a state change.
 **/
void contact_sensor_callback(uint8_t gpio, contact_sensor_state_t state) {
    switch (state) {
        case CONTACT_OPEN:
        case CONTACT_CLOSED:
            printf("Pushing contact sensor state '%s'.\n", state == CONTACT_OPEN ? "open" : "closed");
            homekit_characteristic_notify(&door_open_characteristic, door_state_getter());
            break;
        default:
            printf("Unknown contact sensor event: %d\n", state);
    }
}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_door_lock, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            &name,
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Basic"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, lock_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MECHANISM, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lock"),
            &lock_current_state,
            &lock_target_state,
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MANAGEMENT, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(LOCK_CONTROL_POINT,
                .setter=lock_control_point
            ),
            HOMEKIT_CHARACTERISTIC(VERSION, "1"),
            NULL
        }),
        NULL
    }),

      HOMEKIT_ACCESSORY(
        .id=2,
        .category=homekit_accessory_category_sensor,
        .services=(homekit_service_t*[]) {
          HOMEKIT_SERVICE(
              ACCESSORY_INFORMATION,
              .characteristics=(homekit_characteristic_t*[]) {
                  HOMEKIT_CHARACTERISTIC(NAME, "Contact Sensor"),
                  HOMEKIT_CHARACTERISTIC(MANUFACTURER, "ObjP"),
                  HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "2012345"),
                  HOMEKIT_CHARACTERISTIC(MODEL, "DS1"),
                  HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
                  HOMEKIT_CHARACTERISTIC(IDENTIFY, door_identify),
                  NULL
                  },
                  ),
          HOMEKIT_SERVICE(
              CONTACT_SENSOR,
              .primary=true,
              .characteristics=(homekit_characteristic_t*[]) {
                HOMEKIT_CHARACTERISTIC(NAME, "Kontakt"),
                &door_open_characteristic,
                NULL
            },
        ),
        NULL
}),
    HOMEKIT_ACCESSORY(
        .id=3,
        .category=homekit_accessory_category_programmable_switch,
        .services=(homekit_service_t*[]) {
            HOMEKIT_SERVICE(
                ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Botão Inteligente"),
                    HOMEKIT_CHARACTERISTIC(MANUFACTURER, "renandw"),
                    HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "736242"),
                    HOMEKIT_CHARACTERISTIC(MODEL, "Botão"),
                    HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1.0"),
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, button_identify),
                    NULL
                },
            ),
            HOMEKIT_SERVICE(
                STATELESS_PROGRAMMABLE_SWITCH,
                .primary=true,
                .characteristics=(homekit_characteristic_t*[]) {
                    HOMEKIT_CHARACTERISTIC(NAME, "Botão"),
                    &button_event,
                    NULL
                },
            ),
            NULL
        },
    ),    
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void on_wifi_ready() {
    homekit_server_init(&config);

}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "Lock-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Lock-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
}

void user_init(void) {
    uart_set_baud(0, 115200);

    create_accessory_name();

    wifi_config_init("lockbuttoncontact", NULL, on_wifi_ready);

    printf("Using Sensor at GPIO%d.\n", REED_PIN);
        if (contact_sensor_create(REED_PIN, contact_sensor_callback)) {
            printf("Failed to initialize door\n");
        }

    gpio_init();
    lock_init();

    button_config_t config = BUTTON_CONFIG(
        button_active_low,
        .long_press_time = 5000,
        .max_repeat_presses = 3,
    );

    int r = button_create(BUTTON_PIN, config, button_callback, NULL);
    if (r) {
        printf("Failed to initialize button\n");
    }
    homekit_characteristic_notify(&door_open_characteristic, door_state_getter());
}
