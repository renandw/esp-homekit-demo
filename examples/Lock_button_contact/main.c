/*
Acessório para controlar uma fechadura magnética.
Features: botão programável; sensor de contato
Maiores possibilidades para automação do acessório.
 */

// Bibliotecas a serem inclúidas:
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
// NOVO
#include "led_status.h"
#include "config.h"

//NOVO
// A porta GPIO conectada ao relê.
const int relay_gpio = RELAY_GPIO;
// A porta GPIO conectada ao LED - informando status do acessório.
const int led_gpio = LED_GPIO;
// A porta GPIO conectada ao botão para configuração de automação e reset.
const int button_gpio = BUTTON_GPIO;
// A porta GPIO conectada ao sensor de contato.

const int reed_gpio = REED_GPIO
#define REED_PIN 5
#ifndef REED_PIN
#error REED_PIN is not specified
#endif



// Timeout in seconds to open lock for
const int unlock_period = 5;  // 5 seconds
// Which signal to send to relay to open the lock (0 or 1)
const int relay_open_signal = 1;

//NOVO
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





void lock_lock();
void lock_unlock();

void button_identify(homekit_value_t _value) {
    printf("Button identify\n");
    led_status_signal(status, &mode_identify);
}

void door_identify(homekit_value_t _value) {
    printf("Door identifying\n");
    led_status_signal(status, &mode_identify);
}

void relay_write(int value) {
    gpio_write(relay_gpio, value ? 1 : 0);
}

//NOVO
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
    printf("Resetting configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}

void gpio_init() {
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
        case button_event_tripple_press:
            printf("tripple press\n");
            homekit_characteristic_notify(&button_event, HOMEKIT_UINT8(2));
            break;
        case button_event_long_press:
            printf("long press -- reseting configuration\n");
            reset_configuration();
            break;
        default:
            printf("unknown button event: %d\n", event);
    }
}

void lock_identify(homekit_value_t _value) {
    printf("Lock identify\n");
    led_status_signal(status, &mode_identify);
}


typedef enum {
    lock_state_unsecured = 0,
    lock_state_secured = 1,
    lock_state_jammed = 2,
    lock_state_unknown = 3,
} lock_state_t;



homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Lock");
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, NULL);


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
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "renandw"),
            &serial,
            HOMEKIT_CHARACTERISTIC(MODEL, "Magnética"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "3.0"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, lock_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MECHANISM, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Fechadura"),
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
        HOMEKIT_SERVICE(
            STATELESS_PROGRAMMABLE_SWITCH,
            .primary=false,
            .characteristics=(homekit_characteristic_t*[]) {
                HOMEKIT_CHARACTERISTIC(NAME, "Botão"),
                &button_event,
                NULL
            },
        ),
        HOMEKIT_SERVICE(
            CONTACT_SENSOR,
            .primary=false,
            .characteristics=(homekit_characteristic_t*[]) {
              HOMEKIT_CHARACTERISTIC(NAME, "Sensor de Contato"),
              &door_open_characteristic,
              NULL
          },
      ),
      NULL
   }),
   NULL
};

//NOVO daqui
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
    //observar esta mudança
    .password = ACCESSORY_SETUP_CODE,
    .on_event = on_homekit_event,
    //ci=6
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

    int name_len = snprintf(NULL, 0, "Fechadura-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Fechadura-%02X%02X%02X",
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

//Até aqui


void user_init(void) {
    uart_set_baud(0, 115200);

    //NOVO
    status = led_status_init(led_gpio, LED_ACTIVE_LEVEL);

    create_accessory_name();

    //NOVO
    wifi_config_init2(WIFI_AP_NAME, WIFI_AP_PASSWORD, on_wifi_config_event);

    printf("Using Sensor at GPIO%d.\n", REED_PIN);
        if (contact_sensor_create(REED_PIN, contact_sensor_callback)) {
            printf("Failed to initialize door\n");
        }

    gpio_init();
    lock_init();

    button_config_t config = BUTTON_CONFIG(
        button_active_low,
        .long_press_time = 10000,
        .max_repeat_presses = 3,
    );

    int r = button_create(BUTTON_GPIO, config, button_callback, NULL);
    if (r) {
        printf("Failed to initialize button\n");
    }
    homekit_characteristic_notify(&door_open_characteristic, door_state_getter());

    //NOVO
    led_status_set(status, wifi_is_configured() ? &mode_connecting_to_wifi : &mode_no_wifi_config);
}
