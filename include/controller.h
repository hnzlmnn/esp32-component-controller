#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/cdefs.h>

__BEGIN_DECLS

#define CONTROLLER_ADDRESS 0x52

#define CONTROLLER_REG_INIT 0xF0
#define CONTROLLER_CMD_INIT 0x55
#define CONTROLLER_REG_INIT2 0xFB
#define CONTROLLER_CMD_INIT2 0x00
#define CONTROLLER_REG_STATE 0x00

#define CONTROLLER_UP 0x0001
#define CONTROLLER_DOWN 0x4000
#define CONTROLLER_LEFT 0x0002
#define CONTROLLER_RIGHT 0x8000

#define CONTROLLER_A 0x0010
#define CONTROLLER_B 0x0040

#define CONTROLLER_START 0x0400
#define CONTROLLER_SELECT 0x1000

typedef void (*leds_next_fn_t)(void);

typedef struct Controller {
    // Pins
    int     i2c_bus;
    uint8_t i2c_addr;
    bool    need_init;
    int     poll_delay; // Number of ms to wait between to polls
    // Internal state
    bool           initialized;
    uint16_t       previous_state;
    QueueHandle_t  queue;
    leds_next_fn_t led_cb;
    // Mutex
    SemaphoreHandle_t i2c_semaphore;
    SemaphoreHandle_t led_mutex;
    TaskHandle_t      poll_task_handle;
} Controller;

/**
 * Initializes the device
 * @param device The device reference
 * @return ESP_OK if initialization completed, an error otherwise
 */
extern esp_err_t controller_init(Controller* device);

/**
 * Tests if the controller is currently connected
 * @param device The device reference
 * @return true, if the controller was found, false otherwise
 */
extern bool controller_connected(Controller *device);

/**
 * Get the current state of pressed buttons (this gets updated periodically)
 * @param device The device reference
 * @return A 8 bit number with each bit representing a pressed (1) or unpressed (0) button
 */
extern uint8_t controller_get_state(Controller* device);

/**
 * Returns the current state
 * @param device The device reference
 * @return True, if polling is currently active
 */
extern bool controller_enabled(Controller* device);

/**
 * Enables the controller polling
 * @param device The device reference
 * @return True if the polling has been successfully enabled
 */
extern bool controller_enable(Controller* device);

/**
 * Disables the controller polling
 * @param device The device reference
 * @return True if the polling task was successfully deleted
 */
extern bool controller_disable(Controller* device);

__END_DECLS
