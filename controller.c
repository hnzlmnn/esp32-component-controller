#include "controller.h"

#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdbool.h>
#include <sys/cdefs.h>

#include "keyboard.h"
#include "managed_i2c.h"

#define BUTTON_STATE_CHANGED(DEV, STATE, SEND_FN, BUTTON, KEY)                   \
    if (((DEV)->previous_state & BUTTON) != (STATE & BUTTON)) {                  \
        SEND_FN(DEV, KEY, (STATE & BUTTON) == BUTTON);                           \
        if ((STATE & BUTTON) == BUTTON) pressed = true;                          \
    }

static const char* TAG = "controller";

/* I2C access */
static inline esp_err_t read_reg(Controller* device, uint8_t reg, uint8_t* data, size_t data_len) {
    if (device->i2c_semaphore != NULL) xSemaphoreTake(device->i2c_semaphore, portMAX_DELAY);
    esp_err_t res = i2c_write_byte(device->i2c_bus, device->i2c_addr, reg);
    if (res != ESP_OK) {
        ESP_LOGD(TAG, "write byte error %d", res);
        goto err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    res = i2c_read_bytes(device->i2c_bus, device->i2c_addr, data, data_len);
    if (res != ESP_OK) {
        ESP_LOGD(TAG, "read reg error %d", res);
        goto err;
    }
err:
    if (device->i2c_semaphore != NULL) xSemaphoreGive(device->i2c_semaphore);
    return res;
}

static inline esp_err_t write_reg(Controller* device, uint8_t reg, uint8_t data) {
    if (device->i2c_semaphore != NULL) xSemaphoreTake(device->i2c_semaphore, portMAX_DELAY);
    esp_err_t res = i2c_write_reg(device->i2c_bus, device->i2c_addr, reg, data);
    if (device->i2c_semaphore != NULL) xSemaphoreGive(device->i2c_semaphore);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "write reg error %d", res);
    }
    return res;
}

static inline void send_button_event(Controller *device, uint8_t key, bool state) {
    keyboard_input_message_t message;
    message.input = key;
    message.state = state;
    ESP_LOGI(TAG, "Key event %d, %d", key, state);
    xQueueSend(device->queue, &message, portMAX_DELAY);
}

static inline esp_err_t _init(Controller* device) {
    ESP_LOGI(TAG, "initialize dev");
    esp_err_t res = write_reg(device, CONTROLLER_REG_INIT, CONTROLLER_CMD_INIT);
    if (res != ESP_OK) return res;
    vTaskDelay(pdMS_TO_TICKS(10));

    res = write_reg(device, CONTROLLER_REG_INIT2, CONTROLLER_CMD_INIT2);
    if (res != ESP_OK) return res;
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

static inline esp_err_t _read_id(Controller* device) {
    uint8_t   data[6];
    read_reg(device, 0xfa, data, 6);
    ESP_LOGI(TAG, "%d %d %d %d %d %d", data[0], data[1], data[2], data[3], data[4], data[5]);
    // TODO: this should be 1 0 164 32 1 1
    return ESP_OK;
}

bool update_buttons(Controller *device) {
    uint8_t   data[6];

    esp_err_t res = read_reg(device, CONTROLLER_REG_STATE, data, 6);
    if (res != ESP_OK) {
        return false;
    }

    if (device->need_init && data[0] == 0xff && data[1] == 0xff && data[2] == 0xff && data[3] == 0xff) {
        // This seems like we need an init
        _init(device);
    }

    ESP_LOGD(TAG, "%d %d %d %d %d %d", data[0], data[1], data[2], data[3], data[4], data[5]);

    uint16_t buttons = ((255 - data[4]) << 8) + (255 - data[5]);

    bool pressed = false;

    BUTTON_STATE_CHANGED(device, buttons, send_button_event, CONTROLLER_A,      BUTTON_ACCEPT);
    BUTTON_STATE_CHANGED(device, buttons, send_button_event, CONTROLLER_B,      BUTTON_BACK);
    BUTTON_STATE_CHANGED(device, buttons, send_button_event, CONTROLLER_START,  BUTTON_START);
    BUTTON_STATE_CHANGED(device, buttons, send_button_event, CONTROLLER_SELECT, BUTTON_SELECT);
    BUTTON_STATE_CHANGED(device, buttons, send_button_event, CONTROLLER_UP,     JOYSTICK_UP);
    BUTTON_STATE_CHANGED(device, buttons, send_button_event, CONTROLLER_DOWN,   JOYSTICK_DOWN);
    BUTTON_STATE_CHANGED(device, buttons, send_button_event, CONTROLLER_LEFT,   JOYSTICK_LEFT);
    BUTTON_STATE_CHANGED(device, buttons, send_button_event, CONTROLLER_RIGHT,  JOYSTICK_RIGHT);

    if (pressed) {
        if (device->led_mutex != NULL) xSemaphoreTake(device->led_mutex, portMAX_DELAY);
        if (device->led_cb != NULL) device->led_cb();
        if (device->led_mutex != NULL) xSemaphoreGive(device->led_mutex);
    }

    device->previous_state = buttons;
    return true;
}

_Noreturn static inline void poll_task(void* pvParameters) {
    Controller* device = (Controller*) pvParameters;
    while (1) {
        while (!controller_connected(device)) {
            // Don't pollute the I2C bus too much by waiting a long time
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (!update_buttons(device)) {
            // We failed to read, skip directly to the presence check
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(device->poll_delay));
    }
}

/* Public functions */

esp_err_t controller_init(Controller* device) {
    ESP_LOGD(TAG, "init called");

    if (device->poll_delay == 0) {
        device->poll_delay = 200;
    }

    device->previous_state = 0;

    ESP_LOGD(TAG, "init done");
    return ESP_OK;
}

bool controller_connected(Controller *device) {
    if (device->i2c_semaphore != NULL) xSemaphoreTake(device->i2c_semaphore, portMAX_DELAY);
    esp_err_t res = i2c_write_buffer(device->i2c_bus, device->i2c_addr, NULL, 0);
    if (device->i2c_semaphore != NULL) xSemaphoreGive(device->i2c_semaphore);
    return res == ESP_OK;
}

uint8_t controller_get_state(Controller* device) {
    return device->previous_state;
}

bool controller_enabled(Controller* device) {
    return device->poll_task_handle != NULL;
}

bool controller_enable(Controller* device) {
    if (device->poll_task_handle != NULL) return false;

    if (!device->initialized && device->need_init) {
        if (_init(device) != ESP_OK) return false;

        // Remember that we already initialized the chip
        // TODO: Uninitalize once SAO disconnects
        device->initialized = true;
    }
    _read_id(device);
    xTaskCreate(poll_task, "controller_poll_task", 2048, device, 12, device->poll_task_handle);
    return true;
}

bool controller_disable(Controller* device) {
    if (device->poll_task_handle == NULL) return false;
    vTaskDelete(device->poll_task_handle);
    device->poll_task_handle = NULL;
    return true;
}

void controller_enable_leds(Controller* device, leds_next_fn_t cb) {
    if (device->led_mutex != NULL) xSemaphoreTake(device->led_mutex, portMAX_DELAY);
    device->led_cb = cb;
    if (device->led_mutex != NULL) xSemaphoreGive(device->led_mutex);
}

void controller_disable_leds(Controller* device) {
    if (device->led_mutex != NULL) xSemaphoreTake(device->led_mutex, portMAX_DELAY);
    device->led_cb = NULL;
    if (device->led_mutex != NULL) xSemaphoreGive(device->led_mutex);
}