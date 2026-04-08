#include "esp_log.h"
#include "driver/gpio.h"
#include "relay_driver.h"
#include "esp_zb_relay.h"

static const char *TAG = "RELAY_DRIVER";

void relay_driver_init(bool state) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    relay_driver_set_state(state);
}

void relay_driver_set_state(bool state) {
    ESP_LOGI(TAG, "Relay set to %s", state ? "ON" : "OFF");
    gpio_set_level(RELAY_GPIO, state);
}
