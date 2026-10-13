// Headers
#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "led_strip.h"
#include "CAN.h"

// Tag for logging
static const char *TAG = "Final_Project";

// Set pin numbers
#define BUTTON_GPIO         GPIO_NUM_8
#define ACTIVE_LED_GPIO     GPIO_NUM_3
#define HAZARD_LED_GPIO     GPIO_NUM_42
#define PEDAL_LED_GPIO      GPIO_NUM_35
#define POT_ADC_CHANNEL     ADC_CHANNEL_3
#define POT_LED_GPIO        GPIO_NUM_47

// I2C pins for ltc4151
#define I2C_MASTER_SCL_IO       GPIO_NUM_14
#define I2C_MASTER_SDA_IO       GPIO_NUM_13
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TIMEOUT_MS   1000
#define LTC4151_SENSOR_ADDR     0x69

// Morse implentation definitions
#define MORSE_DOT_MS 150
#define MORSE_DASH_MS 450


// State definitions
typedef enum {
    SYS_STATE_IDLE,
    SYS_STATE_ACTIVE,
} system_state_t;

// System context structure
typedef struct {
    system_state_t state;
    bool hazards_enabled;
    uint8_t pedal_position; 
    float current_voltage;    
} system_context_t;

// Startup state
static system_context_t g_context = {
    .state = SYS_STATE_IDLE,
    .hazards_enabled = false,
    .pedal_position = 0,
    .current_voltage = 12.0f
};

// Mutex for context struct so tasks don't step on each other
static SemaphoreHandle_t context_mutex = NULL;

// Circular buffer parameters for smoothing data
#define BUFFER_SIZE 8

static int pedal_buffer[BUFFER_SIZE];
static int pedal_head = 0;
static int pedal_count = 0;

static int volt_buffer[BUFFER_SIZE];
static int volt_head = 0;
static int volt_count = 0;

// Morse Code Table
static const char *MORSE_ALPHABET[] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---",
    "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-",
    "..-", "...-", ".--", "-..-", "-.--", "--.."
};


// queue for button press events, timer for the idle blink
static QueueHandle_t button_queue = NULL;
static TimerHandle_t idle_blink_timer = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL; // handle for the potentiometer ADC
TaskHandle_t hazard_task_handle = NULL; // need handles so the FSM can suspend/resume these
TaskHandle_t pot_task_handle = NULL;
TaskHandle_t can_rx_task_handle = NULL;
TaskHandle_t can_tx_task_handle = NULL;
TaskHandle_t i2c_task_handle = NULL;

// I2C handles
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t ltc_dev_handle = NULL;

// active led strip handle (RGB LED)
static led_strip_handle_t active_led_strip = NULL;

// sets up the active rgb led using the led_strip driver
void init_active_led(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = ACTIVE_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &active_led_strip));
    led_strip_clear(active_led_strip);
}

// helper to turn active led on or off
static void set_active_led(bool on) {
    if (!active_led_strip) return;
    if (on) {
        // white at moderate brightness
        led_strip_set_pixel(active_led_strip, 0, 16, 16, 16);
        led_strip_refresh(active_led_strip);
    } else {
        led_strip_clear(active_led_strip);
    }
}

// sets up the hazard LED as a standard gpio output, starts it off
void init_gpio_outputs(void) {
    gpio_reset_pin(HAZARD_LED_GPIO);
    gpio_set_direction(HAZARD_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(HAZARD_LED_GPIO, 0);
}

// configures LEDC timer and two PWM channels — one for pedal LED, one for pot LED
void init_ledc_pwm(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t pedal_channel = {
        .gpio_num = PEDAL_LED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pedal_channel));

    ledc_channel_config_t pot_channel = {
        .gpio_num = POT_LED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pot_channel));
}

// just wraps ledc_set_duty for the pedal LED on channel 0
void set_pedal_led_brightness(uint8_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// same thing but for the pot LED on channel 1
void set_pot_led_brightness(uint8_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// initializes ADC unit 1 and configures the pot channel with 12dB attenuation (0-3.3V range)
void init_pot_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_ADC_CHANNEL, &channel_config));
}

// ISR fires on falling edge (button press) — checks pin level to reject release bounce and noise
static void IRAM_ATTR button_isr_handler(void *arg) {
    static int64_t last_press_time = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_press_time > 250000) {
        // confirm pin is actively held low (active-low button), rejecting release spikes
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            last_press_time = now;
            uint32_t event = 1;
            xQueueSendFromISR(button_queue, &event, NULL);
        }
    }
}
void init_button_input(void) {
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL));
}

// read register from ltc4151
static esp_err_t ltc4151_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

// set up i2c master and register sensor
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LTC4151_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

// add value to circular buffer and calculate average
static int smooth_pedal_data(int new_val) {
    pedal_buffer[pedal_head] = new_val;
    pedal_head = (pedal_head + 1) % BUFFER_SIZE;
    if (pedal_count < BUFFER_SIZE) {
        pedal_count++;
    }

    int sum = 0;
    for (int i = 0; i < pedal_count; i++) {
        sum += pedal_buffer[i];
    }
    int average = sum / pedal_count;
    return average;
}

// same buffer calculation for power sensor
static int smooth_volt_data(int new_val) {
    volt_buffer[volt_head] = new_val;
    volt_head = (volt_head + 1) % BUFFER_SIZE;
    if (volt_count < BUFFER_SIZE) {
        volt_count++;
    }

    int sum = 0;
    for (int i = 0; i < volt_count; i++) {
        sum += volt_buffer[i];
    }
    int average = sum / volt_count;
    return average;
}

void hazard_led_task(void *arg) {
    while (1) {
        bool hazard_active = false;
        if (xSemaphoreTake(context_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            hazard_active = g_context.hazards_enabled;
            xSemaphoreGive(context_mutex);
        }

        if (hazard_active) {
            gpio_set_level(HAZARD_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(HAZARD_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            gpio_set_level(HAZARD_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void bonus_pot_task(void *arg) {
    int raw_val = 0;
    static uint8_t last_logged_duty = 0xFF;

    while (1) {
        esp_err_t ret = adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &raw_val);
        if (ret == ESP_OK) {
            uint8_t duty = (uint8_t)(raw_val >> 4);
            set_pot_led_brightness(duty);

            // only log if change is noticeable to prevent flooding console uart
            if (abs((int)duty - (int)last_logged_duty) > 4) {
                last_logged_duty = duty;
                ESP_LOGI(TAG, "Potentiometer ADC: %d, Duty Cycle: %d", raw_val, duty);
            }
        } else {
            ESP_LOGE(TAG, "Failed to read potentiometer ADC. Error: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// listen on can bus and update state
void can_rx_task(void *arg) {
    CAN_message_t rx_msg;
    ESP_LOGI(TAG, "can rx loop started");

    while (1) {
        esp_err_t ret = CAN_receive(&rx_msg, 100);

        if (ret == ESP_OK) {
            if (rx_msg.id == 0x01 && rx_msg.dlc >= 2) {
                uint8_t hb = rx_msg.data[0];
                uint8_t raw_pwr = rx_msg.data[1];
                float pedal_board_volts = raw_pwr / 10.0f;
                ESP_LOGI(TAG, "pedals status received hb: %d, power: %.1f V", hb, pedal_board_volts);
            }
            else if (rx_msg.id == 0x11 && rx_msg.dlc >= 1) {
                uint8_t raw_pedal = rx_msg.data[0];
                int filtered_pedal = smooth_pedal_data(raw_pedal);

                if (xSemaphoreTake(context_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_context.pedal_position = (uint8_t)filtered_pedal;
                    xSemaphoreGive(context_mutex);
                }

                set_pedal_led_brightness((uint8_t)filtered_pedal);
                ESP_LOGI(TAG, "pedal pos raw: %d, filtered: %d", raw_pedal, filtered_pedal);
            }
            else if (rx_msg.id == 0x12 && rx_msg.dlc >= 1) {
                uint8_t toggle_val = rx_msg.data[0];
                bool hazard_state = (toggle_val == 1);

                if (xSemaphoreTake(context_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_context.hazards_enabled = hazard_state;
                    xSemaphoreGive(context_mutex);
                }

                ESP_LOGI(TAG, "hazards toggle packet received: %d", toggle_val);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// periodically send heartbeats and power reading to pedals board
void can_tx_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(100);

    ESP_LOGI(TAG, "can tx task started");

    while (1) {
        vTaskDelayUntil(&last_wake, period_ticks);

        uint8_t heartbeat_val = 0x00;
        CAN_message_t hb_packet = build_packet_no_ext(0x02, &heartbeat_val, 1);
        esp_err_t ret = CAN_send(&hb_packet, 50);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "heartbeat send error: %s", esp_err_to_name(ret));
        }

        float v_reading = 12.0f;
        if (xSemaphoreTake(context_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            v_reading = g_context.current_voltage;
            xSemaphoreGive(context_mutex);
        }

        int temp_pwr = (int)(v_reading * 10.0f);
        if (temp_pwr < 0) {
            temp_pwr = 0;
        }
        if (temp_pwr > 120) {
            temp_pwr = 120;
        }
        uint8_t power_payload = (uint8_t)temp_pwr;

        CAN_message_t pwr_packet = build_packet_no_ext(0x21, &power_payload, 1);
        ret = CAN_send(&pwr_packet, 50);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "power send error: %s", esp_err_to_name(ret));
        }
    }
}

// read ltc4151 sensor over i2c and compute voltage
void power_monitor_task(void *arg) {
    uint8_t rx_bytes[2];

    while (1) {
        esp_err_t ret = ltc4151_register_read(ltc_dev_handle, 0x02, rx_bytes, 2);

        if (ret == ESP_OK) {
            uint16_t b0 = rx_bytes[0];
            uint16_t b1 = rx_bytes[1];
            uint16_t raw_adc = (b0 << 4) | (b1 >> 4);
            float calculated_voltage = raw_adc * 0.025f;

            int scaled_volt = (int)(calculated_voltage * 100);
            int filtered_scaled = smooth_volt_data(scaled_volt);
            float filtered_voltage = filtered_scaled / 100.0f;

            if (xSemaphoreTake(context_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_context.current_voltage = filtered_voltage;
                xSemaphoreGive(context_mutex);
            }

            ESP_LOGI(TAG, "sensor voltage: %.2f V, smoothed: %.2f V", calculated_voltage, filtered_voltage);
        } else {
            ESP_LOGW(TAG, "cant read ltc4151 sensor, defaulting reading: %s", esp_err_to_name(ret));
            if (xSemaphoreTake(context_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_context.current_voltage = 12.0f;
                xSemaphoreGive(context_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void idle_blink_callback(TimerHandle_t xTimer) {
    static bool s_led_level = false;
    s_led_level = !s_led_level;
    set_active_led(s_led_level);
}

static void morse_pulse(int duration_ms) {
    set_active_led(true);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    set_active_led(false);
    vTaskDelay(pdMS_TO_TICKS(MORSE_DOT_MS)); // inter-element gap
}

// walks through each character in the string and flashes the correct dot/dash pattern
static void play_morse_string(const char *str) {
    for (int i = 0; str[i] != '\0'; i++) {
        char c = toupper((unsigned char)str[i]);
        if (c >= 'A' && c <= 'Z') {
            const char *morse = MORSE_ALPHABET[c - 'A'];

            for (int j = 0; morse[j] != '\0'; j++) {
                if (morse[j] == '.') {
                    morse_pulse(MORSE_DOT_MS);
                } else if (morse[j] == '-') {
                    morse_pulse(MORSE_DASH_MS);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(MORSE_DASH_MS));
        } else if (c == ' ') {
            vTaskDelay(pdMS_TO_TICKS(2 * MORSE_DASH_MS)); // Space between words
        }
    }
}

// runs the morse sequence then notifies the supervisor it's done so it can continue
void morse_code_task(void *arg) {
    TaskHandle_t supervisor_handle = (TaskHandle_t)arg;

    const char *message = "Yong and June";

    ESP_LOGI(TAG, "Morse: Playing name: %s", message);
    play_morse_string(message);
    ESP_LOGI(TAG, "Done");

    xTaskNotifyGive(supervisor_handle);

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}


// blocks on the button queue; transitions between IDLE and ACTIVE and handles all the suspend/resume logic
void fsm_supervisor_task(void *arg) {
    uint32_t event;
    while (1) {
        if (xQueueReceive(button_queue, &event, portMAX_DELAY) == pdTRUE) {
            // temporarily disable button interrupts during state change to prevent re-triggering
            gpio_intr_disable(BUTTON_GPIO);

            if (g_context.state == SYS_STATE_IDLE) {
                ESP_LOGI(TAG, "Transitioning to ACTIVE");
                // stop idle blink before running morse sequence so they don't fight
                xTimerStop(idle_blink_timer, 0);
                set_active_led(false);

                // play morse sequence before resuming tasks
                TaskHandle_t morse_task_handle = NULL;
                xTaskCreate(morse_code_task, "morse_code", 2048, (void *)xTaskGetCurrentTaskHandle(), 2, &morse_task_handle);
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for morse task to finish
                vTaskDelete(morse_task_handle);
                morse_task_handle = NULL;
                
                set_active_led(true); // solid on while active
                if (xSemaphoreTake(context_mutex, portMAX_DELAY) == pdTRUE) {
                    g_context.state = SYS_STATE_ACTIVE;
                    xSemaphoreGive(context_mutex);
                }
                // resume all the worker tasks
                if (hazard_task_handle) vTaskResume(hazard_task_handle);
                if (pot_task_handle) vTaskResume(pot_task_handle);
                if (can_rx_task_handle) vTaskResume(can_rx_task_handle);
                if (can_tx_task_handle) vTaskResume(can_tx_task_handle);
                if (i2c_task_handle) vTaskResume(i2c_task_handle);
            } else {
                ESP_LOGI(TAG, "Transitioning to IDLE");
                if (xSemaphoreTake(context_mutex, portMAX_DELAY) == pdTRUE) {
                    g_context.state = SYS_STATE_IDLE;
                    // suspend worker tasks while holding mutex so none are suspended inside a critical section
                    if (hazard_task_handle) vTaskSuspend(hazard_task_handle);
                    if (pot_task_handle) vTaskSuspend(pot_task_handle);
                    if (can_rx_task_handle) vTaskSuspend(can_rx_task_handle);
                    if (can_tx_task_handle) vTaskSuspend(can_tx_task_handle);
                    if (i2c_task_handle) vTaskSuspend(i2c_task_handle);
                    xSemaphoreGive(context_mutex);
                }

                // make sure all outputs are off/safe before going idle
                set_pedal_led_brightness(0);
                set_pot_led_brightness(0);
                gpio_set_level(HAZARD_LED_GPIO, 0);
                set_active_led(false);
                xTimerStart(idle_blink_timer, 0); // start blinking active LED to show we're idle
            }

            // wait for physical button release in case user is still holding it down
            while (gpio_get_level(BUTTON_GPIO) == 0) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // let contacts settle after release

            // flush any queue items that accumulated and re-enable interrupt
            xQueueReset(button_queue);
            gpio_intr_enable(BUTTON_GPIO);
        }
    }
}

// app_main is the entry point — sets up everything then hands off to the scheduler
void app_main(void)
{
    ESP_LOGI(TAG, "Booting Brakelights ESP32 into IDLE state...");

    // create mutex for context
    context_mutex = xSemaphoreCreateMutex();

    // queue depth of 1 avoids accumulating stale presses
    button_queue = xQueueCreate(1, sizeof(uint32_t));
    idle_blink_timer = xTimerCreate(
        "idle_blink",
        pdMS_TO_TICKS(500),
        pdTRUE,                 // auto-reload so it keeps repeating
        NULL,
        idle_blink_callback
    );

    // initialize all the hardware peripherals
    init_active_led();
    init_gpio_outputs();
    init_ledc_pwm();
    init_pot_adc();
    init_button_input();

    // init i2c bus and device
    i2c_master_init(&i2c_bus_handle, &ltc_dev_handle);

    // init can driver
    CAN_init();

    // spin up the supervisor and worker tasks
    xTaskCreate(fsm_supervisor_task, "fsm_supervisor", 3072, NULL, 3, NULL);
    xTaskCreate(hazard_led_task, "hazard_task", 2048, NULL, 2, &hazard_task_handle);
    xTaskCreate(bonus_pot_task, "bonus_pot", 2048, NULL, 1, &pot_task_handle);
    xTaskCreate(can_rx_task, "can_rx", 3072, NULL, 3, &can_rx_task_handle);
    xTaskCreate(can_tx_task, "can_tx", 3072, NULL, 2, &can_tx_task_handle);
    xTaskCreate(power_monitor_task, "power_monitor", 3072, NULL, 1, &i2c_task_handle);

    // start in IDLE — suspend the workers and kick off the idle blink
    vTaskSuspend(hazard_task_handle);
    vTaskSuspend(pot_task_handle);
    vTaskSuspend(can_rx_task_handle);
    vTaskSuspend(can_tx_task_handle);
    vTaskSuspend(i2c_task_handle);
    xTimerStart(idle_blink_timer, 0);

    // background uptime tracker
    while (1) {
        int64_t uptime_seconds = esp_timer_get_time() / 1000000;
        ESP_LOGI(TAG, "uptime: %lld seconds", uptime_seconds);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}