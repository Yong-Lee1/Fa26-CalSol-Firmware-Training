// Headers
#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_log.h"

// Set pin numbers
#define BUTTON_GPIO         GPIO_NUM_8
#define ACTIVE_LED_GPIO     GPIO_NUM_3
#define HAZARD_LED_GPIO     GPIO_NUM_42
#define PEDAL_LED_GPIO      GPIO_NUM_35
#define POT_ADC_CHANNEL     ADC_CHANNEL_3
#define POT_LED_GPIO        GPIO_NUM_47

// Morse implentation definitions
#define MORSE_DOT_MS 150
#define MORSE_DASH_MS 450


// State definitions
typedef enum {
    SYS_STATE_IDLE = 0,
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

// Morse Code Table
static const char *MORSE_ALPHABET[] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---",
    "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-",
    "..-", "...-", ".--", "-..-", "-.--", "--.."
};


// --- TASK & QUEUE HANDLES ---
static QueueHandle_t button_queue = NULL;
static TimerHandle_t idle_blink_timer = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;
// Worker handles managed by Partner 2's FSM
TaskHandle_t hazard_task_handle = NULL;
TaskHandle_t pot_task_handle = NULL;
// Partner 1's tasks (externally declared or shared)
TaskHandle_t can_rx_task_handle;
TaskHandle_t can_tx_task_handle;
TaskHandle_t i2c_task_handle;
// ==========================================
// TODO 1: GPIO & LEDC PWM INITIALIZATION
// ==========================================
void init_gpio_outputs(void) {
    gpio_reset_pin(ACTIVE_LED_GPIO);
    gpio_reset_pin(HAZARD_LED_GPIO);
    gpio_set_direction(ACTIVE_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(HAZARD_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ACTIVE_LED_GPIO, 0);
    gpio_set_level(HAZARD_LED_GPIO, 0);
}
void init_ledc_pwm(void) {
    // TODO: Configure ledc_timer_config_t (5kHz, 8-bit, LOW_SPEED_MODE)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // TODO: Configure Channel 0 on PEDAL_LED_GPIO
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

    // TODO: Configure Channel 1 on POT_LED_GPIO
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
void set_pedal_led_brightness(uint8_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
void set_pot_led_brightness(uint8_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}
// ==========================================
// TODO 2: ADC ONE-SHOT INITIALIZATION
// ==========================================
void init_pot_adc(void) {
    // TODO: Call adc_oneshot_new_unit() on ADC_UNIT_1
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // TODO: Call adc_oneshot_config_channel() for POT_ADC_CHANNEL with ADC_ATTEN_DB_12
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_ADC_CHANNEL, &channel_config));
}
// ==========================================
// TODO 3: BUTTON ISR & DEBOUNCING
// ==========================================
static void IRAM_ATTR button_isr_handler(void *arg) {
    static int64_t last_press_time = 0;
    int64_t now = esp_timer_get_time(); // Microseconds
    // Debounce: 200 ms (200000 us)
    if (now - last_press_time > 200000) {
        last_press_time = now;
        uint32_t event = 1;
        xQueueSendFromISR(button_queue, &event, NULL);
    }
}
void init_button_input(void) {
    // TODO: Configure BUTTON_GPIO with internal pull-up and GPIO_INTR_NEGEDGE
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));
    // TODO: Call gpio_install_isr_service(0)
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // TODO: Call gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL)
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL));
}
// ==========================================
// TODO 4: TASKS & TIMERS
// ==========================================
void hazard_led_task(void *arg) {
    while (1) {
        if (g_context.hazards_enabled) {
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
    while (1) {
        // TODO: Read ADC with adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &raw_val)
        esp_err_t ret = adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &raw_val);
        if (ret == ESP_OK) {
            // TODO: Shift 12-bit to 8-bit: uint8_t duty = raw_val >> 4;
            uint8_t duty = (uint8_t)(raw_val >> 4);
            // TODO: Update PWM via set_pot_led_brightness(duty);
            set_pot_led_brightness(duty);
            // TODO: Print reading to Serial
            ESP_LOGI(TAG, "Potentiometer ADC Reading: %d, Duty Cycle: %d", raw_val, duty);
        } else {
            ESP_LOGE(TAG, "Failed to read potentiometer ADC. Error: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void idle_blink_callback(TimerHandle_t xTimer) {
    // TODO: Toggle ACTIVE_LED_GPIO (or cycle colors for bonus)
    static int s_led_level = false;
    s_led_level = !s_led_level;
    gpio_set_level(ACTIVE_LED_GPIO, s_led_level);
}

static void morse_pulse(int duration_ms) {
    gpio_set_level(ACTIVE_LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(ACTIVE_LED_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(MORSE_DOT_MS)); // Inter-element gap
}

// Morse code helper
static void play_morse_string(const char *str) {
    for (int i = 0; str[i] != '\0'; i++) {
        char c = toupper(str[i]);
        if (c >= 'A' && c <= 'Z') {
            const char *morse = MORSE_ALPHABET[c - 'A'];

            for (int j = 0; morse[j] != '\0'; j++) {
                if (morse[j] == '.') {
                    morse_pulse(MORSE_DOT_MS);
                } else if (pattern[j] == '-') {
                    morse_pulse(MORSE_DASH_MS);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(MORSE_DASH_MS));
        } else if (c == ' ') {
            vTaskDelay(pdMS_TO_TICKS(2 * MORSE_DASH_MS)); // Space between words
        }
    }
}

// Morse code function
void morse_code_task(void *arg) {
    TaskHandle_t supervisor_handle = (TaskHandle_t)arg;

    const char *message = "Yong and June"; // Example message

    ESP_LOGI(TAG, "Morse: Playing name: %s", message);
    play_morse_string(message);
    ESP_LOGI(TAG, "Done");

    xTaskNotifyGive(supervisor_handle); // Notify the supervisor task that Morse code is done

    while (1) {
        vTaskDelay(portMAX_DELAY); // Suspend this task indefinitely
    }
}


// ==========================================
// TODO 5: FSM SUPERVISOR TASK
// ==========================================
void fsm_supervisor_task(void *arg) {
    uint32_t event;
    while (1) {
        if (xQueueReceive(button_queue, &event, portMAX_DELAY) == pdTRUE) {
            if (g_context.state == SYS_STATE_IDLE) {
                ESP_LOGI(TAG, "Transitioning to ACTIVE");
                // TODO (Bonus): Run Morse sequence here before resuming tasks
                TaskHandle_t morse_task_handle = NULL;
                xTaskCreate(morse_code_task, "morse_code", 2048, (void *)xTaskGetCurrentTaskHandle(), 2, &morse_task_handle);
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for Morse code task to complete   
                vTaskDelete(morse_task_handle); // Clean up Morse code task after completion
                morse_task_handle = NULL; // Reset handle to NULL after deletion
                
                xTimerStop(idle_blink_timer, 0);
                gpio_set_level(ACTIVE_LED_GPIO, 1); // Solid Green
                g_context.state = SYS_STATE_ACTIVE;
                // Resume tasks
                if (hazard_task_handle) vTaskResume(hazard_task_handle);
                if (pot_task_handle) vTaskResume(pot_task_handle);
                if (can_rx_task_handle) vTaskResume(can_rx_task_handle);
                if (can_tx_task_handle) vTaskResume(can_tx_task_handle);
                if (i2c_task_handle) vTaskResume(i2c_task_handle);
            } else {
                ESP_LOGI(TAG, "Transitioning to IDLE");
                g_context.state = SYS_STATE_IDLE;
                // Suspend tasks
                if (hazard_task_handle) vTaskSuspend(hazard_task_handle);
                if (pot_task_handle) vTaskSuspend(pot_task_handle);
                if (can_rx_task_handle) vTaskSuspend(can_rx_task_handle);
                if (can_tx_task_handle) vTaskSuspend(can_tx_task_handle);
                if (i2c_task_handle) vTaskSuspend(i2c_task_handle);
                // Set safe outputs
                set_pedal_led_brightness(0);
                set_pot_led_brightness(0);
                gpio_set_level(HAZARD_LED_GPIO, 0);
                // Start idle blink
                xTimerStart(idle_blink_timer, 0);
            }
        }
    }
}

// ==========================================
// ENTRY POINT: app_main
// ==========================================
void app_main(void)
{
    ESP_LOGI(TAG, "Booting Brakelights ESP32 into IDLE state...");

    // 1. Create OS Primitives (Queue & Software Timer)
    button_queue = xQueueCreate(5, sizeof(uint32_t));
    idle_blink_timer = xTimerCreate(
        "idle_blink",
        pdMS_TO_TICKS(500),
        pdTRUE,                 // Auto-reload (repeating timer)
        NULL,
        idle_blink_callback
    );

    // 2. Initialize Hardware Peripherals
    init_gpio_outputs();
    init_ledc_pwm();      // (Uncomment when you write TODO 1)
    init_pot_adc();       // (Uncomment when you write TODO 2)
    init_button_input();  // (Uncomment when you write TODO 3)

    // 3. Create Supervisory & Worker Tasks
    xTaskCreate(fsm_supervisor_task, "fsm_supervisor", 3072, NULL, 3, NULL);
    xTaskCreate(hazard_led_task, "hazard_task", 2048, NULL, 2, &hazard_task_handle);
    xTaskCreate(bonus_pot_task, "bonus_pot", 2048, NULL, 1, &pot_task_handle);

    // 4. Start in IDLE: suspend worker tasks and start idle blink
    vTaskSuspend(hazard_task_handle);
    xTimerStart(idle_blink_timer, 0);
}
















































































/*// --- INCLUDE HEADERS ---

// FREE RTOS
#include "freertos/FreeRTOS.h" // FreeRTOS.h to use FreeRTOS features
#include "freertos/task.h" // task.h enables you to create and manage tasks in FreeRTOS

// DEBUGGING
#include "esp_log.h" // esp_log.h provides logging functions
#include <stdio.h> // stdio.h lets me print into the console

// DRIVERS
#include "CAN.h" // CAN.h is custom header file for CAN driver functions and structures
#include "driver/gpio.h" // GPIO driver for ESP32 allows you to control the GPIO pins on the ESP32
#include "driver/i2c_master.h" // I2C driver for ESP32, allows you to communicate with I2C devices


// *TAG used for logging purposes, helps identify the source of log messages in the console
static const char *TAG = "Final_Project";

// --- DEFINITIONS ---
#define I2C_MASTER_SCL_IO       GPIO_NUM_14 // SCL pin from ESP32 schematic
#define I2C_MASTER_SDA_IO       GPIO_NUM_13 // SDA pin from ESP32 schematic
#define I2C_MASTER_NUM          I2C_NUM_0 // I2C port number, ESP32 has two I2C ports, 0 and 1
#define I2C_MASTER_FREQ_HZ      100000 // 100 kHz I2C clock frequency, standard speed for I2C communication
#define I2C_MASTER_TIMEOUT_MS   1000 // Timeout for I2C operations in milliseconds (will throw error if nothing received in time frame)
#define LTC4151_SENSOR_ADDR     0x69 // Chip address (ADR1 & ADR0 tied high) for LTC4151 power monitor sensor

// ---I2C Helper Functions ---

// Reads bytes from I2C device at specific register address
static esp_err_t ltc4151_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) 
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS); // Transmit the register address and receive the data from the device, with a timeout of 1000 milliseconds
}

// Initializes the I2C bus and adds the LTC4151 device to it
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) 
{
    i2c_master_bus_config_t bus_config = { // configs i2c bus with the parameters defined in the header file
        .i2c_port = I2C_MASTER_NUM, // I2C port number, ESP32 has two I2C ports, 0 and 1
        .sda_io_num = I2C_MASTER_SDA_IO, // SDA pin from ESP32 schematic
        .scl_io_num = I2C_MASTER_SCL_IO, // SCL pin from ESP32 schematic
        .clk_source = I2C_CLK_SRC_DEFAULT, // default clock source for I2C bus
        .glitch_ignore_cnt = 7, // filters out glitches on the line that are shorter than 7 clock cycles
        .flags.enable_internal_pullup = false, // disabled due to external pullups on the board
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle)); // initializes i2c bus

    i2c_device_config_t dev_config = { // configs i2c device with the parameters defined in the header file
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7-bit address length for the LTC4151 sensor
        .device_address = LTC4151_SENSOR_ADDR, // I2C address of the LTC4151 sensor
        .scl_speed_hz = I2C_MASTER_FREQ_HZ, // I2C clock speed for communication with the sensor
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle)); // adds the LTC4151 device to the I2C bus and initializes it
}

// --- TASKS ---
void strobe_led_task(void *arg)
{
    // Configure GPIO pin for output
    gpio_num_t led_pin = (gpio_num_t)arg; // Cast the argument to gpio_num_t type
    gpio_reset_pin(led_pin); // Reset the GPIO pin to its default state
    gpio_set_direction(led_pin, GPIO_MODE_OUTPUT); // Set the GPIO pin as an output pin

    while (1) {
        // Toggle the GPIO pin state
        gpio_set_level(led_pin, 1); // Set GPIO pin high (on)
        ESP_LOGI(TAG, "LED ON"); // Log message indicating LED is on, uses Mutex so thread-safe
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 milliseconds
        gpio_set_level(led_pin, 0); // Set GPIO pin low (off)
        ESP_LOGI(TAG, "LED OFF"); // Log message indicating LED is off
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 milliseconds
    }
}

void power_monitor_task(void *arg)
{
    i2c_master_dev_handle_t ltc_handle = (i2c_master_dev_handle_t)arg;
    
    // Buffer to hold High Byte and Low Byte (from Vin)
    uint8_t data[2];

    while (1) {
        // Read 2 bytes starting from register 0x02 (VIN High Byte)
        esp_err_t ret = ltc4151_register_read(ltc_handle, 0x02, data, 2);

        if (ret == ESP_OK) {
            // The LTC4151 uses a 12-bit ADC. 
            // data[0] holds the top 8 bits. data[1] holds the bottom 4 bits in its upper half.
            uint16_t raw_adc = (data[0] << 4) | (data[1] >> 4);

            // The datasheet states that 1 LSB = 25mV (0.025V)
            float voltage = raw_adc * 0.025;

            ESP_LOGI(TAG, "Sensor Read SUCCESS! Voltage: %.2f V", voltage);

        } else {
            ESP_LOGE(TAG, "Failed to read sensor. Error: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

// app_main is entry point, and basically starts all the tasks and initializes drivers 
void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t ltc_handle;

    i2c_master_init(&bus_handle, &ltc_handle);
    ESP_LOGI(TAG, "I2C initialized, starting tasks...");

    xTaskCreatePinnedToCore(
        strobe_led_task,        // Function that implements the task
        "strobe_led",           // Text name for debugging
        4096,                   // Stack size in bytes
        (void *)GPIO_NUM_42,    // Parameter passed into task, can use structs if more than one parameter is needed
        1,                      // Task priority (0 is lowest, configMAX_PRIORITIES-1 is highest, base is 0->24)
        NULL,                   // No task handle needed, can define a TaskHandle_t <name> = NULL; and use &<name> here if needed
        1                       // Hardcoded Core ID (1 = Core 1, 0 = Core 0, tskNO_AFFINITY = no preference)
    );

    xTaskCreatePinnedToCore(
        power_monitor_task,
        "power_monitor",
        4096,
        (void *)ltc_handle,
        2, // should be 0?
        NULL,               
        0
    );

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }

    // // Initialize the CAN hardware controller
    // ESP_ERROR_CHECK(CAN_init());
    // ESP_LOGI(TAG, "CAN initialized, starting RX loop...");

    // while (1) {
    //     CAN_message_t rx_msg;

    //     // Listen to the bus for up to 2000 milliseconds
    //     esp_err_t ret = CAN_receive(&rx_msg, 2000);
    
    //     if (ret == ESP_OK) {
    //         ESP_LOGI(TAG, "RECEIVED: ID=0x%03x, DLC=%d", rx_msg.id, rx_msg.dlc);
    //     } else if (ret == ESP_ERR_TIMEOUT) {
    //         ESP_LOGW(TAG, "TIMEOUT: No message received in last 2 seconds.");
    //     }

    //     // A tiny delay to keep FreeRTOS happy and prevent the loop from hogging the CPU
    //     vTaskDelay(pdMS_TO_TICKS(10));  
}*/