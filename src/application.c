#include <application.h>
#include <math.h>

// Service mode interval defines how much time
#define SERVICE_MODE_INTERVAL (15 * 60 * 1000)
#define BATTERY_UPDATE_INTERVAL (1 * 20 * 1000)
#define ACCELEROMETER_UPDATE_SERVICE_INTERVAL (200)
#define ACCELEROMETER_UPDATE_NORMAL_INTERVAL (200)

// LED instance
twr_led_t led;

// Button instance
twr_button_t button;

// Thermometer instance
twr_tmp112_t tmp112;

// Accelerometer instance
twr_lis2dh12_t lis2dh12;

// Counters for button events
uint16_t button_click_count = 0;
uint16_t button_hold_count = 0;

// Time of button has press
twr_tick_t tick_start_button_press;
// Flag for button hold event
bool button_hold_event;

typedef enum {
    FALL_STATE_NORMAL,
    FALL_STATE_ACTIVITY_DETECTED,
    FALL_STATE_IMPACT_DETECTED,
    FALL_STATE_MONITORING_STATIC
} fall_detection_state_t;

// Variables to track fall detection
fall_detection_state_t fall_state = FALL_STATE_NORMAL;
twr_tick_t activity_start = 0;
twr_tick_t impact_time = 0;
float last_static_magnitude = 1.0f;  // Initialize to normal gravity

// Function to send custom JSON log including device ID
void log_json_with_device_id(const char *key, const char *value)
{
    char json_message[100];
    snprintf(json_message, sizeof(json_message), "{\"id_device\":\"%llx\",\"%s\":\"%s\"}", twr_radio_get_my_id(), key, value);
    twr_log_info("%s", json_message);
}

//void log_json_with_device_id_int(const char *key, int value) UZITECNA JEN POKUD BUDEM CHTIT DURATION ZMACKNUTI
//{
//    char json_message[100];
//    snprintf(json_message, sizeof(json_message), "{\"id_device\":\"%llx\",\"%s\":%d}", twr_radio_get_my_id(), key, value);
//    twr_log_info("%s", json_message);
//}

void log_json_with_device_id_float(const char *key, float value)
{
    char json_message[100];
    snprintf(json_message, sizeof(json_message), "{\"id_device\":\"%llx\",\"%s\":%.2f}", twr_radio_get_my_id(), key, value);
    twr_log_info("%s", json_message);
}



// This function dispatches button events
void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    if (event == TWR_BUTTON_EVENT_CLICK)
    {
        // Pulse LED for 100 milliseconds
        twr_led_pulse(&led, 100);

        log_json_with_device_id("button_click", "true");


    }
    else if (event == TWR_BUTTON_EVENT_HOLD)
    {
        // Pulse LED for 250 milliseconds
        twr_led_pulse(&led, 250);

        log_json_with_device_id("button_hold", "true");

        // Set button hold event flag
        button_hold_event = true;

    }
    else if (event == TWR_BUTTON_EVENT_PRESS)
    {
        // Reset button hold event flag
        button_hold_event = false;

        tick_start_button_press = twr_tick_get();
    }
    //else if (event == TWR_BUTTON_EVENT_RELEASE)
    //{
    //    if (button_hold_event)
    //    {
    //        int hold_duration = twr_tick_get() - tick_start_button_press;
    //        log_json_with_device_id_int("button_hold_duration", hold_duration);
    //    }
    //}   FUNKCE NA POCITANI DURATION ZMACKNUTI
}

float battery_voltage_to_percentage(float voltage)
{
    // Define voltage range for 1.5V AAA alkaline battery
    const float voltage_max = 3.1f;  // Fresh battery voltage
    const float voltage_min = 1.5f;  // Considered "empty" for most devices
    
    // Constrain voltage to min/max range
    if (voltage > voltage_max) voltage = voltage_max;
    if (voltage < voltage_min) voltage = voltage_min;
    
    // Calculate percentage (non-linear approximation)
    // Alkaline batteries have a more gradual initial drop, then steep decline
    
    // Simple approximation:
    float percentage = ((voltage - voltage_min) / (voltage_max - voltage_min)) * 100.0f;
    
    return percentage;
}
// This function dispatches battery events
void battery_event_handler(twr_module_battery_event_t event, void *event_param)
{
    // Update event?
    if (event == TWR_MODULE_BATTERY_EVENT_UPDATE)
    {
        float voltage;

        // Read battery voltage
        if (twr_module_battery_get_voltage(&voltage))
        {
            // Calculate and log battery percentage
            float percentage = battery_voltage_to_percentage(voltage);
            log_json_with_device_id_float("battery_percentage", percentage);
        }
    }
}


void lis2dh12_event_handler(twr_lis2dh12_t *self, twr_lis2dh12_event_t event, void *event_param)
{
    // This function dispatches accelerometer events
    if (event == TWR_LIS2DH12_EVENT_UPDATE)
    {
        twr_lis2dh12_result_g_t result;
        static float prev_acc_magnitude = 1.0f;  // Initialize to normal gravity
        static bool monitoring_static = false;
        static twr_tick_t acceleration_time = 0;
        static twr_tick_t static_start_time = 0;
        const float static_threshold = 0.15f;  // Threshold for static detection (close to 1G with minimal variation)
        const float acceleration_threshold = 0.2f;  // Threshold for sudden acceleration detection (change between readings)
        const uint32_t static_duration = 2000;    // Duration in ms to confirm static position

        // Successfully read accelerometer vectors?
        if (twr_lis2dh12_get_result_g(self, &result))
        {
            // Calculate magnitude of acceleration vector
            float acc_magnitude = sqrtf(result.x_axis * result.x_axis + 
                                       result.y_axis * result.y_axis + 
                                       result.z_axis * result.z_axis);
            
            
            twr_tick_t current_time = twr_tick_get();
            
            // Calculate the change in acceleration magnitude
            float acc_delta = fabsf(acc_magnitude - prev_acc_magnitude);
            
            // Check for sudden acceleration change
            if (!monitoring_static && acc_magnitude < acceleration_threshold) 
            {
                acceleration_time = current_time;
                monitoring_static = true;
                static_start_time = 0;  // Will be set when we start detecting static position
            }
            
            // After sudden acceleration, check for static position (no movement)
            if (monitoring_static) 
            {
                // Check if acceleration is stable (close to 1G with minimal variation)
                bool is_static = (fabsf(acc_magnitude - 1.0f) < static_threshold) && (acc_delta < static_threshold);
                
                if (is_static) 
                {
                    // If we haven't started timing static position yet
                    if (static_start_time == 0) 
                    {
                        static_start_time = current_time;
                    }
                    // Check if static for long enough duration
                    else if ((current_time - static_start_time) >= static_duration) 
                    {
                        log_json_with_device_id("patient_fell", "true");
                        
                        // Reset monitoring state
                        monitoring_static = false;
                    }
                } 
                else 
                {
                    // Movement detected, reset static timing
                    static_start_time = 0;
                }
                
                // If we've been monitoring for too long without detecting static position, reset
                if ((current_time - acceleration_time) > 10000) // 10 seconds timeout
                {
                    monitoring_static = false;
                    log_json_with_device_id("event", "static_detection_timeout");
                }
            }
            
            // Store current acceleration for next comparison
            prev_acc_magnitude = acc_magnitude;
        }
    }
    // Error event?
    else if (event == TWR_LIS2DH12_EVENT_ERROR)
    {
        twr_log_error("APP: Accelerometer error");
    }
}

// This function is run as task and exits service mode
void exit_service_mode_task(void *param)
{
    // Set accelerometer update interval to normal
    twr_lis2dh12_set_update_interval(&lis2dh12, ACCELEROMETER_UPDATE_NORMAL_INTERVAL);

    // Unregister current task (it has only one-shot purpose)
    twr_scheduler_unregister(twr_scheduler_get_current_task_id());
}

void application_init(void)
{
    // Initialize log
    twr_log_init(TWR_LOG_LEVEL_INFO, TWR_LOG_TIMESTAMP_ABS);
    twr_log_info("APP: Reset");

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, false);
    twr_led_set_mode(&led, TWR_LED_MODE_OFF);

    // Initialize button
    twr_button_init(&button, TWR_GPIO_BUTTON, TWR_GPIO_PULL_DOWN, false);
    twr_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize battery
    twr_module_battery_init();
    twr_module_battery_set_event_handler(battery_event_handler, NULL);
    twr_module_battery_set_update_interval(BATTERY_UPDATE_INTERVAL);

    // Initialize accelerometer
    twr_lis2dh12_init(&lis2dh12, TWR_I2C_I2C0, 0x19);
    twr_lis2dh12_set_event_handler(&lis2dh12, lis2dh12_event_handler, NULL);
    twr_lis2dh12_set_update_interval(&lis2dh12, ACCELEROMETER_UPDATE_SERVICE_INTERVAL);

    // Initialize Radio
    twr_radio_init(TWR_RADIO_MODE_NODE_SLEEPING);

    // Initialize USB
    twr_usb_cdc_init();

    twr_scheduler_register(exit_service_mode_task, NULL, SERVICE_MODE_INTERVAL);

    // Pulse LED
    twr_led_pulse(&led, 2000);
}