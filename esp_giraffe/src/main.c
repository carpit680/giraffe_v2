#include "stdio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"   // Include the semaphore header
#include "esp_log.h"
#include "rotary_encoder.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h" 
#include <math.h>

///////PIN BINDINGS///////
// Left Motor         ////
// PWM_FWD: D2       ////
// PWM_REV: D15       ////
// EncA: D14          ////
// EncB: D27          ////
//////////////////////////
// Right Motor        ////
// PWM_FWD: D12       ////
// PWM_REV: D13       ////
// EncA: D23          ////
// EncB: D22          ////
//////////////////////////

#define GPIO_PWM_FWD_LEFT 2
#define GPIO_PWM_REV_LEFT 15

#define GPIO_PWM_FWD_RIGHT 12
#define GPIO_PWM_REV_RIGHT 13

#define PI 3.14159
#define MAX_VEL 1.0
#define BUF_SIZE 1024
#define TICKS_PER_REV 559.2
#define UART_NUM UART_NUM_0
#define WHEEL_DIAMETER 0.0955

float kp_left = 2.0;
float ki_left = 1.5;
float kd_left = 0.01;

float kp_right = 2.0;
float ki_right = 1.5;
float kd_right = 0.01;

float setpoint_left = 0;  // desired velocity in m/s
float setpoint_right = 0; // desired velocity in m/s
float error_left;
float error_right;
float prev_error_left = 0;
float prev_error_right = 0;
float integral_left = 0;
float integral_right = 0;
float derivative_left;
float derivative_right;

// These are shared between tasks so we protect them with a mutex
float duty_cycle_left;
float duty_cycle_right;

// Declare a global mutex handle
SemaphoreHandle_t dutyCycleMutex = NULL;

void updatePID(float current_velocity_left, float current_velocity_right, float dt) {
    // Compute errors (adjust sign if needed for your hardware)
    error_left = setpoint_left - current_velocity_left;
    error_right = setpoint_right - current_velocity_right;

    // Update integral (using dt) and derivative terms
    integral_left += error_left * dt;
    integral_right += error_right * dt;
    derivative_left = (error_left - prev_error_left) / dt;
    derivative_right = (error_right - prev_error_right) / dt;

    float pid_output_left = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
    float pid_output_right = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;

    prev_error_left = error_left;
    prev_error_right = error_right;

    // Convert output to a duty cycle percentage
    float new_duty_left = (pid_output_left / MAX_VEL) * 100.0;
    float new_duty_right = (pid_output_right / MAX_VEL) * 100.0;

    // Clamp between -100% and 100%
    if (new_duty_left > 100) new_duty_left = 100;
    if (new_duty_left < -100) new_duty_left = -100;
    if (new_duty_right > 100) new_duty_right = 100;
    if (new_duty_right < -100) new_duty_right = -100;

    // Use mutex to update the shared duty cycle variables safely
    xSemaphoreTake(dutyCycleMutex, portMAX_DELAY);
    duty_cycle_left = new_duty_left;
    duty_cycle_right = new_duty_right;
    xSemaphoreGive(dutyCycleMutex);
}

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_FWD_LEFT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM_REV_LEFT);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, GPIO_PWM_FWD_RIGHT);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, GPIO_PWM_REV_RIGHT);
}

static void set_motor_velocity(
    mcpwm_unit_t mcpwm_num_left,
    mcpwm_unit_t mcpwm_num_right,
    float left_duty_cycle,
    float right_duty_cycle)
{
    // Left Motor
    if (left_duty_cycle >= 0) {
        mcpwm_set_duty(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_A, fabs(left_duty_cycle));
        mcpwm_set_duty(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    } else {
        mcpwm_set_duty(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        mcpwm_set_duty(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_B, fabs(left_duty_cycle));
    }

    // Right Motor
    if (right_duty_cycle >= 0) {
        mcpwm_set_duty(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_A, fabs(right_duty_cycle));
        mcpwm_set_duty(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    } else {
        mcpwm_set_duty(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_B, fabs(right_duty_cycle));
    }
}

void motor_controller(void *arg)
{
    mcpwm_example_gpio_initialize();

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

    while (1)
    {
        float dc_left, dc_right;
        // Safely read the duty cycles using the mutex
        xSemaphoreTake(dutyCycleMutex, portMAX_DELAY);
        dc_left = duty_cycle_left;
        dc_right = duty_cycle_right;
        xSemaphoreGive(dutyCycleMutex);

        set_motor_velocity(MCPWM_UNIT_0, MCPWM_UNIT_1, dc_left, dc_right);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void encoder_reader(void *arg)
{
    uint32_t pcnt_unit_enc_left = 0;   // Pulse counter unit for left encoder
    uint32_t pcnt_unit_enc_right = 1;    // Pulse counter unit for right encoder

    // Configure left encoder
    // Note: The order of the pins may be adjusted depending on your wiring.
    rotary_encoder_config_t config_encoder_left = ROTARY_ENCODER_DEFAULT_CONFIG(
        (rotary_encoder_dev_t)pcnt_unit_enc_left, 14, 27);
    rotary_encoder_t *encoder_left = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_left, &encoder_left));
    ESP_ERROR_CHECK(encoder_left->set_glitch_filter(encoder_left, 1));
    ESP_ERROR_CHECK(encoder_left->start(encoder_left));

    // Configure right encoder
    rotary_encoder_config_t config_encoder_right = ROTARY_ENCODER_DEFAULT_CONFIG(
        (rotary_encoder_dev_t)pcnt_unit_enc_right, 23, 22);
    rotary_encoder_t *encoder_right = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_right, &encoder_right));
    ESP_ERROR_CHECK(encoder_right->set_glitch_filter(encoder_right, 1));
    ESP_ERROR_CHECK(encoder_right->start(encoder_right));

    int encoder_left_prev_val = encoder_left->get_counter_value(encoder_left);
    int encoder_right_prev_val = encoder_right->get_counter_value(encoder_right);
    int64_t time_prev = esp_timer_get_time(); // in microseconds

    // Variables for accumulating counts over a longer interval for reporting
    int accumulated_left_ticks = 0;
    int accumulated_right_ticks = 0;
    int64_t accumulated_time = 0;
    const int64_t reporting_interval_us = 100000; // 100ms

    uint8_t data[128];

    while (1)
    {
        int encoder_left_val = encoder_left->get_counter_value(encoder_left);
        int encoder_right_val = encoder_right->get_counter_value(encoder_right);

        int ticks_left = encoder_left_val - encoder_left_prev_val;
        int ticks_right = encoder_right_val - encoder_right_prev_val;
        encoder_left_prev_val = encoder_left_val;
        encoder_right_prev_val = encoder_right_val;

        int64_t time_now = esp_timer_get_time();
        int64_t dt_us = time_now - time_prev;
        time_prev = time_now;

        // Accumulate ticks and time for reporting
        accumulated_left_ticks += ticks_left;
        accumulated_right_ticks += ticks_right;
        accumulated_time += dt_us;

        // Use the short interval dt for PID update
        float dt = dt_us / 1e6f;
        float vel_left = PI * WHEEL_DIAMETER * ticks_left / (TICKS_PER_REV * dt);
        float vel_right = -PI * WHEEL_DIAMETER * ticks_right / (TICKS_PER_REV * dt);
        updatePID(vel_left, vel_right, dt);

        // Report velocity every 100ms based on accumulated counts
        if (accumulated_time >= reporting_interval_us) {
            float dt_acc = accumulated_time / 1e6f;
            float avg_vel_left = PI * WHEEL_DIAMETER * accumulated_left_ticks / (TICKS_PER_REV * dt_acc);
            float avg_vel_right = -PI * WHEEL_DIAMETER * accumulated_right_ticks / (TICKS_PER_REV * dt_acc);
            int len = sprintf((char *)data, "%.2f,%.2f\n", avg_vel_left, avg_vel_right);
            uart_write_bytes(UART_NUM, (const char *)data, len);

            // Reset accumulators
            accumulated_left_ticks = 0;
            accumulated_right_ticks = 0;
            accumulated_time = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Keep control loop fast
    }
}

// void uart_reader(void *arg)
// {
//     uint8_t data[128];
//     while (1)
//     {
//         int len = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
//         if (len > 0)
//         {
//             data[len] = '\0';
//             sscanf((char *)data, "%f %f", &setpoint_left, &setpoint_right);
//         }
//     }
// }

void uart_reader(void *arg)
{
    uint8_t data[128];
    while (1)
    {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            data[len] = '\0';
            // Use sscanf with the pattern "<%f %f>" to extract the two float values.
            if (sscanf((char *)data, "<%f %f>", &setpoint_left, &setpoint_right) == 2)
            {
                // Successfully parsed the two velocities.
            }
            // else
            // {
            //     // Parsing failed; optionally, log or handle the error.
            // }
        }
    }
}

void app_main(void)
{
    // Create the mutex before tasks start using it
    dutyCycleMutex = xSemaphoreCreateMutex();
    if (dutyCycleMutex == NULL) {
        printf("Error creating dutyCycleMutex\n");
        return;
    }
    
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);

    xTaskCreate(encoder_reader, "encoder_reader", 4096, NULL, 5, NULL);
    xTaskCreate(motor_controller, "motor_controller", 4096, NULL, 4, NULL);
    xTaskCreate(uart_reader, "uart_reader", 2048, NULL, 3, NULL);
}
