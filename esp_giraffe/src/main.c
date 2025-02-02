#include "stdio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rotary_encoder.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h" 


///////PIN BINDINGS///////
// Left Motor         ////
// PWM_FWD: D2       ////
// PWM_REV: D15       ////
// EncA: D14          ////
// EncB: D27          ////
//////////////////////////
// Right Motor        ////
// PWM_FWD: D12       ////
// PWM_REV: D13        ////
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

double kp_left = 1.2;
double ki_left = 0.4;
double kd_left = 0;

double kp_right = 1.2;
double ki_right = 0.4;
double kd_right = 0;

double setpoint_left = 0;  // desired velocity in m/s
double setpoint_right = 0; // desired velocity in m/s
double error_left;
double error_right;
double prev_error_left = 0;
double prev_error_right = 0;
double integral_left = 0;
double integral_right = 0;
double derivative_left;
double derivative_right;
double duty_cycle_left;
double duty_cycle_right;


void updatePID(double current_velocity_left, double current_velocity_right) {
    // Calculate errors
    error_left = setpoint_left - current_velocity_left;
    error_right = setpoint_right - current_velocity_right;

    // Update integral terms
    integral_left += error_left;
    integral_right += error_right;

    // Calculate derivatives
    derivative_left = error_left - prev_error_left;
    derivative_right = error_right - prev_error_right;

    // PID controller output
    double pid_output_left = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
    double pid_output_right = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;

    // Update previous errors
    prev_error_left = error_left;
    prev_error_right = error_right;

    // Convert PID output (desired RPM) to duty cycle
    duty_cycle_left = (pid_output_left / MAX_VEL) * 100.0;
    duty_cycle_right = (pid_output_right / MAX_VEL) * 100.0;

    // Constrain duty cycles between -100% and 100%
    duty_cycle_left = duty_cycle_left > 100 ? 100 : (duty_cycle_left < -100 ? -100 : duty_cycle_left);
    duty_cycle_right = duty_cycle_right > 100 ? 100 : (duty_cycle_right < -100 ? -100 : duty_cycle_right);
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
    if (left_duty_cycle > 0) {
        mcpwm_set_duty(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_A, left_duty_cycle);
        mcpwm_set_duty(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    } else {
        mcpwm_set_duty(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        mcpwm_set_duty(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_B, left_duty_cycle);
    }

    // Right Motor
    if (right_duty_cycle > 0) {
        mcpwm_set_duty(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_A, right_duty_cycle);
        mcpwm_set_duty(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    } else {
        mcpwm_set_duty(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_B, right_duty_cycle);
    }

    mcpwm_set_duty_type(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num_left, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num_right, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
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

    while (1)
    {
        set_motor_velocity(MCPWM_UNIT_0, MCPWM_UNIT_1, duty_cycle_left, duty_cycle_right);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void encoder_reader(void *arg)
{
    uint32_t pcnt_unit_enc_left = 0;   // Pulse counter unit for left encoder
    uint32_t pcnt_unit_enc_right = 1; // Pulse counter unit for right encoder

    // Configure left encoder
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

    // Variables to store encoder counts and time
    int encoder_left_val = 0, encoder_right_val = 0;
    int encoder_left_prev_val = 0, encoder_right_prev_val = 0;

    int64_t time_now = 0, time_prev = 0;
    float delta_time = 0; // Time difference in seconds
    float vel_left = 0.0;
    float vel_right = 0.0;

    // Buffer for UART logging
    uint8_t data[128];

    // Initial time
    time_prev = esp_timer_get_time(); // Time in microseconds

    while (1)
    {
        // Read current tick values
        encoder_left_val = encoder_left->get_counter_value(encoder_left);
        encoder_right_val = encoder_right->get_counter_value(encoder_right);

        // Calculate time difference
        time_now = esp_timer_get_time(); // Current time in microseconds
        delta_time = (time_now - time_prev) / 1e6; // Convert to seconds

        // Calculate velocities
        vel_left = PI * WHEEL_DIAMETER * (float)(encoder_left_val - encoder_left_prev_val) / (TICKS_PER_REV * delta_time);
        vel_right = -PI * WHEEL_DIAMETER * (float)(encoder_right_val - encoder_right_prev_val) / (TICKS_PER_REV * delta_time);

        // Log distances and velocities via UART
        int len = sprintf((char *)data, "%.2f,%.2f\n", vel_left, vel_right);
        uart_write_bytes(UART_NUM, (const char *)data, len);

        // Update previous values
        encoder_left_prev_val = encoder_left_val;
        encoder_right_prev_val = encoder_right_val;
        time_prev = time_now;

        // Wait for 500 ms before the next iteration
        updatePID(vel_left, vel_right);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void uart_reader(void *arg)
{
    uint8_t data[128];
    while (1)
    {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            data[len] = '\0';
            sscanf((char *)data, "%lf %lf", &setpoint_left, &setpoint_right);
        }
    }
}

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);

    xTaskCreate(encoder_reader, "encoder_reader", 4096, NULL, 5, NULL);
    xTaskCreate(motor_controller, "motor_controller", 4096, NULL, 4, NULL);
    xTaskCreate(uart_reader, "uart_reader", 2048, NULL, 3, NULL);
}
