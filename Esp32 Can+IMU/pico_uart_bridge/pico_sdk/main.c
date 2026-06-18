#include "hardware/uart.h"
#include "pico/stdlib.h"

#define UART_ID      uart0
#define BAUD_RATE    115200
#define UART_TX_PIN  4
#define UART_RX_PIN  5

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif
#define LED_PIN PICO_DEFAULT_LED_PIN

#define HEARTBEAT_US      1000000
#define ACTIVITY_PULSE_US   30000

static absolute_time_t activity_until;
static absolute_time_t last_heartbeat;
static bool heartbeat_state = false;

static void pulse_activity(void) {
    activity_until = make_timeout_time_us(ACTIVITY_PULSE_US);
    gpio_put(LED_PIN, 1);
}

static void update_led(void) {
    if (!time_reached(activity_until)) {
        gpio_put(LED_PIN, 1);
        return;
    }
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_heartbeat, now) >= HEARTBEAT_US) {
        heartbeat_state = !heartbeat_state;
        gpio_put(LED_PIN, heartbeat_state);
        last_heartbeat = now;
    }
}

int main(void) {
    stdio_init_all();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    activity_until = get_absolute_time();
    last_heartbeat = get_absolute_time();

    while (true) {
        // USB → UART
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            uart_putc_raw(UART_ID, (char)c);
            pulse_activity();
        }

        // UART → USB
        if (uart_is_readable(UART_ID)) {
            putchar_raw(uart_getc(UART_ID));
            pulse_activity();
        }

        update_led();
    }
}
