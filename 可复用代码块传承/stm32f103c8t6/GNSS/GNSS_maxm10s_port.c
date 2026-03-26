#include "GNSS_maxm10s_port.h"

#include <string.h>

#define GNSS_MAXM10S_RX_BUF_SIZE 512u

typedef struct {
    uint8_t buf[GNSS_MAXM10S_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} gnss_rb_t;

static UART_HandleTypeDef *s_uart = NULL;
static gnss_rb_t s_rb;

static void rb_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rb.head + 1u) % GNSS_MAXM10S_RX_BUF_SIZE);
    if (next == s_rb.tail) {
        s_rb.tail = (uint16_t)((s_rb.tail + 1u) % GNSS_MAXM10S_RX_BUF_SIZE);
    }
    s_rb.buf[s_rb.head] = byte;
    s_rb.head = next;
}

void GNSS_maxm10s_port_init(UART_HandleTypeDef *uart)
{
    s_uart = uart;
    memset(&s_rb, 0, sizeof(s_rb));
}

static void poll_rx_bytes(void)
{
    uint8_t byte;

    if (s_uart == NULL) {
        return;
    }

    while (HAL_UART_Receive(s_uart, &byte, 1, 0) == HAL_OK) {
        rb_push(byte);
    }
}

bool GNSS_maxm10s_port_read_line(char *line, uint16_t max_len)
{
    uint16_t idx = 0;
    bool seen_cr = false;

    if ((line == NULL) || (max_len < 2u)) {
        return false;
    }

    poll_rx_bytes();

    while ((s_rb.tail != s_rb.head) && (idx < (uint16_t)(max_len - 1u))) {
        uint8_t byte = s_rb.buf[s_rb.tail];
        s_rb.tail = (uint16_t)((s_rb.tail + 1u) % GNSS_MAXM10S_RX_BUF_SIZE);

        line[idx++] = (char)byte;

        if (byte == '\r') {
            seen_cr = true;
        } else if ((byte == '\n') && seen_cr) {
            if (idx >= 2u) {
                line[idx - 2u] = '\0';
            } else {
                line[0] = '\0';
            }
            return true;
        }
    }

    line[(idx < max_len) ? idx : (uint16_t)(max_len - 1u)] = '\0';
    return false;
}
