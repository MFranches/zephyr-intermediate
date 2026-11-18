#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(ls01, LOG_LEVEL_DBG);

#define STACK_SIZE 1024
#define TLS_PARSER_STACK_SIZE 1024

#define TLS_PARSER_PRIO 4

#define TX_BUFFER_LEN   100
#define RX_CHUNK_LEN 32

/*
 * Classroom switch:
 * 1 = a 1536-byte TLS reassembly context is automatic (on the parser stack).
 * 0 = exactly the same context is static/global instead.
 *
 * Keep TLS_PARSER_STACK_SIZE unchanged while changing this switch. That makes
 * the memory-placement lesson visible: the fix is not a larger thread stack.
 */
#define TLS_PARSE_BUFFER_IS_LOCAL 0
#define TLS_PARSE_BUFFER_SIZE 1536

#define TA_WAIT_PERIOD_MS  100
#define TB_WAIT_PERIOD_MS  100

static K_MUTEX_DEFINE(tx_buffer_mutex);
static K_SEM_DEFINE(uart_tx_sem, 0, 1);

struct tls_rx_fragment {
	uint8_t data[RX_CHUNK_LEN];
	uint8_t len;
};

K_MSGQ_DEFINE(tls_rx_msgq, sizeof(struct tls_rx_fragment), 8, 4);

static uint8_t async_rx_buffer[2][RX_CHUNK_LEN];
static volatile uint8_t async_rx_buffer_idx;

struct tls_parser_context {
	uint8_t certificate[TLS_PARSE_BUFFER_SIZE];
	size_t certificate_len;
};
static volatile uint8_t tls_parse_checksum;

volatile struct tls_parser_context global_parser_context;


const struct device *const uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));

static char tx_buffer[TX_BUFFER_LEN] = {0};

/*
 * This is intentionally unsafe when TLS_PARSE_BUFFER_IS_LOCAL is 1. The
 * reassembly context is larger than tls_parser's 1024-byte stack. A received
 * UART fragment reaches this function through tls_parser_thread(), then the
 * MPU stack guard reports a fatal stack overflow.
 *
 * Even if that thread had a bigger stack, this local context would be reset
 * on every call and could not assemble a multi-fragment certificate. The
 * static/global alternative preserves the context between calls.
 */
__noinline static void stage_tls_certificate_fragment(const struct tls_rx_fragment *fragment)
{
	volatile struct tls_parser_context *parser_context = &global_parser_context;


	/* Append the fragment to the certificate reassembly context. */
	for (size_t i = 0; i < fragment->len; i++) {
		if (parser_context->certificate_len == TLS_PARSE_BUFFER_SIZE) {
			LOG_ERR("TLS certificate is too large; discarding it");
			parser_context->certificate_len = 0;
		}

		parser_context->certificate[parser_context->certificate_len++] = fragment->data[i];
	}

	/* A newline marks the end of this classroom 'certificate'. */
	if ((parser_context->certificate_len > 0U) &&
	    (parser_context->certificate[parser_context->certificate_len - 1U] == '\n')) {
		tls_parse_checksum = 0;
		for (size_t i = 0; i < parser_context->certificate_len; i++) {
			tls_parse_checksum ^= parser_context->certificate[i];
		}

		LOG_INF("TLS certificate reassembled: %u bytes", parser_context->certificate_len);
		parser_context->certificate_len = 0;
	}
}

static void tls_parser_thread(void *p1, void *p2, void *p3)
{
	struct tls_rx_fragment fragment;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_msgq_get(&tls_rx_msgq, &fragment, K_FOREVER);
		LOG_INF("TLS parser: staging %u-byte certificate fragment", fragment.len);
		stage_tls_certificate_fragment(&fragment);
		LOG_INF("TLS parser: fragment handled (checksum %u)", tls_parse_checksum);
	}
}

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	int rc;
	switch (evt->type) {
	case UART_TX_DONE:
		LOG_INF("TX complete");
		k_sem_give(&uart_tx_sem);
		break;
	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART requests the next RX buffer");
		rc = uart_rx_buf_rsp(dev, async_rx_buffer[async_rx_buffer_idx],
				     sizeof(async_rx_buffer[0]));
		__ASSERT_NO_MSG(rc == 0);
		async_rx_buffer_idx = async_rx_buffer_idx ? 0 : 1;
		break;
	case UART_RX_BUF_RELEASED:
		break;
	case UART_RX_DISABLED:
		async_rx_buffer_idx = async_rx_buffer_idx ? 0 : 1;
		uart_rx_enable(uart1, async_rx_buffer[async_rx_buffer_idx], RX_CHUNK_LEN, 100);
		break;
	case UART_RX_RDY: {
		struct tls_rx_fragment fragment = {
			.len = evt->data.rx.len,
		};

		for (size_t i = 0; i < fragment.len; i++) {
			fragment.data[i] = evt->data.rx.buf[evt->data.rx.offset + i];
		}

		// LOG_HEXDUMP_INF(fragment.data, fragment.len, "UART RX fragment");
		rc = k_msgq_put(&tls_rx_msgq, &fragment, K_NO_WAIT);
		if (rc != 0) {
			LOG_WRN("TLS RX queue full: fragment dropped");
		}
		break;
	}
	default:
		break;
	}
}

K_THREAD_DEFINE(tls_parser, TLS_PARSER_STACK_SIZE, tls_parser_thread,
		NULL, NULL, NULL, TLS_PARSER_PRIO, 0, 0);

int main(void)
{
	int rc = 0;
    LOG_INF("=== LS3 Demo: Memory and Debugging ===");
	__ASSERT_NO_MSG(device_is_ready(uart1));

    rc = uart_callback_set(uart1, uart_callback, NULL);
    __ASSERT_NO_MSG(rc == 0);

    async_rx_buffer_idx = 1;
    rc = uart_rx_enable(uart1, async_rx_buffer[0], RX_CHUNK_LEN, 100);
    __ASSERT_NO_MSG(rc == 0);

	LOG_INF("Send newline-terminated data to UART1 to start the TLS parser demo");
    return 0;
}
