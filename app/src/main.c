#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(demo, LOG_LEVEL_DBG);

#define STACK_SIZE 1024

#define PRIO_A 5
#define PRIO_B 5

#define TX_BUFFER_LEN   100

#define TA_WAIT_PERIOD_MS  100
#define TB_WAIT_PERIOD_MS  100

static K_MUTEX_DEFINE(tx_buffer_mutex);
static K_SEM_DEFINE(uart_tx_sem, 0, 1);

const struct device *const htu21 = DEVICE_DT_GET_ONE(sensirion_sht21);
const struct device *const aht20 = DEVICE_DT_GET_ONE(aosong_aht20);
const struct device *const uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));

static char tx_buffer[TX_BUFFER_LEN] = {0};

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	int rc;

	LOG_INF("EVENT: %d", evt->type);

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("TX complete");
		k_sem_give(&uart_tx_sem);
		break;
	case UART_RX_BUF_REQUEST:
		break;
	case UART_RX_BUF_RELEASED:
	case UART_RX_DISABLED:
		break;
	case UART_RX_RDY:
		break;
	default:
		LOG_WRN("Unhandled event %d", evt->type);
	}
}

void thread_a_fn(void *p1, void *p2, void *p3)
{
    if(htu21 == NULL) {
        LOG_ERR("Sensor device not found");
        return;
    }

    if(!device_is_ready(htu21)) {
        LOG_ERR("Sensor device not ready");
        return;
    }

    if(!device_is_ready(uart1)) {
        LOG_ERR("uart1 device not ready");
        return;
    }

    LOG_INF("Found htu21 device %s", htu21->name);

    struct sensor_value temp, hum;

    while (1) {
        int rc = sensor_sample_fetch(htu21);

		if (rc == 0) {
			rc = sensor_channel_get(htu21, SENSOR_CHAN_AMBIENT_TEMP,
						&temp);
		}
		if (rc == 0) {
			rc = sensor_channel_get(htu21, SENSOR_CHAN_HUMIDITY,
						&hum);
		}
		if (rc != 0) {
			LOG_ERR("HTU21D: failed: %d\n", rc);
			break;
		}
        LOG_INF("[A] HTU21D: %.2f Cel ; %0.2f %%RH\n",
		       sensor_value_to_double(&temp),
		       sensor_value_to_double(&hum));
        
        int lock_state = k_mutex_lock(&tx_buffer_mutex, K_FOREVER);
	    __ASSERT_NO_MSG(lock_state == 0);
	    ARG_UNUSED(lock_state);

        int len = snprintf(tx_buffer, TX_BUFFER_LEN, "[A] HTU21D: %.2f Cel ; %0.2f %%RH\n",
		       sensor_value_to_double(&temp),
		       sensor_value_to_double(&hum));

        rc = uart_tx(uart1, tx_buffer, len, SYS_FOREVER_US);
        k_sem_take(&uart_tx_sem, K_FOREVER);
        k_mutex_unlock(&tx_buffer_mutex);

        k_msleep(TA_WAIT_PERIOD_MS);
    }
}

void thread_b_fn(void *p1, void *p2, void *p3)
{
    if(aht20 == NULL) {
        LOG_ERR("Sensor device not found");
        return;
    }

    if(!device_is_ready(aht20)) {
        LOG_ERR("aht20 device not ready");
        return;
    }

    if(!device_is_ready(uart1)) {
        LOG_ERR("uart1 device not ready");
        return;
    }

    LOG_INF("Found aht20 device %s", aht20->name);

    struct sensor_value temp, hum;

    while (1) {
        int rc = sensor_sample_fetch(aht20);

		if (rc == 0) {
			rc = sensor_channel_get(aht20, SENSOR_CHAN_AMBIENT_TEMP,
						&temp);
		}
		if (rc == 0) {
			rc = sensor_channel_get(aht20, SENSOR_CHAN_HUMIDITY,
						&hum);
		}
		if (rc != 0) {
			LOG_ERR("AHT10: failed: %d\n", rc);
			break;
		}
        LOG_INF("[B] AHT10: %.2f Cel ; %0.2f %%RH\n",
		       sensor_value_to_double(&temp),
		       sensor_value_to_double(&hum));

        int lock_state = k_mutex_lock(&tx_buffer_mutex, K_FOREVER);
        int len = snprintf(tx_buffer, TX_BUFFER_LEN, "[B] AHT10: %.2f Cel ; %0.2f %%RH\n",
		       sensor_value_to_double(&temp),
		       sensor_value_to_double(&hum));

        rc = uart_tx(uart1, tx_buffer, len, SYS_FOREVER_US);
        k_sem_take(&uart_tx_sem, K_FOREVER);
        k_mutex_unlock(&tx_buffer_mutex);

        k_msleep(TB_WAIT_PERIOD_MS);
    }
}

K_THREAD_DEFINE(thread_a, STACK_SIZE, thread_a_fn,
                NULL, NULL, NULL, PRIO_A, 0, 0);
K_THREAD_DEFINE(thread_b, STACK_SIZE, thread_b_fn,
                NULL, NULL, NULL, PRIO_B, 0, 0);

int main(void)
{
    LOG_INF("=== LS1 Demo 1: Thread Synchronization ===");
    uart_callback_set(uart1, uart_callback, NULL);
    return 0;
}

