#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(demo, LOG_LEVEL_INF);

#define ALERT_THREAD_STACK_SIZE 2048
#define ALERT_THREAD_PRIORITY 5

#define SAMPLE_PERIOD_MS 1000
#define ALERT_QUEUE_DEPTH 4
#define UART_LINE_LEN 160

#define TEMP_DIFF_ALERT_MC 300
#define HIGH_TEMP_ALERT_MC 28000 

enum alert_reason {
	ALERT_REASON_TEMP_DIFF,
	ALERT_REASON_HIGH_TEMP,
};

struct sensor_reading {
	int32_t temp_mc;
	uint32_t humidity_centi_pct;
};

struct alert_msg {
	uint32_t sample_id;
	enum alert_reason reason;
	int32_t htu_temp_mc;
	int32_t aht_temp_mc;
	int32_t diff_mc;
	int32_t avg_temp_mc;
	int64_t timestamp_ms;
};

K_MSGQ_DEFINE(alert_msgq, sizeof(struct alert_msg), ALERT_QUEUE_DEPTH, 4);

static const struct device *const htu21 = DEVICE_DT_GET_ONE(sensirion_sht21);
static const struct device *const aht20 = DEVICE_DT_GET_ONE(aosong_aht20);
static const struct device *const uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));

static uint32_t sample_id;
static uint32_t dropped_alerts;

static int32_t sensor_value_to_milli_celsius(const struct sensor_value *value)
{
	return (int32_t)sensor_value_to_milli(value);
}

static uint32_t sensor_value_to_centi_pct(const struct sensor_value *value)
{
	return (uint32_t)(sensor_value_to_milli(value) / 10);
}

static int32_t abs_i32(int32_t value)
{
	return value < 0 ? -value : value;
}

static void format_temp(char *buf, size_t len, int32_t temp_mc)
{
	(void)snprintf(buf, len, "%s%d.%03d",
		       temp_mc < 0 ? "-" : "",
		       abs_i32(temp_mc / 1000),
		       abs_i32(temp_mc % 1000));
}

static void format_humidity(char *buf, size_t len, uint32_t humidity_centi_pct)
{
	(void)snprintf(buf, len, "%u.%02u",
		       humidity_centi_pct / 100U,
		       humidity_centi_pct % 100U);
}

static void uart_write_string(const char *str)
{
	for (size_t i = 0; str[i] != '\0'; i++) {
		uart_poll_out(uart1, str[i]);
	}
}

static bool devices_ready(void)
{
	if (!device_is_ready(htu21)) {
		LOG_ERR("HTU21D device is not ready");
		return false;
	}

	if (!device_is_ready(aht20)) {
		LOG_ERR("AHT10/AHT20 device is not ready");
		return false;
	}

	if (!device_is_ready(uart1)) {
		LOG_ERR("UART1 device is not ready");
		return false;
	}

	return true;
}

static int read_sensor(const struct device *dev, struct sensor_reading *reading)
{
	struct sensor_value temp;
	struct sensor_value humidity;
	int err;

	err = sensor_sample_fetch(dev);
	if (err != 0) {
		return err;
	}

	err = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (err != 0) {
		return err;
	}

	err = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
	if (err != 0) {
		return err;
	}

	reading->temp_mc = sensor_value_to_milli_celsius(&temp);
	reading->humidity_centi_pct = sensor_value_to_centi_pct(&humidity);

	return 0;
}

static void submit_alert(const struct alert_msg *alert)
{
	int err = k_msgq_put(&alert_msgq, alert, K_NO_WAIT);

	if (err != 0) {
		dropped_alerts++;
		LOG_WRN("alert queue full: dropping alert for sample %u, dropped=%u",
			alert->sample_id, dropped_alerts);
	}
}

static void sensor_work_handler(struct k_work *work)
{
	struct sensor_reading htu;
	struct sensor_reading aht;
	char htu_temp[16];
	char aht_temp[16];
	char htu_hum[16];
	char aht_hum[16];
	int err;
	uint32_t id;
	int32_t diff_mc;
	int32_t avg_mc;

	ARG_UNUSED(work);

	id = ++sample_id;

	err = read_sensor(htu21, &htu);
	if (err != 0) {
		LOG_ERR("sample %u: HTU21D read failed: %d", id, err);
		return;
	}

	err = read_sensor(aht20, &aht);
	if (err != 0) {
		LOG_ERR("sample %u: AHT10/AHT20 read failed: %d", id, err);
		return;
	}

	diff_mc = abs_i32(htu.temp_mc - aht.temp_mc);
	avg_mc = (htu.temp_mc + aht.temp_mc) / 2;

	format_temp(htu_temp, sizeof(htu_temp), htu.temp_mc);
	format_temp(aht_temp, sizeof(aht_temp), aht.temp_mc);
	format_humidity(htu_hum, sizeof(htu_hum), htu.humidity_centi_pct);
	format_humidity(aht_hum, sizeof(aht_hum), aht.humidity_centi_pct);

	LOG_INF("sample %u: HTU21D=%s C/%s %%RH, AHT10=%s C/%s %%RH, diff=%d.%03d C",
		id, htu_temp, htu_hum, aht_temp, aht_hum, diff_mc / 1000, diff_mc % 1000);

	if (diff_mc > TEMP_DIFF_ALERT_MC) {
		struct alert_msg alert = {
			.sample_id = id,
			.reason = ALERT_REASON_TEMP_DIFF,
			.htu_temp_mc = htu.temp_mc,
			.aht_temp_mc = aht.temp_mc,
			.diff_mc = diff_mc,
			.avg_temp_mc = avg_mc,
			.timestamp_ms = k_uptime_get(),
		};

		submit_alert(&alert);
	}

	if (avg_mc > HIGH_TEMP_ALERT_MC) {
		struct alert_msg alert = {
			.sample_id = id,
			.reason = ALERT_REASON_HIGH_TEMP,
			.htu_temp_mc = htu.temp_mc,
			.aht_temp_mc = aht.temp_mc,
			.diff_mc = diff_mc,
			.avg_temp_mc = avg_mc,
			.timestamp_ms = k_uptime_get(),
		};

		submit_alert(&alert);
	}
}

K_WORK_DEFINE(sensor_work, sensor_work_handler);

static void sample_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	/* Timer expiry runs in interrupt context: do not read I2C sensors here. */
	(void)k_work_submit(&sensor_work);
}

K_TIMER_DEFINE(sample_timer, sample_timer_expiry, NULL);

static void alert_thread(void *p1, void *p2, void *p3)
{
	char line[UART_LINE_LEN];
	char htu_temp[16];
	char aht_temp[16];
	char avg_temp[16];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_thread_name_set(k_current_get(), "alert_thread");

	while (true) {
		struct alert_msg alert;
		const char *reason;
		int len;

		(void)k_msgq_get(&alert_msgq, &alert, K_FOREVER);

		format_temp(htu_temp, sizeof(htu_temp), alert.htu_temp_mc);
		format_temp(aht_temp, sizeof(aht_temp), alert.aht_temp_mc);
		format_temp(avg_temp, sizeof(avg_temp), alert.avg_temp_mc);

		switch (alert.reason) {
		case ALERT_REASON_TEMP_DIFF:
			reason = "SENSORS_DISAGREE";
			break;
		case ALERT_REASON_HIGH_TEMP:
			reason = "HIGH_TEMPERATURE";
			break;
		default:
			reason = "UNKNOWN";
			break;
		}

		len = snprintf(line, sizeof(line),
			       "[ACTION] sample=%u reason=%s HTU21D=%sC AHT10=%sC avg=%sC diff=%d.%03dC\r\n",
			       alert.sample_id, reason, htu_temp, aht_temp, avg_temp,
			       alert.diff_mc / 1000, alert.diff_mc % 1000);

		if (len > 0) {
			uart_write_string(line);
		}

		LOG_INF("action consumed alert from msgq: sample=%u reason=%s", alert.sample_id,
			reason);
	}
}

K_THREAD_DEFINE(alert_tid, ALERT_THREAD_STACK_SIZE, alert_thread,
		NULL, NULL, NULL, ALERT_THREAD_PRIORITY, 0, 0);

int main(void)
{
	k_thread_name_set(k_current_get(), "main");

	LOG_INF("Live Session 02: deferred sensor alert demo");
	LOG_INF("Flow: timer -> work item -> sensors -> msgq -> alert thread -> UART1");
	LOG_INF("Thresholds: diff > %d.%03d C, average > %d.%03d C",
		TEMP_DIFF_ALERT_MC / 1000, TEMP_DIFF_ALERT_MC % 1000,
		HIGH_TEMP_ALERT_MC / 1000, HIGH_TEMP_ALERT_MC % 1000);

	if (!devices_ready()) {
		LOG_ERR("Demo stopped: one or more devices are not ready");
		return 0;
	}

	uart_write_string("\r\nLS02 demo running: waiting for sensor threshold alerts...\r\n");

	k_timer_start(&sample_timer, K_MSEC(SAMPLE_PERIOD_MS), K_MSEC(SAMPLE_PERIOD_MS));

	return 0;
}
