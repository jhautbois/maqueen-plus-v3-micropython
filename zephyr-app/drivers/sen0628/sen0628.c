/*
 * SEN0628 8x8 ToF LIDAR Sensor Driver
 *
 * Driver for DFRobot SEN0628 Matrix ToF sensor.
 * Communicates with the onboard RP2040 via I2C.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "sen0628.h"
#include "sen0628_proto.h"

/* Device tree compatible string: "dfrobot,sen0628" -> dfrobot_sen0628 */
#define DT_DRV_COMPAT dfrobot_sen0628

LOG_MODULE_REGISTER(sen0628, CONFIG_SEN0628_LOG_LEVEL);

/*
 * Driver configuration (from device tree)
 */
struct sen0628_config {
	struct i2c_dt_spec i2c;
	uint8_t initial_mode;
};

/*
 * Driver runtime data
 */
struct sen0628_data {
	struct k_mutex lock;            /* Thread safety for cached data */
	uint8_t mode;
	bool initialized;
	uint8_t error_count;            /* Errors in last scan */
	struct sen0628_scan last_scan;
};

/*
 * Protocol Layer: Build command packet
 */
static int build_packet(uint8_t *buf, size_t buf_size,
			uint8_t cmd, const uint8_t *args, size_t args_len)
{
	size_t pkt_len = SEN0628_HEADER_SIZE + args_len;

	if (pkt_len > buf_size) {
		return -ENOMEM;
	}

	buf[0] = SEN0628_PACKET_HEAD;
	buf[1] = (args_len >> 8) & 0xFF;    /* ArgsNumH (MSB) */
	buf[2] = args_len & 0xFF;           /* ArgsNumL (LSB) */
	buf[3] = cmd;

	if (args && args_len > 0) {
		memcpy(&buf[4], args, args_len);
	}

	return pkt_len;
}

/*
 * Protocol Layer: Send packet with chunking
 */
static int send_packet(const struct i2c_dt_spec *i2c,
		       const uint8_t *pkt, size_t pkt_len)
{
	int ret;

	if (pkt_len <= SEN0628_I2C_MAX_CHUNK) {
		/* Single transfer */
		ret = i2c_write_dt(i2c, pkt, pkt_len);
	} else {
		/* Chunked transfer */
		size_t offset = 0;
		while (offset < pkt_len) {
			size_t chunk = MIN(pkt_len - offset, SEN0628_I2C_MAX_CHUNK);
			ret = i2c_write_dt(i2c, &pkt[offset], chunk);
			if (ret < 0) {
				return ret;
			}
			offset += chunk;
			if (offset < pkt_len) {
				k_sleep(K_MSEC(SEN0628_CHUNK_DELAY_MS));
			}
		}
	}

	return ret;
}

/*
 * Protocol Layer: Receive response packet with retry
 */
static int recv_packet(const struct i2c_dt_spec *i2c,
		       uint8_t *status, uint8_t *cmd,
		       uint8_t *data, size_t *data_len, size_t max_len)
{
	uint8_t header[4];
	int ret;
	uint16_t len;

	/* Read header */
	ret = i2c_read_dt(i2c, header, sizeof(header));
	if (ret < 0) {
		LOG_ERR("Failed to read response header: %d", ret);
		return ret;
	}

	*status = header[0];
	*cmd = header[1];
	len = header[2] | (header[3] << 8);  /* Little-endian */

	LOG_DBG("Response: status=0x%02x cmd=0x%02x len=%u", *status, *cmd, len);

	/* Validate response - check for corrupted header */
	if (*status != SEN0628_STATUS_SUCCESS && *status != SEN0628_STATUS_FAILURE) {
		LOG_WRN("Invalid status byte: 0x%02x (expected 0x53 or 0x63)", *status);
		*data_len = 0;
		return -EAGAIN;  /* Sensor not ready, caller can retry */
	}

	if (len > max_len) {
		LOG_WRN("Response truncated: %u > %zu", len, max_len);
		len = max_len;
	}

	*data_len = len;

	if (len == 0) {
		return 0;
	}

	/* Read data with chunking */
	size_t offset = 0;
	while (offset < len) {
		size_t chunk = MIN(len - offset, SEN0628_I2C_MAX_CHUNK);
		ret = i2c_read_dt(i2c, &data[offset], chunk);
		if (ret < 0) {
			LOG_ERR("Failed to read response data: %d", ret);
			return ret;
		}
		offset += chunk;
		if (offset < len) {
			k_sleep(K_MSEC(SEN0628_CHUNK_DELAY_MS));
		}
	}

	return 0;
}

/*
 * Send command and receive response with retry support
 */
static int send_command_with_retry(const struct device *dev,
				   uint8_t cmd, const uint8_t *args, size_t args_len,
				   uint8_t *resp_data, size_t *resp_len, size_t max_resp_len,
				   int retries, int retry_delay_ms)
{
	const struct sen0628_config *cfg = dev->config;
	uint8_t pkt[SEN0628_HEADER_SIZE + 8];  /* Max 8 args */
	uint8_t status, resp_cmd;
	int ret;
	int attempt;

	/* Build packet */
	ret = build_packet(pkt, sizeof(pkt), cmd, args, args_len);
	if (ret < 0) {
		return ret;
	}
	int pkt_len = ret;

	for (attempt = 0; attempt <= retries; attempt++) {
		if (attempt > 0) {
			LOG_DBG("Retry %d for command 0x%02x", attempt, cmd);
			k_sleep(K_MSEC(retry_delay_ms));
		}

		/* Send packet */
		ret = send_packet(&cfg->i2c, pkt, pkt_len);
		if (ret < 0) {
			LOG_ERR("Failed to send command 0x%02x: %d", cmd, ret);
			continue;
		}

		/* Wait for response */
		k_sleep(K_MSEC(CONFIG_SEN0628_READ_DELAY_MS));

		/* Receive response */
		ret = recv_packet(&cfg->i2c, &status, &resp_cmd,
				  resp_data, resp_len, max_resp_len);
		if (ret == -EAGAIN) {
			/* Sensor not ready, retry */
			continue;
		}
		if (ret < 0) {
			continue;
		}

		/* Check status */
		if (status == SEN0628_STATUS_SUCCESS) {
			return 0;
		}

		LOG_WRN("Command 0x%02x returned status 0x%02x", cmd, status);
	}

	LOG_ERR("Command 0x%02x failed after %d attempts", cmd, retries + 1);
	return -EIO;
}

/*
 * Send command (no retry, for fast operations)
 */
static int send_command(const struct device *dev,
			uint8_t cmd, const uint8_t *args, size_t args_len,
			uint8_t *resp_data, size_t *resp_len, size_t max_resp_len)
{
	return send_command_with_retry(dev, cmd, args, args_len,
				       resp_data, resp_len, max_resp_len, 0, 0);
}

/*
 * Internal: Device tree initialization callback
 * Note: Used when CONFIG_SEN0628 is enabled with device tree instantiation
 */
static int __unused sen0628_dt_init(const struct device *dev)
{
	const struct sen0628_config *cfg = dev->config;
	struct sen0628_data *data = dev->data;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Initialize mutex */
	k_mutex_init(&data->lock);

	/* Initialize data structure */
	data->mode = cfg->initial_mode;
	data->initialized = false;
	data->error_count = 0;

	LOG_INF("SEN0628 driver loaded for 0x%02x", cfg->i2c.addr);

	return 0;  /* Don't probe hardware at boot - let application call init */
}

/*
 * API: Initialize sensor (call from application)
 */
int sen0628_init(const struct device *dev)
{
	const struct sen0628_config *cfg = dev->config;
	struct sen0628_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->initialized) {
		k_mutex_unlock(&data->lock);
		return 0;  /* Already initialized */
	}

	/* Probe device with empty write */
	uint8_t dummy = 0;
	ret = i2c_write_dt(&cfg->i2c, &dummy, 0);
	if (ret < 0) {
		LOG_ERR("Device not found at 0x%02x", cfg->i2c.addr);
		k_mutex_unlock(&data->lock);
		return -ENODEV;
	}

	k_sleep(K_MSEC(SEN0628_INIT_DELAY_MS));

	/* Mark as initialized */
	data->initialized = true;

	LOG_INF("SEN0628 initialized at 0x%02x, mode=%dx%d",
		cfg->i2c.addr, data->mode, data->mode);

	k_mutex_unlock(&data->lock);
	return 0;
}

/*
 * API: Set matrix mode
 */
int sen0628_set_mode(const struct device *dev, uint8_t mode)
{
	struct sen0628_data *data = dev->data;
	uint8_t args[1] = { mode };
	uint8_t resp[4];
	size_t resp_len;
	int ret;

	if (mode != SEN0628_MODE_4X4 && mode != SEN0628_MODE_8X8) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Skip if already in requested mode */
	if (data->mode == mode) {
		LOG_INF("Already in %dx%d mode", mode, mode);
		k_mutex_unlock(&data->lock);
		return 0;
	}

	LOG_INF("Setting mode to %dx%d...", mode, mode);

	/* Mode change needs retry - sensor may not be ready */
	ret = send_command_with_retry(dev, SEN0628_CMD_SETMODE, args, 1,
				      resp, &resp_len, sizeof(resp),
				      3, 100);  /* 3 retries, 100ms between */
	if (ret < 0) {
		LOG_ERR("Failed to set mode to %dx%d: %d", mode, mode, ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	data->mode = mode;

	/* Critical: Wait for sensor to stabilize */
	LOG_INF("Waiting %d ms for mode change...", CONFIG_SEN0628_MODE_SETTLE_MS);
	k_sleep(K_MSEC(CONFIG_SEN0628_MODE_SETTLE_MS));

	k_mutex_unlock(&data->lock);
	return 0;
}

/*
 * API: Get current mode
 */
int sen0628_get_mode(const struct device *dev)
{
	struct sen0628_data *data = dev->data;
	return data->mode;
}

/*
 * API: Read single point
 */
int sen0628_read_point(const struct device *dev,
		       uint8_t x, uint8_t y,
		       uint16_t *distance)
{
	struct sen0628_data *data = dev->data;
	uint8_t args[2] = { x, y };
	uint8_t resp[4];
	size_t resp_len;
	int ret;

	if (x >= data->mode || y >= data->mode) {
		return -EINVAL;
	}

	ret = send_command(dev, SEN0628_CMD_FIXED_POINT, args, 2,
			   resp, &resp_len, sizeof(resp));
	if (ret < 0) {
		return ret;
	}

	if (resp_len < 2) {
		LOG_ERR("Invalid response length: %zu", resp_len);
		return -EIO;
	}

	/* Distance is 16-bit little-endian */
	*distance = resp[0] | (resp[1] << 8);

	return 0;
}

/*
 * API: Read entire matrix
 *
 * Returns: 0 on success (all points read), negative error count on partial failure
 */
int sen0628_read_matrix(const struct device *dev, uint16_t *distances)
{
	struct sen0628_data *data = dev->data;
	uint16_t dist;
	int ret;
	int errors = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Read point by point (CMD_ALLDATA is unreliable) */
	for (uint8_t y = 0; y < data->mode; y++) {
		for (uint8_t x = 0; x < data->mode; x++) {
			ret = sen0628_read_point(dev, x, y, &dist);
			if (ret < 0) {
				dist = SEN0628_DIST_INVALID;
				errors++;
			}
			distances[y * data->mode + x] = dist;
		}
	}

	data->error_count = errors;

	k_mutex_unlock(&data->lock);

	/* Return negative error count if any errors, else 0 */
	return errors > 0 ? -errors : 0;
}

/*
 * API: Read scan with full metadata
 *
 * Returns: 0 on success, negative error count if some points failed
 */
int sen0628_read_scan(const struct device *dev, struct sen0628_scan *scan)
{
	struct sen0628_data *data = dev->data;
	uint32_t start_time;
	uint16_t dist;
	int ret;
	int errors = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	memset(scan, 0, sizeof(*scan));
	scan->mode = data->mode;
	scan->min_distance = SEN0628_DIST_INVALID;
	start_time = k_uptime_get_32();
	scan->timestamp_ms = start_time;

	/* Read point by point */
	for (uint8_t y = 0; y < data->mode; y++) {
		for (uint8_t x = 0; x < data->mode; x++) {
			ret = sen0628_read_point(dev, x, y, &dist);
			if (ret < 0) {
				dist = SEN0628_DIST_INVALID;
				errors++;
			}

			uint8_t idx = y * data->mode + x;
			scan->distances[idx] = dist;

			/* Update statistics */
			if (dist < SEN0628_DIST_INVALID && dist >= SEN0628_DIST_MIN) {
				scan->valid_count++;
				if (dist < scan->min_distance) {
					scan->min_distance = dist;
					scan->min_x = x;
					scan->min_y = y;
				}
			}
		}
	}

	scan->duration_ms = k_uptime_get_32() - start_time;

	/* Cache for later use */
	memcpy(&data->last_scan, scan, sizeof(*scan));
	data->error_count = errors;

	LOG_INF("Scan complete: %d valid points, min=%umm at (%d,%d), took %ums%s",
		scan->valid_count, scan->min_distance,
		scan->min_x, scan->min_y, scan->duration_ms,
		errors > 0 ? " (with errors)" : "");

	k_mutex_unlock(&data->lock);

	return errors > 0 ? -errors : 0;
}

/*
 * API: Get column statistics
 */
int sen0628_get_columns(const struct device *dev, struct sen0628_columns *cols)
{
	struct sen0628_data *data = dev->data;
	struct sen0628_scan *scan;
	uint32_t sum[8] = {0};

	k_mutex_lock(&data->lock, K_FOREVER);

	scan = &data->last_scan;
	memset(cols, 0, sizeof(*cols));

	/* Initialize minimums to invalid */
	for (int i = 0; i < 8; i++) {
		cols->min[i] = SEN0628_DIST_INVALID;
	}

	/* Calculate per-column statistics */
	for (uint8_t y = 0; y < scan->mode; y++) {
		for (uint8_t x = 0; x < scan->mode; x++) {
			uint16_t dist = scan->distances[y * scan->mode + x];

			if (dist < SEN0628_DIST_INVALID && dist >= SEN0628_DIST_MIN) {
				if (dist < cols->min[x]) {
					cols->min[x] = dist;
				}
				sum[x] += dist;
				cols->valid_count[x]++;
			}
		}
	}

	/* Calculate averages */
	for (uint8_t x = 0; x < scan->mode; x++) {
		if (cols->valid_count[x] > 0) {
			cols->avg[x] = sum[x] / cols->valid_count[x];
		} else {
			cols->avg[x] = SEN0628_DIST_INVALID;
		}
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

/*
 * API: Read column minimums (fast obstacle check)
 */
int sen0628_read_columns_min(const struct device *dev, uint16_t *min_per_col)
{
	struct sen0628_scan scan;
	struct sen0628_columns cols;
	int ret;

	ret = sen0628_read_scan(dev, &scan);
	/* Continue even with errors - we have partial data */

	ret = sen0628_get_columns(dev, &cols);
	if (ret < 0) {
		return ret;
	}

	memcpy(min_per_col, cols.min, scan.mode * sizeof(uint16_t));

	return 0;
}

/*
 * API: Get minimum distance (from cached scan)
 */
uint16_t sen0628_get_min_distance(const struct device *dev)
{
	struct sen0628_data *data = dev->data;
	uint16_t min;

	k_mutex_lock(&data->lock, K_FOREVER);
	min = data->last_scan.min_distance;
	k_mutex_unlock(&data->lock);

	return min;
}

/*
 * API: Check if sector is clear
 */
bool sen0628_sector_clear(const struct device *dev,
			  uint8_t sector,
			  uint16_t threshold)
{
	struct sen0628_data *data = dev->data;
	struct sen0628_scan *scan;
	bool clear = true;

	k_mutex_lock(&data->lock, K_FOREVER);

	scan = &data->last_scan;

	if (sector >= scan->mode) {
		k_mutex_unlock(&data->lock);
		return false;
	}

	/* Check all rows in this column/sector */
	for (uint8_t y = 0; y < scan->mode; y++) {
		uint16_t dist = scan->distances[y * scan->mode + sector];
		if (dist < threshold && dist >= SEN0628_DIST_MIN) {
			clear = false;  /* Obstacle detected */
			break;
		}
	}

	k_mutex_unlock(&data->lock);

	return clear;
}

/*
 * API: Get error count from last scan
 */
int sen0628_get_error_count(const struct device *dev)
{
	struct sen0628_data *data = dev->data;
	return data->error_count;
}

/*
 * Debug: Print scan as ASCII art
 */
void sen0628_print_scan(const struct sen0628_scan *scan)
{
	printk("\n=== LIDAR Scan (%dx%d, %u ms) ===\n",
	       scan->mode, scan->mode, scan->duration_ms);

	for (uint8_t y = 0; y < scan->mode; y++) {
		printk("  ");
		for (uint8_t x = 0; x < scan->mode; x++) {
			uint16_t d = scan->distances[y * scan->mode + x];
			if (d >= SEN0628_DIST_INVALID) {
				printk(" ---- ");
			} else {
				printk(" %4u ", d);
			}
		}
		printk("\n");
	}
	printk("Min: %u mm at (%d, %d)\n",
	       scan->min_distance, scan->min_x, scan->min_y);
}

/*
 * Debug: Print column minimums as bar chart
 */
void sen0628_print_columns(const struct sen0628_columns *cols, uint8_t mode)
{
	printk("\n=== Column Minimums ===\n");

	/* Find max for scaling */
	uint16_t max_dist = 0;
	for (uint8_t x = 0; x < mode; x++) {
		if (cols->min[x] < SEN0628_DIST_INVALID && cols->min[x] > max_dist) {
			max_dist = cols->min[x];
		}
	}

	if (max_dist == 0) max_dist = 1000;

	/* Print bars */
	for (uint8_t x = 0; x < mode; x++) {
		uint16_t d = cols->min[x];
		int bar_len = (d < SEN0628_DIST_INVALID) ?
			      (d * 20 / max_dist) : 0;

		printk("  C%d [%4u mm] ", x, d);
		for (int i = 0; i < bar_len; i++) {
			printk("#");
		}
		printk("\n");
	}
}

/*
 * Device tree instantiation
 */
#define SEN0628_INIT(n)                                              \
	static struct sen0628_data sen0628_data_##n;                 \
                                                                     \
	static const struct sen0628_config sen0628_config_##n = {    \
		.i2c = I2C_DT_SPEC_INST_GET(n),                      \
		.initial_mode = DT_INST_PROP_OR(n, initial_mode,     \
						CONFIG_SEN0628_DEFAULT_MODE), \
	};                                                           \
                                                                     \
	DEVICE_DT_INST_DEFINE(n,                                     \
			      sen0628_dt_init,                        \
			      NULL,                                  \
			      &sen0628_data_##n,                      \
			      &sen0628_config_##n,                    \
			      POST_KERNEL,                           \
			      CONFIG_SEN0628_INIT_PRIORITY,          \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(SEN0628_INIT)
