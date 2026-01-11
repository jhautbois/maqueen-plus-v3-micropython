/*
 * SEN0628 LIDAR Benchmark Application
 *
 * Measures real hardware performance:
 *   1. I2C communication test
 *   2. Single point read timing
 *   3. Full 4x4 matrix read timing (16 points)
 *   4. Full 8x8 matrix read timing (64 points)
 *   5. Column statistics calculation time
 *   6. Bulk read test (CMD_ALLDATA)
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

LOG_MODULE_REGISTER(lidar_bench, LOG_LEVEL_INF);

/* Device handles */
#define I2C_EXT_NODE DT_NODELABEL(i2c1)
#define LIDAR_NODE   DT_NODELABEL(lidar)

static const struct device *i2c_ext = DEVICE_DT_GET(I2C_EXT_NODE);
static const struct device *lidar = DEVICE_DT_GET(LIDAR_NODE);

/* Test parameters */
#define NUM_ITERATIONS    5   /* Number of timing iterations */
#define WARMUP_READS      2   /* Warmup reads before timing */

/*
 * Benchmark results structure
 */
struct bench_result {
	const char *name;
	uint32_t min_ms;
	uint32_t max_ms;
	uint32_t avg_ms;
	int iterations;
	int success;
};

static struct bench_result results[10];
static int num_results = 0;

/*
 * Add benchmark result
 */
static void add_result(const char *name, uint32_t *times, int count, int success)
{
	if (num_results >= ARRAY_SIZE(results)) {
		return;
	}

	struct bench_result *r = &results[num_results++];
	r->name = name;
	r->iterations = count;
	r->success = success;

	if (count == 0 || !success) {
		r->min_ms = r->max_ms = r->avg_ms = 0;
		return;
	}

	uint32_t sum = 0;
	r->min_ms = UINT32_MAX;
	r->max_ms = 0;

	for (int i = 0; i < count; i++) {
		sum += times[i];
		if (times[i] < r->min_ms) r->min_ms = times[i];
		if (times[i] > r->max_ms) r->max_ms = times[i];
	}
	r->avg_ms = sum / count;
}

/*
 * Print all results
 */
static void print_results(void)
{
	printk("\n");
	printk("╔══════════════════════════════════════════════════════════════╗\n");
	printk("║              SEN0628 LIDAR BENCHMARK RESULTS                 ║\n");
	printk("╠══════════════════════════════════════════════════════════════╣\n");
	printk("║ Test                           │ Min   │ Avg   │ Max   │ OK  ║\n");
	printk("╠────────────────────────────────┼───────┼───────┼───────┼─────╣\n");

	for (int i = 0; i < num_results; i++) {
		struct bench_result *r = &results[i];
		printk("║ %-30s │ %5u │ %5u │ %5u │ %s ║\n",
		       r->name,
		       r->min_ms, r->avg_ms, r->max_ms,
		       r->success ? "YES" : "NO ");
	}

	printk("╚══════════════════════════════════════════════════════════════╝\n");
	printk("\n");

	/* Performance summary */
	printk("Performance Summary:\n");
	for (int i = 0; i < num_results; i++) {
		struct bench_result *r = &results[i];
		if (r->success && r->avg_ms > 0) {
			uint32_t hz = 1000 / r->avg_ms;
			printk("  %s: ~%u Hz refresh rate\n", r->name, hz);
		}
	}
}

/*
 * I2C bus scan for debugging
 */
static int bench_i2c_scan(void)
{
	uint8_t dummy = 0;
	int found = 0;

	LOG_INF("=== I2C Bus Scan ===");

	for (uint8_t addr = 0x08; addr < 0x78; addr++) {
		struct i2c_msg msg = {
			.buf = &dummy,
			.len = 0,
			.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
		};
		int ret = i2c_transfer(i2c_ext, &msg, 1, addr);
		if (ret == 0) {
			LOG_INF("  Found device at 0x%02x", addr);
			found++;
		}
	}

	LOG_INF("  Total: %d devices", found);

	/* Check for LIDAR specifically */
	struct i2c_msg msg = {
		.buf = &dummy,
		.len = 0,
		.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
	};
	int ret = i2c_transfer(i2c_ext, &msg, 1, 0x33);

	return (ret == 0) ? 0 : -ENODEV;
}

/*
 * Benchmark: Single point read
 */
static void bench_single_point(void)
{
	uint32_t times[NUM_ITERATIONS];
	uint16_t distance;
	int ret;
	int success = 1;

	LOG_INF("=== Benchmark: Single Point Read ===");

	/* Warmup */
	for (int i = 0; i < WARMUP_READS; i++) {
		sen0628_read_point(lidar, 4, 4, &distance);
	}

	/* Timed reads */
	for (int i = 0; i < NUM_ITERATIONS; i++) {
		uint32_t start = k_uptime_get_32();
		ret = sen0628_read_point(lidar, 4, 4, &distance);
		times[i] = k_uptime_get_32() - start;

		if (ret < 0) {
			LOG_ERR("  Read %d failed: %d", i, ret);
			success = 0;
		} else {
			LOG_INF("  Read %d: %u mm in %u ms", i, distance, times[i]);
		}
	}

	add_result("Single point read", times, NUM_ITERATIONS, success);
}

/*
 * Benchmark: Full 4x4 matrix read
 */
static void bench_4x4_matrix(void)
{
	uint32_t times[NUM_ITERATIONS];
	struct sen0628_scan scan;
	int ret;
	int success = 1;

	LOG_INF("=== Benchmark: 4x4 Matrix Read ===");

	/* Set 4x4 mode */
	ret = sen0628_set_mode(lidar, SEN0628_MODE_4X4);
	if (ret < 0) {
		LOG_ERR("Failed to set 4x4 mode: %d", ret);
		add_result("4x4 matrix read", times, 0, 0);
		return;
	}

	/* Warmup */
	sen0628_read_scan(lidar, &scan);

	/* Timed reads */
	for (int i = 0; i < NUM_ITERATIONS; i++) {
		uint32_t start = k_uptime_get_32();
		ret = sen0628_read_scan(lidar, &scan);
		times[i] = k_uptime_get_32() - start;

		if (ret < 0) {
			LOG_ERR("  Scan %d failed: %d", i, ret);
			success = 0;
		} else {
			LOG_INF("  Scan %d: %d valid, min=%u mm, %u ms",
				i, scan.valid_count, scan.min_distance, times[i]);
		}
	}

	add_result("4x4 matrix (16 pts)", times, NUM_ITERATIONS, success);

	/* Print last scan */
	sen0628_print_scan(&scan);
}

/*
 * Benchmark: Full 8x8 matrix read
 */
static void bench_8x8_matrix(void)
{
	uint32_t times[NUM_ITERATIONS];
	struct sen0628_scan scan;
	int ret;
	int success = 1;

	LOG_INF("=== Benchmark: 8x8 Matrix Read ===");

	/* Set 8x8 mode */
	ret = sen0628_set_mode(lidar, SEN0628_MODE_8X8);
	if (ret < 0) {
		LOG_ERR("Failed to set 8x8 mode: %d", ret);
		add_result("8x8 matrix read", times, 0, 0);
		return;
	}

	/* Warmup */
	sen0628_read_scan(lidar, &scan);

	/* Timed reads */
	for (int i = 0; i < NUM_ITERATIONS; i++) {
		uint32_t start = k_uptime_get_32();
		ret = sen0628_read_scan(lidar, &scan);
		times[i] = k_uptime_get_32() - start;

		if (ret < 0) {
			LOG_ERR("  Scan %d failed: %d", i, ret);
			success = 0;
		} else {
			LOG_INF("  Scan %d: %d valid, min=%u mm, %u ms",
				i, scan.valid_count, scan.min_distance, times[i]);
		}
	}

	add_result("8x8 matrix (64 pts)", times, NUM_ITERATIONS, success);

	/* Print last scan */
	sen0628_print_scan(&scan);
}

/*
 * Benchmark: Column statistics calculation
 */
static void bench_column_stats(void)
{
	uint32_t times[NUM_ITERATIONS];
	struct sen0628_columns cols;
	int ret;
	int success = 1;

	LOG_INF("=== Benchmark: Column Statistics ===");

	/* Ensure we have fresh scan data */
	struct sen0628_scan scan;
	sen0628_read_scan(lidar, &scan);

	/* Timed calculations */
	for (int i = 0; i < NUM_ITERATIONS; i++) {
		uint32_t start = k_uptime_get_32();
		ret = sen0628_get_columns(lidar, &cols);
		times[i] = k_uptime_get_32() - start;

		if (ret < 0) {
			LOG_ERR("  Calc %d failed: %d", i, ret);
			success = 0;
		} else {
			LOG_INF("  Calc %d: %u ms", i, times[i]);
		}
	}

	add_result("Column stats calc", times, NUM_ITERATIONS, success);

	/* Print column data */
	int mode = sen0628_get_mode(lidar);
	sen0628_print_columns(&cols, mode);
}

/*
 * Benchmark: Combined scan + column (typical navigation use)
 */
static void bench_navigation_cycle(void)
{
	uint32_t times[NUM_ITERATIONS];
	struct sen0628_scan scan;
	struct sen0628_columns cols;
	int ret;
	int success = 1;

	LOG_INF("=== Benchmark: Navigation Cycle (scan + columns) ===");

	/* Timed full cycle */
	for (int i = 0; i < NUM_ITERATIONS; i++) {
		uint32_t start = k_uptime_get_32();

		ret = sen0628_read_scan(lidar, &scan);
		if (ret == 0) {
			ret = sen0628_get_columns(lidar, &cols);
		}

		times[i] = k_uptime_get_32() - start;

		if (ret < 0) {
			LOG_ERR("  Cycle %d failed: %d", i, ret);
			success = 0;
		} else {
			LOG_INF("  Cycle %d: min=%u mm, %u ms",
				i, scan.min_distance, times[i]);
		}
	}

	add_result("Navigation cycle", times, NUM_ITERATIONS, success);
}

/*
 * Test: Bulk read (CMD_ALLDATA) - experimental
 *
 * Note: This has been unreliable in MicroPython. Testing here to see
 * if Zephyr's I2C driver handles it better.
 */
static void bench_bulk_read(void)
{
	/* Direct I2C test of CMD_ALLDATA */
	const struct i2c_dt_spec i2c_spec = I2C_DT_SPEC_GET(LIDAR_NODE);
	uint8_t cmd_pkt[4] = {
		SEN0628_PACKET_HEAD,
		0x00,                   /* ArgsNumH */
		0x00,                   /* ArgsNumL */
		SEN0628_CMD_ALLDATA     /* Command */
	};
	uint8_t resp[256];
	int ret;

	LOG_INF("=== Test: Bulk Read (CMD_ALLDATA) ===");

	/* Send command */
	ret = i2c_write_dt(&i2c_spec, cmd_pkt, sizeof(cmd_pkt));
	if (ret < 0) {
		LOG_ERR("Failed to send CMD_ALLDATA: %d", ret);
		add_result("Bulk read (ALLDATA)", NULL, 0, 0);
		return;
	}

	/* Wait longer for bulk response */
	k_sleep(K_MSEC(500));

	/* Try to read response */
	ret = i2c_read_dt(&i2c_spec, resp, 4);  /* Just header first */
	if (ret < 0) {
		LOG_ERR("Failed to read bulk response: %d", ret);
		add_result("Bulk read (ALLDATA)", NULL, 0, 0);
		return;
	}

	LOG_INF("Bulk response header: %02x %02x %02x %02x",
		resp[0], resp[1], resp[2], resp[3]);

	uint8_t status = resp[0];
	uint16_t data_len = resp[2] | (resp[3] << 8);

	if (status == SEN0628_STATUS_SUCCESS) {
		LOG_INF("Bulk read SUCCESS! Data length: %u bytes", data_len);

		/* Read remaining data */
		if (data_len > 0 && data_len <= sizeof(resp) - 4) {
			k_sleep(K_MSEC(50));
			ret = i2c_read_dt(&i2c_spec, &resp[4], data_len);
			if (ret == 0) {
				LOG_INF("Read %u bytes of bulk data", data_len);

				/* Interpret as distances (little-endian 16-bit) */
				int num_points = data_len / 2;
				LOG_INF("Contains %d distance values", num_points);

				/* Print first few */
				for (int i = 0; i < MIN(8, num_points); i++) {
					uint16_t d = resp[4 + i*2] | (resp[4 + i*2 + 1] << 8);
					LOG_INF("  Point %d: %u mm", i, d);
				}

				uint32_t times[1] = {0};  /* Would measure here */
				add_result("Bulk read (ALLDATA)", times, 1, 1);
				return;
			}
		}
	} else {
		LOG_WRN("Bulk read returned status 0x%02x (expected 0x53)", status);
	}

	add_result("Bulk read (ALLDATA)", NULL, 0, 0);
}

/*
 * Test: Read delay sensitivity
 *
 * Tests different delays between command and response to find
 * the minimum reliable delay.
 */
static void bench_read_delay(void)
{
	const struct i2c_dt_spec i2c_spec = I2C_DT_SPEC_GET(LIDAR_NODE);
	uint8_t cmd_pkt[6] = {
		SEN0628_PACKET_HEAD,
		0x00,
		0x02,                   /* 2 args */
		SEN0628_CMD_FIXED_POINT,
		4,                      /* x = 4 */
		4                       /* y = 4 */
	};
	uint8_t resp[8];
	int delays[] = {10, 15, 20, 25, 30, 40, 50};
	int ret;

	LOG_INF("=== Test: Read Delay Sensitivity ===");

	for (int d = 0; d < ARRAY_SIZE(delays); d++) {
		int delay_ms = delays[d];
		int success_count = 0;

		for (int trial = 0; trial < 5; trial++) {
			/* Send command */
			ret = i2c_write_dt(&i2c_spec, cmd_pkt, sizeof(cmd_pkt));
			if (ret < 0) continue;

			/* Variable delay */
			k_sleep(K_MSEC(delay_ms));

			/* Read response */
			ret = i2c_read_dt(&i2c_spec, resp, sizeof(resp));
			if (ret < 0) continue;

			if (resp[0] == SEN0628_STATUS_SUCCESS) {
				success_count++;
			}
		}

		LOG_INF("  Delay %2d ms: %d/5 success", delay_ms, success_count);
	}
}

/*
 * Main entry point
 */
int main(void)
{
	int ret;

	printk("\n");
	printk("╔══════════════════════════════════════════════════════════════╗\n");
	printk("║           SEN0628 LIDAR BENCHMARK APPLICATION                ║\n");
	printk("║                                                              ║\n");
	printk("║  Measures real hardware performance for SLAM planning        ║\n");
	printk("╚══════════════════════════════════════════════════════════════╝\n");
	printk("\n");

	/* Check I2C bus */
	if (!device_is_ready(i2c_ext)) {
		LOG_ERR("External I2C (i2c1) not ready!");
		return -1;
	}
	LOG_INF("I2C bus ready");

	/* Scan I2C bus */
	ret = bench_i2c_scan();
	if (ret < 0) {
		LOG_ERR("LIDAR not found at 0x33! Check connections.");
		return -1;
	}

	/* Check LIDAR device */
	if (!device_is_ready(lidar)) {
		LOG_ERR("LIDAR device not ready!");
		return -1;
	}

	/* Initialize sensor */
	ret = sen0628_init(lidar);
	if (ret < 0) {
		LOG_ERR("Failed to initialize LIDAR: %d", ret);
		return -1;
	}

	LOG_INF("Starting benchmarks...\n");
	k_sleep(K_SECONDS(1));

	/* Run all benchmarks */
	bench_single_point();
	k_sleep(K_MSEC(500));

	bench_4x4_matrix();
	k_sleep(K_MSEC(500));

	bench_8x8_matrix();
	k_sleep(K_MSEC(500));

	bench_column_stats();
	k_sleep(K_MSEC(500));

	bench_navigation_cycle();
	k_sleep(K_MSEC(500));

	bench_bulk_read();
	k_sleep(K_MSEC(500));

	bench_read_delay();
	k_sleep(K_MSEC(500));

	/* Print summary */
	print_results();

	/* Expected vs measured analysis */
	printk("Expected Timing Analysis:\n");
	printk("  - Single point: ~30ms (CONFIG_SEN0628_READ_DELAY_MS)\n");
	printk("  - 4x4 matrix:   ~480ms (16 * 30ms)\n");
	printk("  - 8x8 matrix:   ~1920ms (64 * 30ms)\n");
	printk("  - If bulk read works: ~100-200ms for full matrix\n");
	printk("\n");

	/* SLAM implications */
	printk("SLAM Implications:\n");
	int mode = sen0628_get_mode(lidar);
	printk("  - Current mode: %dx%d\n", mode, mode);
	printk("  - For SLAM, need ~5-10 Hz update rate\n");
	printk("  - Point-by-point: 8x8 too slow (~0.5 Hz)\n");
	printk("  - Options if bulk fails:\n");
	printk("    1. Use 4x4 mode (~2 Hz)\n");
	printk("    2. Read only center rows (faster scan)\n");
	printk("    3. Reduce I2C delays (needs testing)\n");
	printk("\n");

	LOG_INF("Benchmark complete. Entering idle loop.");

	while (1) {
		k_sleep(K_SECONDS(10));
	}

	return 0;
}
