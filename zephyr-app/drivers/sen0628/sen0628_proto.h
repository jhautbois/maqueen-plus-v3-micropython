/*
 * SEN0628 Protocol Definitions
 *
 * Communication protocol for DFRobot SEN0628 8x8 ToF Matrix Sensor.
 * The sensor has an RP2040 that handles VL53L7CX communication and
 * exposes a simplified packet-based I2C interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SEN0628_PROTO_H_
#define SEN0628_PROTO_H_

/*
 * I2C Addresses
 * The sensor supports 4 addresses for cascading multiple sensors.
 */
#define SEN0628_ADDR_0          0x30
#define SEN0628_ADDR_1          0x31
#define SEN0628_ADDR_2          0x32
#define SEN0628_ADDR_3          0x33    /* Default */
#define SEN0628_DEFAULT_ADDR    SEN0628_ADDR_3

/*
 * Command Packet Format (Host -> Sensor)
 *
 * ┌──────┬───────────┬───────────┬─────┬────────────┐
 * │ Head │ ArgsNumH  │ ArgsNumL  │ Cmd │ Args[...]  │
 * │ 0x55 │  (MSB)    │  (LSB)    │     │            │
 * └──────┴───────────┴───────────┴─────┴────────────┘
 *
 * - Head: Always 0x55
 * - ArgsNum: 16-bit big-endian argument count
 * - Cmd: Command ID
 * - Args: Variable length arguments
 */
#define SEN0628_PACKET_HEAD     0x55
#define SEN0628_HEADER_SIZE     4       /* Head + ArgsNumH + ArgsNumL + Cmd */

/*
 * Response Packet Format (Sensor -> Host)
 *
 * ┌────────┬─────┬────────┬────────┬─────────────┐
 * │ Status │ Cmd │ LenL   │ LenH   │ Data[...]   │
 * │        │     │ (LSB)  │ (MSB)  │             │
 * └────────┴─────┴────────┴────────┴─────────────┘
 *
 * - Status: 0x53 (success) or 0x63 (failure)
 * - Cmd: Echo of the command
 * - Len: 16-bit little-endian data length
 * - Data: Variable length response data
 */
#define SEN0628_RESP_HEADER_SIZE 4

/* Status codes */
#define SEN0628_STATUS_SUCCESS  0x53    /* 'S' */
#define SEN0628_STATUS_FAILURE  0x63    /* 'c' */

/*
 * Commands
 */
#define SEN0628_CMD_SETMODE     0x01    /* Set matrix mode (4x4 or 8x8) */
#define SEN0628_CMD_ALLDATA     0x02    /* Read all matrix data (bulk) */
#define SEN0628_CMD_FIXED_POINT 0x03    /* Read single point */

/*
 * Matrix Modes
 */
#define SEN0628_MODE_4X4        4       /* 16 zones, faster */
#define SEN0628_MODE_8X8        8       /* 64 zones, higher resolution */

/*
 * Timing Constants (milliseconds)
 */
#define SEN0628_MODE_SETTLE_MS  5000    /* Wait after mode change */
#define SEN0628_READ_DELAY_MS   30      /* Wait after point read command */
#define SEN0628_CHUNK_DELAY_MS  5       /* Delay between I2C chunks */
#define SEN0628_RECV_TIMEOUT_MS 8000    /* Maximum wait for response */
#define SEN0628_POLL_INTERVAL_MS 17     /* Retry poll interval */
#define SEN0628_INIT_DELAY_MS   50      /* Initial delay after probe */

/*
 * I2C Constraints
 */
#define SEN0628_I2C_MAX_CHUNK   32      /* Maximum I2C transfer size */

/*
 * Distance Constants (millimeters)
 */
#define SEN0628_DIST_MIN        20      /* Minimum reliable distance */
#define SEN0628_DIST_MAX        4000    /* Maximum distance */
#define SEN0628_DIST_INVALID    4000    /* Invalid/no-target value */

/*
 * Matrix Dimensions
 */
#define SEN0628_MAX_COLS        8
#define SEN0628_MAX_ROWS        8
#define SEN0628_MAX_POINTS      64      /* 8x8 */

/*
 * Field of View
 */
#define SEN0628_FOV_HORIZONTAL  60      /* Degrees */
#define SEN0628_FOV_VERTICAL    60      /* Degrees */
#define SEN0628_FOV_DIAGONAL    90      /* Degrees */

/* Angle per zone in 8x8 mode: 60/8 = 7.5 degrees */
#define SEN0628_ZONE_ANGLE_8X8  750     /* 7.50 degrees * 100 */
/* Angle per zone in 4x4 mode: 60/4 = 15 degrees */
#define SEN0628_ZONE_ANGLE_4X4  1500    /* 15.00 degrees * 100 */

#endif /* SEN0628_PROTO_H_ */
