#pragma once
/*
 * imu_flash_log.h
 *
 * Raw-partition IMU logger for ESP32-C6 / ESP-IDF.
 *
 *   - Samples are produced every 10 ms (100 Hz) by an esp_timer callback.
 *   - Samples land in one of two RAM sector buffers (double buffering).
 *   - When a buffer fills exactly one flash sector's worth of samples,
 *     it is handed to a dedicated writer task which erases + writes the
 *     target flash sector using the raw esp_partition_* API directly
 *     (no FAT/LittleFS/NVS layer in the way).
 *   - The partition is treated as a circular log ("ring") across all of
 *     its sectors. Because every sector in the 4 MB region gets erased
 *     and rewritten in strict round-robin order, wear is spread exactly
 *     evenly across the whole partition -- this *is* the wear-leveling
 *     strategy (no separate translation layer is needed for a pure
 *     sequential log like this).
 *   - Each sector is self-describing (magic + monotonic sequence number
 *     + CRC32), so on boot we scan headers to find where the log left
 *     off and resume there, surviving resets/power loss.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_LOG_PARTITION_LABEL   "imu_log"
#define IMU_SAMPLE_PERIOD_US      10000   /* 10 ms -> 100 Hz */

/* ---- one IMU sample -----------------------------------------------------
 * 8 + 4 + 4 + 4 = 20 bytes, packed so the on-flash layout is exact and
 * portable (no compiler-inserted padding).
 */
typedef struct __attribute__((packed)) {
    uint64_t timestamp_us;   /* esp_timer_get_time() at sample time      */
    float    accel_x;
    float    accel_y;
    float    accel_z;
} imu_samples_t;

_Static_assert(sizeof(imu_samples_t) == 20, "imu_samples_t must be 20 bytes");

typedef struct {
    uint32_t sectors_written;
    uint32_t sectors_erase_failed;
    uint32_t sectors_write_failed;
    uint32_t buffer_overruns;     /* writer couldn't keep up in time     */
    uint32_t next_sector;
    uint32_t next_seq;
    uint32_t total_sectors;
    uint32_t wrap_count;          /* how many times the ring has wrapped */
} imu_log_stats_t;

/* One-time setup: finds the partition, scans it for a resume point,
 * creates the writer task + queue. Does NOT start sampling yet. */
esp_err_t imu_flash_log_init(void);

/* Starts the 10 ms esp_timer that feeds the logger. */
esp_err_t imu_flash_log_start(void);

/* Stops the timer (does not flush a partially-filled buffer; call
 * imu_flash_log_flush_partial() first if you need that on shutdown). */
esp_err_t imu_flash_log_stop(void);

/* Force whatever is currently buffered out to flash immediately, even
 * if the sector isn't full (pads the rest of the sector with the
 * partial count recorded in the header -- unused sample slots are
 * simply not read back). Useful before an orderly power-down. */
esp_err_t imu_flash_log_flush_partial(void);

void imu_flash_log_get_stats(imu_log_stats_t *out);

/* Read back one raw 4096-byte sector (header + samples) for offline
 * extraction / a host-side decode tool / unit tests. */
esp_err_t imu_flash_log_read_sector_raw(uint32_t sector_index,
                                         void *out_buf_4096_bytes);

#ifdef __cplusplus
}
#endif
