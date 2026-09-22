#pragma once


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_rom_crc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "imu.h"


#define IMU_LOG_PARTITION_LABEL   "imu_log"
#define IMU_SAMPLE_PERIOD_US      10000   /* 10 ms -> 100 Hz */
#define FLASH_SECTOR_SIZE   4096u
#define SECTOR_MAGIC        0x494D5546u   /* "IMUF" */

extern const char *TAG;

typedef struct __attribute__((packed)) {
    uint32_t magic;         /* SECTOR_MAGIC when this sector holds valid data */
    uint32_t seq;           /* monotonically increasing write sequence number */
    uint16_t sample_count;  /* number of valid imu_sample_t entries that follow */
    uint16_t reserved;
    uint32_t crc32;         /* CRC32 over the first sample_count samples       */
} sector_header_t;

_Static_assert(sizeof(sector_header_t) == 16, "header must be 16 bytes");

#define SAMPLES_PER_SECTOR ((FLASH_SECTOR_SIZE - sizeof(sector_header_t)) / sizeof(imu_samples_t))

/* A log_sector_t is exactly one flash sector. Sampling writes straight
 * into buf.samples[]; at flush time we finish filling buf.header and
 * push the *entire* 4096-byte struct to flash in a single
 * esp_partition_write() call -- this is the "fastest api" write path:
 * one erase_range() + one write() per sector, no partial writes, no
 * filesystem bookkeeping layered on top. */
typedef struct __attribute__((packed)) {
    sector_header_t             header;                         // 16 byte header
    imu_samples_t               samples[SAMPLES_PER_SECTOR];    // 21 bytes IMU flash
    uint32_t                    padd;                           // 4 byte padding
    uint16_t                    reserved;                       // 2 bytes reserved
} log_sector_t;

_Static_assert(sizeof(log_sector_t) == FLASH_SECTOR_SIZE,  "log_sector_t must be exactly one flash sector");


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




static const esp_partition_t *s_partition;
static uint32_t s_total_sectors;

/* Double buffer: while one is being filled by the timer callback, the
 * other is either idle (already flushed) or being written by the
 * writer task. Exactly one of {s_buf[0], s_buf[1]} is "active" at a
 * time; the other is either empty or in flight to flash. */
static log_sector_t s_buf[2];
static volatile uint8_t s_active = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t   s_flush_q;      /* holds indices (0/1) of full buffers */
static TaskHandle_t    s_writer_task;


static uint32_t s_next_sector;
static uint32_t s_seq;
static imu_log_stats_t s_stats;



/* One-time setup: finds the partition, scans it for a resume point,
 * creates the writer task + queue. Does NOT start sampling yet. */
esp_err_t flashlog_init(void);

/* Starts the 10 ms esp_timer that feeds the logger. */
esp_err_t flash_log_start(void);

/* Stops the timer (does not flush a partially-filled buffer; call
 * imu_flash_log_flush_partial() first if you need that on shutdown). */
esp_err_t flash_log_stop(void);

/* Force whatever is currently buffered out to flash immediately, even
 * if the sector isn't full (pads the rest of the sector with the
 * partial count recorded in the header -- unused sample slots are
 * simply not read back). Useful before an orderly power-down. */
esp_err_t imu_flash_log_flush_partial(void);

void imu_flash_log_get_stats(imu_log_stats_t *out);

/* Read back one raw 4096-byte sector (header + samples) for offline
 * extraction / a host-side decode tool / unit tests. */
esp_err_t imu_flash_log_read_sector_raw(uint32_t sector_index, void *out_buf_4096_bytes);


