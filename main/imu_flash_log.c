#include "imu_flash_log.h"

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

static const char *TAG = "imu_flash_log";

/* -------------------------------------------------------------------- */
/* On-flash layout                                                       */
/* -------------------------------------------------------------------- */

#define FLASH_SECTOR_SIZE   4096u
#define SECTOR_MAGIC        0x494D5546u   /* "IMUF" */

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

/* -------------------------------------------------------------------- */
/* State                                                                 */
/* -------------------------------------------------------------------- */

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



/* -------------------------------------------------------------------- */
/* Sequence-number compare that tolerates uint32 wraparound              */
/* -------------------------------------------------------------------- */

static inline bool seq_is_newer(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}




/* -------------------------------------------------------------------- */
/* Writer task: the only task that ever touches the flash partition      */
/* -------------------------------------------------------------------- */

static void write_sector_to_flash(log_sector_t *sec)
{
    const uint8_t *payload = (const uint8_t *)sec + sizeof(sector_header_t);

    sec->header.magic = SECTOR_MAGIC;
    sec->header.seq   = s_seq++;
    sec->header.crc32 = esp_rom_crc32_le(0, payload, (uint32_t)sec->header.sample_count * sizeof(imu_samples_t));

    size_t offset = (size_t)s_next_sector * FLASH_SECTOR_SIZE;

    int64_t t0 = esp_timer_get_time();

    /* Flash can only clear bits via erase; every sector must be erased
     * before it is reused (this is a ring, so after the first lap every
     * sector already holds old data). */
    esp_err_t err = esp_partition_erase_range(s_partition, offset, FLASH_SECTOR_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "erase failed @ sector %" PRIu32 ": %s", s_next_sector, esp_err_to_name(err));
        s_stats.sectors_erase_failed++;
        goto advance;
    }

    /* Single write call for the whole sector (header + payload together)
     * -- this is the fastest available IDF path for raw partition I/O:
     * esp_partition_write() maps directly onto the underlying
     * spi_flash_write(), with no filesystem indirection. */
    err = esp_partition_write(s_partition, offset, sec, FLASH_SECTOR_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write failed @ sector %" PRIu32 ": %s", s_next_sector, esp_err_to_name(err));
        s_stats.sectors_write_failed++;
        goto advance;
    }

    s_stats.sectors_written++;
    ESP_LOGI(TAG, "sector %" PRIu32 " (seq %" PRIu32 ", %u samples) written in %lld us",
              s_next_sector, sec->header.seq, sec->header.sample_count,
              (long long)(esp_timer_get_time() - t0));

advance:
    s_next_sector++;
    if (s_next_sector >= s_total_sectors) {
        s_next_sector = 0;
        s_stats.wrap_count++;
    }
    s_stats.next_sector = s_next_sector;
    s_stats.next_seq     = s_seq;
}

static void writer_task_fn(void *arg)
{
    uint8_t idx;
    for (;;) {
        if (xQueueReceive(s_flush_q, &idx, portMAX_DELAY) == pdTRUE) {
            write_sector_to_flash(&s_buf[idx]);
            /* Buffer is now free for the sampler to reuse. */
            taskENTER_CRITICAL(&s_mux);
            s_buf[idx].header.sample_count = 0;
            taskEXIT_CRITICAL(&s_mux);
        }
    }
}





/* -------------------------------------------------------------------- */
/* Public API                                                            */
/* -------------------------------------------------------------------- */

esp_err_t flash_log_start(void)
{
    s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, IMU_LOG_PARTITION_LABEL);
    if (!s_partition) {
        ESP_LOGE(TAG, "partition '%s' not found -- check partitions.csv", IMU_LOG_PARTITION_LABEL);
        return ESP_ERR_NOT_FOUND;
    }
    if (s_partition->size % FLASH_SECTOR_SIZE != 0) {
        ESP_LOGE(TAG, "partition size must be a multiple of %u bytes", FLASH_SECTOR_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    s_total_sectors = (uint32_t)(s_partition->size / FLASH_SECTOR_SIZE);
    memset(s_buf, 0, sizeof(s_buf));
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.total_sectors = s_total_sectors;

    ESP_LOGI(TAG, "partition '%s': %u bytes, %" PRIu32 " sectors, %u samples/sector",
              s_partition->label, (unsigned)s_partition->size, s_total_sectors,
              (unsigned)SAMPLES_PER_SECTOR);

    s_next_sector = 0;
    s_seq = 0;
    s_stats.next_sector = s_next_sector;
    s_stats.next_seq    = s_seq;

    s_flush_q = xQueueCreate(2, sizeof(uint8_t));
    if (!s_flush_q) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreate(writer_task_fn, "imu_flash_writer",  4096, NULL, tskIDLE_PRIORITY + 3, &s_writer_task);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}


esp_err_t flash_log_stop(void) {

    //vTaskDelete(&s_writer_task);
    ESP_LOGI(TAG, "flashlog stoped");
    return ESP_OK;
}


esp_err_t imu_flash_log_flush_partial(void)
{
    uint8_t idx;
    uint16_t count;

    taskENTER_CRITICAL(&s_mux);
    idx = s_active;
    count = s_buf[idx].header.sample_count;
    if (count > 0) {
        uint8_t other = 1 - idx;
        s_active = other; /* stop new samples from landing in idx */
    }
    taskEXIT_CRITICAL(&s_mux);

    if (count == 0) {
        return ESP_OK; /* nothing pending */
    }
    xQueueSend(s_flush_q, &idx, portMAX_DELAY);
    return ESP_OK;
}

void imu_flash_log_get_stats(imu_log_stats_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL(&s_mux);
    *out = s_stats;
    taskEXIT_CRITICAL(&s_mux);
}

esp_err_t imu_flash_log_read_sector_raw(uint32_t sector_index, void *out_buf_4096_bytes)
{
    if (sector_index >= s_total_sectors) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_partition_read(s_partition, (size_t)sector_index * FLASH_SECTOR_SIZE,  out_buf_4096_bytes, FLASH_SECTOR_SIZE);
}
