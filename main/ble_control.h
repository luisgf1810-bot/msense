#pragma once
/*
 * ble_control.h
 *
 * Minimal NimBLE GATT peripheral that lets a phone (or any BLE central)
 * start/stop the IMU flash logger and read its current on/off state.
 *
 * GATT layout
 * -----------
 * Service            59fa0001-0d60-41ed-9c1b-cf1c17e1a708
 *   Command char      59fa0002-0d60-41ed-9c1b-cf1c17e1a708   WRITE / WRITE-NO-RSP
 *       write 0x01 -> start logging
 *       write 0x00 -> stop logging
 *   Status char       59fa0003-0d60-41ed-9c1b-cf1c17e1a708   READ / NOTIFY
 *       1 byte: 0x00 = stopped, 0x01 = running
 *       (notified automatically whenever the state changes)
 *
 * Advertises as "ESP32C6-IMULOG". No pairing/bonding is configured --
 * see the README for the security caveat before using this beyond a
 * bench test.
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
/* Includes */

/* NimBLE stack APIs */
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

/* NimBLE GAP APIs */
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"

/* NimBLE GATT APIs */
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

/* Defines GAP*/
#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_URI_PREFIX_HTTPS 0x17
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00

#define DEVICE_NAME "ESP32C6-IMULOG"


static const char *TAG = "ble_control";

/* ---- command byte values written to the Command characteristic ------- */
typedef enum {
    IMU_BLE_CMD_STOP  = 0x00,
    IMU_BLE_CMD_START = 0x01,
} imu_ble_cmd_t;

/* ---- 128-bit UUIDs (see ble_control.h for the human-readable form) --- */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x08, 0xa7, 0xe1, 0x17, 0x1c, 0xcf, 0x1b, 0x9c,  0xed, 0x41, 0x60, 0x0d, 0x01, 0x00, 0xfa, 0x59);

static const ble_uuid128_t cmd_uuid =
    BLE_UUID128_INIT(0x08, 0xa7, 0xe1, 0x17, 0x1c, 0xcf, 0x1b, 0x9c,  0xed, 0x41, 0x60, 0x0d, 0x02, 0x00, 0xfa, 0x59);

static const ble_uuid128_t status_uuid =
    BLE_UUID128_INIT(0x08, 0xa7, 0xe1, 0x17, 0x1c, 0xcf, 0x1b, 0x9c,  0xed, 0x41, 0x60, 0x0d, 0x03, 0x00, 0xfa, 0x59);

static uint8_t  own_addr_type;
static uint8_t  addr_val[6] = {0};
static uint16_t s_conn_handle   = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_status_handle;
static bool     s_logging_active = false;

esp_err_t ble_control_init(void);
static int gap_event_handler(struct ble_gap_event *event, void *arg); 

/* Library function declarations */
void ble_store_config_init(void);


#ifdef __cplusplus
}
#endif