#include "ble_control.h"
#include "imu_flash_log.h"
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"

/* -------------------------------------------------------------------- */
/* Utils                                                                 */
/* -------------------------------------------------------------------- */

inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}


/* -------------------------------------------------------------------- */
/* Logger control wrappers (keep s_logging_active authoritative)         */
/* -------------------------------------------------------------------- */

static void notify_status(void);

static void do_start(void)
{
    if (!s_logging_active) {
        if (imu_flash_log_start() == ESP_OK) {
            s_logging_active = true;
            ESP_LOGI(TAG, "logging started (BLE command)");
        }
    }
    notify_status();
}

static void do_stop(void)
{
    if (s_logging_active) {
        imu_flash_log_stop();
        s_logging_active = false;
        ESP_LOGI(TAG, "logging stopped (BLE command)");
    }
    notify_status();
}

/* -------------------------------------------------------------------- */
/* GATT access callbacks                                                 */
/* -------------------------------------------------------------------- */

static int gatt_access_cmd(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t data[4];
    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, data, sizeof(data), &out_len);
    if (rc != 0 || out_len < 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    switch (data[0]) {
        case IMU_BLE_CMD_START:
            do_start();
            break;
        case IMU_BLE_CMD_STOP:
            do_stop();
            break;
        default:
            ESP_LOGW(TAG, "unknown command byte 0x%02x", data[0]);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    return 0;
}

static int gatt_access_status(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    uint8_t status_byte = s_logging_active ? 0x01 : 0x00;
    int rc = os_mbuf_append(ctxt->om, &status_byte, sizeof(status_byte));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static void notify_status(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return; /* nobody connected/subscribed */
    }
    uint8_t status_byte = s_logging_active ? 0x01 : 0x00;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&status_byte, sizeof(status_byte));
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_status_handle, om);
    }
}

/* -------------------------------------------------------------------- */
/* GATT service table                                                    */
/* -------------------------------------------------------------------- */

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &cmd_uuid.u,
                .access_cb = gatt_access_cmd,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &status_uuid.u,
                .access_cb = gatt_access_status,
                .val_handle = &s_status_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }, /* terminator */
        },
    },
    { 0 }, /* terminator */
};

/* -------------------------------------------------------------------- */
/* Advertising / GAP                                                     */
/* -------------------------------------------------------------------- */

static void start_advertising(void)
{
    /* Local variables */
    int rc = 0;
    char addr_str[18] = {0};
    const char *name;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    /* Make sure we have proper BT identity address set (random preferred) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Printing ADDR */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "device address: %s", addr_str);




    /* Set advertising flags */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Set device name */
    name = ble_svc_gap_device_name();
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    /* Set device tx power */
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1;

    /* Set device appearance */
    adv_fields.appearance = BLE_GAP_APPEARANCE_GENERIC_TAG;
    adv_fields.appearance_is_present = 1;

    /* Set device LE role */
    adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    adv_fields.le_role_is_present = 1;

    /* Set advertisement fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    /* Set device address */
    rsp_fields.device_addr = addr_val;
    rsp_fields.device_addr_type = own_addr_type;
    rsp_fields.device_addr_is_present = 1;

    /* Set advertising interval */
    rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
    rsp_fields.adv_itvl_is_present = 1;

    /* Set scan response fields */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set scan response data, error code: %d", rc);
        return;
    }

    /* Set undirected connectable and general discoverable mode */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Set advertising interval */
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    /* Start advertising */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,  gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising started!");
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "connect %s", event->connect.status == 0 ? "established" : "failed");
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
            } else {
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnected, reason=%d", event->disconnect.reason);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_advertising();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            start_advertising();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "subscribe: attr=%d notify=%d",
                      event->subscribe.attr_handle, event->subscribe.cur_notify);
            break;

        default:
            break;
    }
    return 0;
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE stack synced, advertising as \"%s\"", DEVICE_NAME);
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
}

static void host_task(void *param)
{
    nimble_port_run(); /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;

    /* Store host configuration */
    ble_store_config_init();
}



/* -------------------------------------------------------------------- */
/* Public entry point                                                    */
/* -------------------------------------------------------------------- */

esp_err_t ble_control_init(void)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init(); 
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Initialize the NimBLE stack
    ESP_ERROR_CHECK(nimble_port_init());

    // Init gap and gatt services

    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Update and add GATT services counter
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }

    // Set GAP device name
    ble_svc_gap_device_name_set(DEVICE_NAME);
/* Library function declarations */
void ble_store_config_init(void)            ;
    /* Set host callbacks */
    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
   

    /* Store host configuration */
    nimble_host_config_init();


    // Nimble freertos stack initialization
    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "BLE control ready");
    return ESP_OK;
}