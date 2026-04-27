#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_log.h"

//A Tag for BLE transmission such as errors or setup
static const char *TAG = "BLE_OUT";

// UUIDs must match your project
// Below is a more convenient way to view the service and characteristic UUIDs,
// however they need to be defince as 128 bit BLE UUIDs rather than character arrays.
// #define GESTURE_SERVICE_UUID        "c1b47598-8439-47b2-9ef7-69ee7c2349b8"
// #define GESTURE_CHARACTERISTIC_UUID "2316ba1a-066c-49d3-81e0-3f5c2599630e"
// UUID objects (NOT strings)
static const ble_uuid128_t GESTURE_SERVICE_UUID =
    BLE_UUID128_INIT(0xc1,0xb4,0x75,0x98,0x84,0x39,0x47,0xb2,
                     0x9e,0xf7,0x69,0xee,0x7c,0x23,0x49,0xb8);

static const ble_uuid128_t GESTURE_CHARACTERISTIC_UUID =
    BLE_UUID128_INIT(0x23,0x16,0xba,0x1a,0x06,0x6c,0x49,0xd3,
                     0x81,0xe0,0x3f,0x5c,0x25,0x99,0x63,0x0e);


// Global connection handle
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;

//BLE Characteristic handle
static uint16_t gesture_chr_handle;

// A buffer to store the last detected gesture
static char last_gesture_msg[64] = {0};

// Forward declaration
static int gesture_chr_access_cb(uint16_t conn_handle,
                                 uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt,
                                 void *arg);


// --------------------------------------------------
// GATT Characteristic Definition
// --------------------------------------------------                                 
static const struct  ble_gatt_chr_def gesture_characteristics[] = {
            {
                .uuid = &GESTURE_CHARACTERISTIC_UUID.u,
                .access_cb = gesture_chr_access_cb,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = 0,
                .val_handle = &gesture_chr_handle,
                .cpfd = nullptr
                
            },
            {0}
        };

// --------------------------------------------------
// GATT Service Definition
// --------------------------------------------------
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        /*** Gesture Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &GESTURE_SERVICE_UUID.u,
        .characteristics = gesture_characteristics
    },

    { 0 } // end of services
};

// --------------------------------------------------
// A simple characteristic callback handler
// --------------------------------------------------
static int gesture_chr_access_cb(uint16_t conn_handle,
                                 uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt,
                                 void *arg)
{
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGI(TAG, "Client READ gesture characteristic");
            os_mbuf_append(ctxt->om, last_gesture_msg, strlen(last_gesture_msg));
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            ESP_LOGI(TAG, "Client tried to WRITE a gesture characteristic");
            break;
    }
    return 0;
}

// --------------------------------------------------
// Prototype for GAP callback handler for connections and disconnections
// --------------------------------------------------
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

// --------------------------------------------------
// Advertising set up for transmission
// --------------------------------------------------
static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack synced, starting advertising");

    struct ble_gap_adv_params adv_params{};
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    const char *name = "A.R.G.H_v0.5";

    ble_svc_gap_device_name_set(name);

    struct ble_hs_adv_fields fields{};
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields: %d", rc);
    }

    // -----------------------------
    // Scan response packet
    // -----------------------------
    struct ble_hs_adv_fields scan_rsp;
    memset(&scan_rsp, 0, sizeof(scan_rsp));

    // Advertise 128-bit service UUID 
    scan_rsp.uuids128 = (ble_uuid128_t *)&GESTURE_SERVICE_UUID;
    scan_rsp.num_uuids128 = 1;
    scan_rsp.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response: %d", rc);
    }

    // Set a fixed random static BLE address 
    // This is a temporary fix to make it visible for the Web UI demo
    uint8_t addr_val[6] = {0xC0, 0x98, 0xE5, 0x00, 0x00, 0x01};
    ble_hs_id_set_rnd(addr_val);


    //start advertising
     rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_cb, NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising: %d", rc);
    }

}

// --------------------------------------------------
// GAP callback handler for connections and disconnections
// --------------------------------------------------
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                g_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Client connected");
            } else {
                ESP_LOGI(TAG, "Connection failed, restarting advertising");
                ble_on_sync();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Client disconnected");
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_on_sync();
            break;

        case BLE_GAP_EVENT_NOTIFY_TX:
            ESP_LOGI(TAG, "Notification sent");
            break;
    }
    return 0;
}

// --------------------------------------------------
// Log a reset
// --------------------------------------------------
static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset, reason=%d", reason);
}

// --------------------------------------------------
// Public function: update gesture characteristic
// Format: "Gesture: gesture_name        Confidence: 99.6%"
// --------------------------------------------------
void ble_send_gesture(const char *gesture_msg)
{
    strncpy(last_gesture_msg, gesture_msg, sizeof(last_gesture_msg));

    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    uint16_t svc_handle, chr_handle;

    int rc = ble_gatts_find_chr(&GESTURE_SERVICE_UUID.u,
                                &GESTURE_CHARACTERISTIC_UUID.u,
                                &svc_handle,
                                &chr_handle);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to find characteristic");
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(gesture_msg, strlen(gesture_msg));
    ble_gatts_notify_custom(g_conn_handle, chr_handle, om);
}


// --------------------------------------------------
// BLE Task to run in main.c
// Should do all the necessary set up before passing it to
// default NimBLE to handle continued advertising.
// --------------------------------------------------
extern "C" void bleOut_Task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting BLE Output Task");

    //Initialize NVS
    nvs_flash_init();

    nimble_port_init();

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;

    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    nimble_port_freertos_init([](void *){
        nimble_port_run();
        nimble_port_freertos_deinit();
    });

    vTaskDelete(NULL);
}
