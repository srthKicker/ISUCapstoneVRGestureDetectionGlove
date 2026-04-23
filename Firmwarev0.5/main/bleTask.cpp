#include <cstring>
#include <cstdint>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
//#include "esp_timer.h"

#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

//#define SERVICE_UUID        "c1b47598-8439-47b2-9ef7-69ee7c2349b8"
//#define CHARACTERISTIC_UUID "2316ba1a-066c-49d3-81e0-3f5c2599630e"

#define SERVICE_UUID        "fff0"
#define CHARACTERISTIC_UUID "fff1"

static const ble_uuid16_t service_uuid  = BLE_UUID16_INIT(0xFFF0);
static const ble_uuid16_t char_uuid     = BLE_UUID16_INIT(0xFFF1);


static uint8_t gesture_value = 0;


static int gesture_chr_access_cb(uint16_t conn_handle,
                                 uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt,
                                 void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        os_mbuf_append(ctxt->om, &gesture_value, sizeof(gesture_value));
    }
    return 0;
}

static uint16_t gesture_chr_handle;

static const struct  ble_gatt_chr_def gesture_characteristics[] = {
            {
                .uuid = &char_uuid.u,
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

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,
        .includes = nullptr,
        .characteristics = gesture_characteristics
        
    },
    {0}
};

// static void On_ble_sync(void) {
//     // Register GATT services first
//     ble_gatts_count_cfg(gatt_svcs);
//     ble_gatts_add_svcs(gatt_svcs);

//     // Then start advertising
//     struct ble_gap_adv_params adv_params = {};
//     adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
//     adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
//     ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
//                       &adv_params, NULL, NULL);
// }
static void On_ble_sync(void) {
    struct ble_gap_adv_params adv_params = {};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields = {};
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name                  = (const uint8_t *)"A.R.G.H V0.5";
    fields.name_len              = strlen("A.R.G.H V0.5");
    fields.name_is_complete      = 1;
    ble_gap_adv_set_fields(&fields);
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv_params, NULL, NULL);
    ESP_LOGI("BLE", "adv_start rc=%d", rc);  // Log the return code!
}

static void nimble_host_task(void *param) {
    nimble_port_run();         // blocks here running the NimBLE event loop
    nimble_port_freertos_deinit();
}

/*****************************************************
 * This is the BLE task to advertise the detected gesture.
 ***************************************************/
extern "C" void bleOut_Task(void *pvParameters){
    
    //Initialize NVS
    nvs_flash_init();

    //Initialize NimBLE host
    nimble_port_init();

    

    // Register GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    //Configure GAP Device Name
    ble_svc_gap_device_name_set("A.R.G.H V0.5");

    int rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    ble_hs_cfg.sync_cb = On_ble_sync;

    /*int rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    
    //Define GATT services and characteristic
    struct ble_gap_adv_params adv_params = {};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                  &adv_params, NULL, NULL);*/


    //Start advertising
    nimble_port_freertos_init(nimble_host_task);
}