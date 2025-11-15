#include <cstring>
#include <string>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
}

static const char *TAG = "BLE_HELLO";

// ---------- Simple OO wrapper for a tiny BLE app ----------
class BLEHello {
public:
    static BLEHello& instance() {
        static BLEHello i;
        return i;
    }

    // Start advertising once stack is ready
    void startAdvertising() {
        // Set advertising data (flags + name)
        ble_hs_adv_fields fields{};
        fields.flags = BLE_HS_ADV_F_BREDR_UNSUP | BLE_HS_ADV_F_DISC_GEN;
        fields.tx_pwr_lvl_is_present = 1;
        fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

        const char *name = ble_svc_gap_device_name();
        fields.name = (uint8_t*)name;
        fields.name_len = std::strlen(name);
        fields.name_is_complete = 1;

        int rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_set_fields: %d", rc);
            return;
        }

        // Advertising params
        ble_gap_adv_params advp{};
        advp.conn_mode = BLE_GAP_CONN_MODE_UND;  // connectable
        advp.disc_mode = BLE_GAP_DISC_MODE_GEN;  // general discoverable

        rc = ble_gap_adv_start(_own_addr_type, nullptr, BLE_HS_FOREVER, &advp,
                               &BLEHello::gapEventThunk, this);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_start: %d", rc);
        } else {
            ESP_LOGI(TAG, "Advertising as \"%s\" ...", name);
        }
    }

    // Called when the BLE host syncs with controller
    void onSync() {
        // Get a valid address type
        int rc = ble_hs_id_infer_auto(0, &_own_addr_type);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_hs_id_infer_auto: %d", rc);
            return;
        }

        // Optional: log MAC
        uint8_t addr_val[6]{};
        ble_hs_id_copy_addr(_own_addr_type, addr_val, nullptr);
        ESP_LOGI(TAG, "Device address: %02X:%02X:%02X:%02X:%02X:%02X",
                 addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0]);

        startAdvertising();
    }

    // Install GATT services (1 service, 1 readable char)
    void initGatt() {
        // Init standard GAP/GATT services (name, etc.)
        ble_svc_gap_init();
        ble_svc_gatt_init();

        ble_gatts_count_cfg(gatt_svcs);
        int rc = ble_gatts_add_svcs(gatt_svcs);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gatts_add_svcs: %d", rc);
        }
    }

    // GAP event handler
    int onGapEvent(struct ble_gap_event *event) {
        switch (event->type) {
            case BLE_GAP_EVENT_CONNECT:
                if (event->connect.status == 0) {
                    ESP_LOGI(TAG, "Connected; handle=%d", event->connect.conn_handle);
                } else {
                    ESP_LOGW(TAG, "Connect failed; restarting adv (status=%d)", event->connect.status);
                    startAdvertising();
                }
                break;

            case BLE_GAP_EVENT_DISCONNECT:
                ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
                startAdvertising();
                break;

            case BLE_GAP_EVENT_ADV_COMPLETE:
                ESP_LOGW(TAG, "Adv complete; restarting");
                startAdvertising();
                break;

            case BLE_GAP_EVENT_SUBSCRIBE:
                ESP_LOGI(TAG, "Subscribe event (chr val handle=%d, notify=%d, indicate=%d)",
                         event->subscribe.attr_handle, event->subscribe.cur_notify, event->subscribe.cur_indicate);
                break;

            default:
                break;
        }
        return 0;
    }

    // GATT access callback: returns "Hello BLE"
    int onChrAccess(struct ble_gatt_access_ctxt *ctxt) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            static const char msg[] = "Hello BLE";
            int rc = os_mbuf_append(ctxt->om, msg, sizeof(msg) - 1);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;
    }

    // Hook up C callbacks to C++ methods
    static int gapEventThunk(struct ble_gap_event *e, void *arg) {
        return static_cast<BLEHello*>(arg)->onGapEvent(e);
    }
    static int chrAccessThunk(uint16_t /*conn_handle*/, uint16_t /*attr_handle*/,
                              struct ble_gatt_access_ctxt *ctxt, void *arg) {
        return static_cast<BLEHello*>(arg)->onChrAccess(ctxt);
    }

private:
    BLEHello() = default;
    uint8_t _own_addr_type{0};

    // ---------- UUIDs ----------
    // 128-bit UUIDs (randomly generated example values)
    static constexpr ble_uuid128_t SVC_UUID =
            BLE_UUID128_INIT(0x6b,0x7f,0x0d,0x44,0x9a,0x1e,0x41,0x3e,0x8e,0x63,0x44,0x38,0x00,0x00,0xBE,0xEF);
    static constexpr ble_uuid128_t CHR_UUID =
            BLE_UUID128_INIT(0x3a,0x9f,0x3b,0x64,0x12,0x55,0x47,0x2d,0x9b,0x4b,0x88,0x2a,0x00,0x01,0xBE,0xEF);

    // ---------- GATT table ----------
    static int _chrAccess(uint16_t ch, uint16_t ah, struct ble_gatt_access_ctxt *ctxt, void *arg) {
        return BLEHello::chrAccessThunk(ch, ah, ctxt, arg);
    }

    static struct ble_gatt_chr_def* chrDefs() {
        static struct ble_gatt_chr_def chrs[] = {
                {
                        .uuid = const_cast<ble_uuid_t*>(&CHR_UUID.u),
                        .access_cb = _chrAccess,
                        .arg = &BLEHello::instance(),
                        .flags = BLE_GATT_CHR_F_READ,
                },
                { 0 } // end
        };
        return chrs;
    }

    static struct ble_gatt_svc_def* svcDefs() {
        static struct ble_gatt_svc_def svcs[] = {
                {
                        .type = BLE_GATT_SVC_TYPE_PRIMARY,
                        .uuid = const_cast<ble_uuid_t*>(&SVC_UUID.u),
                        .characteristics = chrDefs(),
                },
                { 0 } // end
        };
        return svcs;
    }

    static struct ble_gatt_svc_def* gatt_svcs;
};

// Define the static member
struct ble_gatt_svc_def* BLEHello::gatt_svcs = BLEHello::svcDefs();

// ---------- NimBLE host task ----------
static void host_task(void *param) {
    nimble_port_run();               // This blocks until NimBLE host stops
    nimble_port_freertos_deinit();   // Clean up
    vTaskDelete(nullptr);
}

// ---------- ESP-IDF entry ----------
extern "C" void app_main(void) {
    // NVS must be init for BLE stack
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());
    nimble_port_init();

    // Configure BLE host
    ble_hs_cfg.reset_cb = [](int reason) {
        ESP_LOGW(TAG, "Reset (reason=%d)", reason);
    };
    ble_hs_cfg.sync_cb = []() {
        // Install services and start advertising after sync
        BLEHello::instance().initGatt();
        ble_svc_gap_device_name_set("ESP32-NimBLE-Hello");
        BLEHello::instance().onSync();
    };

    // Launch NimBLE host on its own FreeRTOS task
    nimble_port_freertos_init(host_task);
}
