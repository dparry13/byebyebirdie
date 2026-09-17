#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "sound.h"
#include "hawkSound.h"

// Bluetooth includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define SLEEPTIME_MICROSECONDS (60000000) // 60 seconds
#define SOUND_TYPE 0
#define SPRAY_TYPE 1
#define SPRAY_GPIO_PIN GPIO_NUM_17
#define GPIO_WAKEUP_PIN GPIO_NUM_15
#define APP_TAG "Bye Bye Birdie"

// --- RTC Data: Preserved during deep sleep ---
RTC_DATA_ATTR static uint8_t soundCount;
RTC_DATA_ATTR static uint8_t sprayCount;
RTC_DATA_ATTR static bool firstBoot = true;
RTC_DATA_ATTR uint8_t soundTick;
RTC_DATA_ATTR uint8_t sprayTick;
RTC_DATA_ATTR static volatile bool sound_set_by_bluetooth;
RTC_DATA_ATTR static volatile bool spray_set_by_bluetooth;

// --- Global variables for BLE configuration ---
// These are the values you can set from your phone/controller.
// 'volatile' is used because they can be changed by the BLE task at any time.
static volatile uint8_t g_ble_sound_count_config;
static volatile uint8_t g_ble_spray_count_config;



// --- BLE UUID Definitions ---
// Main Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
static const ble_uuid128_t gatt_svc_uuid =
    BLE_UUID128_INIT(0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f, 0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f);

// Sound Count Characteristic UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8
static const ble_uuid128_t gatt_sound_config_chr_uuid =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7, 0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe);

// Spray Count Characteristic UUID: 1c95d5e3-d8f5-4542-a485-f58f3002b6a7
static const ble_uuid128_t gatt_spray_config_chr_uuid =
    BLE_UUID128_INIT(0xa7, 0xb6, 0x02, 0x30, 0x8f, 0xf5, 0x85, 0xa4, 0x42, 0x45, 0xf5, 0xd8, 0xe3, 0xd5, 0x95, 0x1c);


// Forward declarations
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
void ble_app_advertise(void);

static int gatt_svr_dsc_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    // The 'arg' parameter holds the string we provided in the definition
    const char* desc = (const char*)arg;
    os_mbuf_append(ctxt->om, desc, strlen(desc));
    return 0;
}
// --- GATT Service and Characteristics Definition ---
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                // Characteristic: Sound Trigger Count
                .uuid = &gatt_sound_config_chr_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            	.descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901), // Standard UUID for Characteristic User Description
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = gatt_svr_dsc_access,
                        .arg = "Sound Trigger Count"
                    },
                    { 0 } // End of descriptors
                }
            },
            {
                // Characteristic: Spray Trigger Count
                .uuid = &gatt_spray_config_chr_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            	.descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901), // Standard UUID for Characteristic User Description
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = gatt_svr_dsc_access,
                        .arg = "Spray Trigger Count"
                    },
                    { 0 } // End of descriptors
                }
            },
            {0}, // End of characteristics
        },
    },
    {0}, // End of services
};


// --- GATT Access Callback Function ---
// This function is called when a client tries to read or write a characteristic.
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    uint8_t current_val;

    // --- Sound Count Characteristic ---
    if (ble_uuid_cmp(uuid, &gatt_sound_config_chr_uuid.u) == 0) {
        switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Client is reading the current value
            current_val = (uint8_t)g_ble_sound_count_config;;
            os_mbuf_append(ctxt->om, &current_val, sizeof(current_val));
            return 0; // Success

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            // Client is writing a new value
            if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
                // Directly read the first byte from the data buffer
                g_ble_sound_count_config = ctxt->om->om_data[0];
                ESP_LOGI(APP_TAG, "BLE WRITE: New Sound Count Config = %d", (int)g_ble_sound_count_config);
            }
            return 0; // Success
        default:
            break;
        }
    }

    // --- Spray Count Characteristic ---
    if (ble_uuid_cmp(uuid, &gatt_spray_config_chr_uuid.u) == 0) {
        switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Client is reading the current value
            current_val = (uint8_t)g_ble_spray_count_config;
            os_mbuf_append(ctxt->om, &current_val, sizeof(current_val));
            return 0; // Success

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            // Client is writing a new value
            if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
                 // Directly read the first byte from the data buffer
                g_ble_spray_count_config = ctxt->om->om_data[0];
                ESP_LOGI(APP_TAG, "BLE WRITE: New Spray Count Config = %d", (int)g_ble_spray_count_config);
            }
            return 0; // Success
        default:
            break;
        }
    }

    // If the UUID doesn't match any of our characteristics, return an error
    return BLE_ATT_ERR_UNLIKELY;
}

// --- Standard BLE GAP Event Handler ---
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(APP_TAG, "BLE GAP EVENT CONNECT status=%d", event->connect.status);
            if (event->connect.status != 0) {
                ble_app_advertise(); // Connection failed, start advertising again
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(APP_TAG, "BLE GAP EVENT DISCONNECT");
            ble_app_advertise(); // Start advertising again to allow new connections
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(APP_TAG, "BLE GAP EVENT ADV_COMPLETE");
            ble_app_advertise();
            break;
        default:
            break;
    }
    return 0;
}

// --- Function to start BLE Advertising ---
void ble_app_advertise(void) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name = "Bye Bye Birdie";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// Called when the BLE Host is synced and ready
void ble_app_on_sync(void) {
    ble_svc_gap_device_name_set("Bye Bye Birdie");
    ble_app_advertise();
}

// NimBLE Host Task
void host_task(void *param) {
    ESP_LOGI(APP_TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// retrieve a new random time interval
uint8_t getTickTime(uint8_t type) {
    // Note: time(0) will not provide a good seed after deep sleep without an RTC.
    // Using boot_count for a simple pseudo-random seed.
    unsigned int seed = time(0);
    switch (type) {
        case SOUND_TYPE:
            return (rand_r(&seed) % (10 - 3 + 1) + 3); // Range 3-10
        case SPRAY_TYPE:
            return (rand_r(&seed) % (5 - 3 + 1) + 3);  // Range 3-5
        default:
            return 3;
    }
}

// Main application
void app_main(void) {
	//init function
	if (firstBoot == true) {
		soundCount = 3;
		sprayCount = 3;
		sound_set_volume(100);
		firstBoot = false;
	}
	
	//gpio_set_level(GPIO_WAKEUP_PIN, 0);

    // Initialize peripherals
    sound_init(HAWKSOUND_SAMPLE_RATE);
    gpio_reset_pin(SPRAY_GPIO_PIN);
    gpio_set_direction(SPRAY_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SPRAY_GPIO_PIN, 0);

    // Configure the GPIO pin that wakes the device from sleep
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << GPIO_WAKEUP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_config);

    // --- Check if we should enter BLE configuration mode ---
    // If the wakeup pin is held high on boot, start BLE.
    if (gpio_get_level(GPIO_WAKEUP_PIN) == 0) {
        ESP_LOGI(APP_TAG, "Wakeup pin is low. Starting BLE configuration mode.");
        g_ble_sound_count_config = soundCount;
        g_ble_spray_count_config = sprayCount;
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        nimble_port_init();
        ble_hs_cfg.sync_cb = ble_app_on_sync;
        ble_svc_gatt_init();
        ble_gatts_count_cfg(gatt_svcs);
        ble_gatts_add_svcs(gatt_svcs);
        nimble_port_freertos_init(host_task);
        
        ESP_LOGI(APP_TAG, "Configuration mode active. Release switch to exit.");

        // Wait here as long as the button is held down.
        while(gpio_get_level(GPIO_WAKEUP_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
        }

        // --- NEW: Exit BLE mode and restart ---
        ESP_LOGI(APP_TAG, "Switch released. Exiting configuration mode and restarting.");
		soundCount = g_ble_sound_count_config;
		if (g_ble_sound_count_config != 0) {
			sound_set_by_bluetooth = true;
		} else {
			soundCount = getTickTime(SOUND_TYPE);
		}
		
		sprayCount = g_ble_spray_count_config;
		if (g_ble_spray_count_config != 0) {
			spray_set_by_bluetooth = true;
		} else {
			sprayCount = getTickTime(SPRAY_TYPE);
		}
		
		soundTick = 0;
		sprayTick = 0;
		ESP_LOGI(APP_TAG, "New values saved: soundCount=%d, sprayCount=%d", soundCount, sprayCount);
		
        // Properly stop the nimble host task
        if (nimble_port_stop() == 0) {
            nimble_port_deinit();
            ESP_LOGI(APP_TAG, "NimBLE stack de-initialized.");
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Brief delay for cleanup
    }

    // --- Normal Operation (Device will sleep) ---
    ESP_LOGI(APP_TAG, "Normal operation mode.");

    soundTick++;
    sprayTick++;
    ESP_LOGI(APP_TAG, "Sound tick: %d/%d, Spray tick: %d/%d", soundTick, soundCount, sprayTick, sprayCount);

    if (soundTick >= soundCount && soundCount != 255) {
        ESP_LOGI(APP_TAG, "Activating sound...");
        sound_start(hawkSound, sizeof(hawkSound), true);
        soundTick = 0;

        if (!sound_set_by_bluetooth) {
			soundCount = getTickTime(SOUND_TYPE);
		}
		 // Get new random interval
        ESP_LOGI(APP_TAG, "New sound count target: %d", soundCount);
    }

    if (sprayTick >= sprayCount && sprayCount != 255) {
        ESP_LOGI(APP_TAG, "Activating spray...");
        gpio_set_level(SPRAY_GPIO_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10000)); // Spray for 10 seconds
        gpio_set_level(SPRAY_GPIO_PIN, 0);
        sprayTick = 0;
        
        if (!spray_set_by_bluetooth) {
			sprayCount = getTickTime(SPRAY_TYPE); // Get new random interval
		}
		
		if (sprayTick >= 254) {
			sprayTick = 0;
		}
		
		if (soundTick >= 254) {
			soundTick = 0;
		}
		
        ESP_LOGI(APP_TAG, "New spray count target: %d", sprayCount);
    }
    
    // --- Configure and Enter Deep Sleep ---
    ESP_LOGI(APP_TAG, "Entering deep sleep for %d microseconds.", SLEEPTIME_MICROSECONDS);
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_RTC);
    esp_sleep_enable_timer_wakeup(SLEEPTIME_MICROSECONDS);
    // Configure wakeup on low signal for the GPIO pin
    esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 0);
    esp_deep_sleep_start();
}
