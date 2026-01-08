//BLUETOOTH 3
//SMART PLUG: EmonLib Algorithm for Low Power Measurement
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"
#include "qrcode.h"

// --- MQTT CONFIG ---
#define ESP_BROKER_URI      "mqtt://0.tcp.ap.ngrok.io:15403" 
#define MQTT_USERNAME       "admin"
#define MQTT_PASSWORD       "114125121"
#define CONTROL_TOPIC       "/plugin/control"
#define STATUS_TOPIC        "/plugin/status"

// --- HARDWARE CONFIG ---
#define ADC_CHANNEL         ADC_CHANNEL_3
#define ADC_UNIT            ADC_UNIT_1
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define RELAY_GPIO          8

// --- SENSOR SENSITIVITY ---
// ACS712-5A: 185.0 | ACS712-20A: 100.0 | ACS712-30A: 66.0
#define SENSITIVITY         185.0 

// --- CALIBRATION ---
// 1. Chỉnh CALIB_FACTOR để dòng đo được khớp với thực tế ở tải lớn (>100W)
#define CALIB_FACTOR        0.98

// 2. Chỉnh NOISE_RMS_AMPS để khử dòng ảo khi không tải
// Cách tìm: Set số này = 0, chạy code khi KHÔNG cắm tải, xem Log báo bao nhiêu (vd: 0.045A) thì điền vào đây.
#define NOISE_RMS_AMPS      0.078  

// 3. Số mẫu lấy mỗi lần đo. 
// Bỏ delay, lấy mẫu càng nhanh càng tốt. 4000 mẫu mất khoảng 200ms trên ESP32
#define NUMBER_OF_SAMPLES   3000   

// --- PROVISIONING CONFIG ---
#define PROV_QR_VERSION     "v1"
#define PROV_TRANSPORT_BLE  "ble"
#define QRCODE_BASE_URL     "https://espressif.github.io/esp-jumpstart/qrcode.html"
#define PROV_SEC2_USERNAME  "wifiprov"
#define PROV_SEC2_PWD       "abcd1234"
#define MAX_PROV_RETRY      5
#define MAX_RETRY_NETIF     3

static const char *TAG = "SMART_PLUG";

// --- Global Variables ---
adc_oneshot_unit_handle_t adc_handle = NULL;
adc_cali_handle_t adc_cali_handle = NULL;
bool do_calibration = false;
esp_mqtt_client_handle_t mqtt_client = NULL;
bool mqtt_connected = false;
int current_relay_state = 0;
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_EVENT = BIT0;
static int s_retry_num = 0;

// --- ADC Functions ---
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&(adc_cali_curve_fitting_config_t){
        .unit_id = unit, .chan = channel, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT,
    }, &handle);
    if (ret == ESP_OK) {
        *out_handle = handle;
        return true;
    }
    return false;
}

// ===== THUẬT TOÁN EMONLIB: DIGITAL FILTER =====
// Thuật toán này tự động lọc bỏ thành phần DC (điểm 0)
// và tính RMS cực kỳ chính xác cho tải nhỏ.
float get_current_rms() {
    int sample = 0;
    int voltage_mv = 0;
    
    double filtered_I = 0;
    double offset_I = 0;     // DC Offset động
    double sqI = 0;
    double sum_I = 0;
    
    // Biến lưu giá trị mẫu trước đó để lọc
    static double last_filtered_I = 0;
    static int last_sample_mv = 0; 

    // Khởi tạo giá trị đầu tiên để tránh cú sốc bộ lọc
    adc_oneshot_read(adc_handle, ADC_CHANNEL, &sample);
    if (do_calibration) adc_cali_raw_to_voltage(adc_cali_handle, sample, &last_sample_mv);
    else last_sample_mv = sample * 3300 / 4095;

    // VÒNG LẶP LẤY MẪU TỐC ĐỘ CAO (KHÔNG DÙNG DELAY)
    for (int n = 0; n < NUMBER_OF_SAMPLES; n++) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL, &sample);
        
        if (do_calibration) {
            adc_cali_raw_to_voltage(adc_cali_handle, sample, &voltage_mv);
        } else {
            voltage_mv = sample * 3300 / 4095;
        }

        // --- Digital High Pass Filter ---
        // Công thức: y[n] = 0.996 * (y[n-1] + x[n] - x[n-1])
        // Mục đích: Loại bỏ thành phần một chiều (DC Offset) để chỉ còn lại sóng xoay chiều (AC)
        last_filtered_I = 0.996 * (last_filtered_I + voltage_mv - last_sample_mv);
        
        // Cập nhật mẫu cũ
        last_sample_mv = voltage_mv;
        
        // Bình phương tín hiệu đã lọc (Root Mean SQUARE)
        sqI = last_filtered_I * last_filtered_I;
        sum_I += sqI;
    }

    // Tính RMS từ tổng bình phương
    double I_ratio = 1.0; // Tỷ lệ (nếu cần scale thêm)
    double I_rms_voltage = I_ratio * sqrt(sum_I / NUMBER_OF_SAMPLES);
    
    // Chuyển đổi từ Voltage RMS sang Current RMS
    float current = I_rms_voltage / SENSITIVITY;
    
    // --- NOISE SUBTRACTION (KHỬ NHIỄU NỀN) ---
    // Công thức: I_thực = sqrt(|I_đo^2 - I_nhiễu^2|)
    // Giúp đo được dòng nhỏ (vd: 0.05A) dù nhiễu nền là 0.04A
    if (current < NOISE_RMS_AMPS) {
        current = 0.0;
    } else {
        current = sqrt((current * current) - (NOISE_RMS_AMPS * NOISE_RMS_AMPS));
    }
    
    current = current * CALIB_FACTOR;

    return (float)current;
}

// --- MQTT Handler ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    char response_payload[128];

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "✅ MQTT Connected");
            mqtt_connected = true;
            esp_mqtt_client_subscribe(mqtt_client, CONTROL_TOPIC, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, CONTROL_TOPIC, event->topic_len) == 0) {
                if (event->data_len == 1) {
                    if (event->data[0] == '1') {
                        current_relay_state = 1;
                        gpio_set_level(RELAY_GPIO, 1);
                    } else if (event->data[0] == '0') {
                        current_relay_state = 0;
                        gpio_set_level(RELAY_GPIO, 0);
                    }
                    
                    vTaskDelay(pdMS_TO_TICKS(500));
                    
                    // Nếu relay tắt, ép về 0 luôn
                    float I = (current_relay_state == 1) ? get_current_rms() : 0.0;
                    float P = I * 220.0;
                    
                    snprintf(response_payload, sizeof(response_payload), 
                             "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", I, P, current_relay_state);
                    esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, response_payload, 0, 1, 0);
                }
            }
            break;
        default: break;
    }
}

// --- Helper Functions ---
static void get_device_service_name(char *service_name, size_t max) {
    uint8_t eth_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "PLUG_%02X%02X%02X", eth_mac[3], eth_mac[4], eth_mac[5]);
}

static void print_qr_code(const char *name, const char *username, const char *pop, const char *transport) {
    if (!name) return;
    char payload[150];
    snprintf(payload, sizeof(payload), 
             "{\"ver\":\"%s\",\"name\":\"%s\",\"username\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}",
             PROV_QR_VERSION, name, username, pop, transport);
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&cfg, payload);
    ESP_LOGI(TAG, "Provisioning Pass: %s", pop);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    static int prov_retries = 0;
    if (event_base == WIFI_PROV_EVENT) {
        if (event_id == WIFI_PROV_CRED_FAIL) {
            prov_retries++;
            if (prov_retries >= MAX_PROV_RETRY) wifi_prov_mgr_reset_sm_state_on_failure();
        } else if (event_id == WIFI_PROV_CRED_SUCCESS) prov_retries = 0;
    } else if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (s_retry_num < MAX_RETRY_NETIF) { esp_wifi_connect(); s_retry_num++; }
            else { wifi_prov_mgr_reset_provisioning(); esp_restart(); }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
        esp_mqtt_client_config_t mqtt_cfg = {
            .broker.address.uri = ESP_BROKER_URI,
            .credentials = {.username = MQTT_USERNAME, .authentication = {.password = MQTT_PASSWORD}}
        };
        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(mqtt_client);
    }
}

// --- Main Task ---
void monitoring_task(void *pvParameters) {
    char payload[128];
    // Bộ lọc trung bình động (Moving Average) để số nhảy mượt hơn
    float current_avg = 0;
    
    while (1) {
        int relay_stat = gpio_get_level(RELAY_GPIO);
        float I = 0.0;
        
        if (relay_stat == 1) {
            I = get_current_rms();
        } else {
            I = 0.0;
            current_avg = 0.0; // Reset bộ lọc
        }

        // Lọc hiển thị: Cập nhật từ từ (Low Pass Filter cho hiển thị)
        if (current_avg == 0) current_avg = I;
        else current_avg = (current_avg * 0.8) + (I * 0.2);

        // Hiển thị 0 nếu quá nhỏ sau khi đã trừ nhiễu
        if (current_avg < 0.005) current_avg = 0.0;

        float P = current_avg * 220.0;

        if (mqtt_connected) {
            snprintf(payload, sizeof(payload), 
                     "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
                     current_avg, P, relay_stat);
            esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, payload, 0, 1, 0);
        }
        
        ESP_LOGI(TAG, "RAW: %.3f A | AVG: %.3f A | P: %.1f W", I, current_avg, P);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    
    gpio_config(&(gpio_config_t){
        .pin_bit_mask = (1ULL << RELAY_GPIO), .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE
    });
    gpio_set_level(RELAY_GPIO, 0);

    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
    adc_oneshot_new_unit(&init_config, &adc_handle);
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);
    do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &adc_cali_handle);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_prov_mgr_config_t prov_config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_config));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));
        uint8_t custom_uuid[] = { 0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf, 0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02 };
        wifi_prov_scheme_ble_set_service_uuid(custom_uuid);
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1, PROV_SEC2_PWD, service_name, NULL));
        print_qr_code(service_name, PROV_SEC2_USERNAME, PROV_SEC2_PWD, PROV_TRANSPORT_BLE);
    } else {
        wifi_prov_mgr_deinit();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, pdMS_TO_TICKS(10000));
    xTaskCreate(monitoring_task, "monitoring", 4096, NULL, 5, NULL);
}


// //BLUETOOTH 2
// //SMART PLUG với WiFi Provisioning + MQTT Authentication
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "esp_log.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "mqtt_client.h"
// #include "driver/gpio.h"
// #include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"
// #include "wifi_provisioning/manager.h"
// #include "wifi_provisioning/scheme_ble.h"
// #include "qrcode.h"

// // --- MQTT CONFIG (WITH AUTHENTICATION) ---
// #define ESP_BROKER_URI      "mqtt://0.tcp.ap.ngrok.io:18055"  // Thay bằng Ngrok URL của bạn
// #define MQTT_USERNAME       "admin"                            // MQTT username
// #define MQTT_PASSWORD       "114125121"                         // MQTT password
// #define CONTROL_TOPIC       "/plugin/control"
// #define STATUS_TOPIC        "/plugin/status"

// // --- HARDWARE CONFIG ---
// #define ADC_CHANNEL         ADC_CHANNEL_3
// #define ADC_UNIT            ADC_UNIT_1
// #define ADC_ATTEN           ADC_ATTEN_DB_12
// #define SENSITIVITY         185.0
// #define CALIB_FACTOR        0.235
// #define RELAY_GPIO          8

// // --- PROVISIONING CONFIG ---
// #define PROV_QR_VERSION     "v1"
// #define PROV_TRANSPORT_BLE  "ble"
// #define QRCODE_BASE_URL     "https://espressif.github.io/esp-jumpstart/qrcode.html"
// #define PROV_SEC2_USERNAME  "wifiprov"
// #define PROV_SEC2_PWD       "abcd1234"
// #define MAX_PROV_RETRY      5

// // --- AUTO RESET CONFIG ---
// #define MAX_RETRY_NETIF     3

// static const char *TAG = "SMART_PLUG";

// // --- Security 2 Parameters ---
// static const char sec2_salt[] = {
//     0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 
//     0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4
// };

// static const char sec2_verifier[] = {
//     0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43,
//     0x78, 0xcf, 0xfd, 0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24,
//     0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9, 0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1, 0x32, 0x24, 0xda, 0x11,
//     0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a, 0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89,
//     0xaa, 0x16, 0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6, 0xe4, 0x3a, 0x71, 0x2e,
//     0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff, 0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87, 0xdc,
//     0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35, 0x6a, 0xc4,
//     0x70, 0xad, 0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e,
//     0xc3, 0xa5, 0xf9, 0x68, 0xdd, 0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e,
//     0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18, 0x34, 0x29, 0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9,
//     0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e, 0x5f, 0x08, 0x33, 0x40, 0x76,
//     0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05, 0x44, 0x17,
//     0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44, 0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23,
//     0x81, 0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17, 0xbd, 0x08, 0xb8, 0x96,
//     0xbc, 0x26, 0xeb, 0x32, 0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc,
//     0xa9, 0x38, 0xa6, 0x67, 0xf0, 0xad, 0x50, 0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2,
//     0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b, 0xb1, 0xe2, 0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64,
//     0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6, 0x21, 0xe7, 0x2d,
//     0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84,
//     0x53, 0x4a, 0xb3, 0x0c, 0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb,
//     0xec, 0xf0, 0x10, 0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84, 0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0,
//     0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7, 0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8,
//     0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71, 0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1,
//     0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62, 0xde, 0x36, 0xb2, 0xba
// };

// // --- Global Variables ---
// adc_oneshot_unit_handle_t adc_handle = NULL;
// adc_cali_handle_t adc_cali_handle = NULL;
// bool do_calibration = false;
// esp_mqtt_client_handle_t mqtt_client = NULL;
// bool mqtt_connected = false;
// int current_relay_state = 0;
// static EventGroupHandle_t wifi_event_group;
// const int WIFI_CONNECTED_EVENT = BIT0;
// static int s_retry_num = 0;

// // --- Security Functions ---
// static esp_err_t get_sec2_salt(const char **salt, uint16_t *salt_len) {
//     *salt = sec2_salt;
//     *salt_len = sizeof(sec2_salt);
//     return ESP_OK;
// }

// static esp_err_t get_sec2_verifier(const char **verifier, uint16_t *verifier_len) {
//     *verifier = sec2_verifier;
//     *verifier_len = sizeof(sec2_verifier);
//     return ESP_OK;
// }

// // --- ADC Functions ---
// static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
//     adc_cali_handle_t handle = NULL;
//     esp_err_t ret = adc_cali_create_scheme_curve_fitting(&(adc_cali_curve_fitting_config_t){
//         .unit_id = unit, .chan = channel, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT,
//     }, &handle);
//     if (ret == ESP_OK) {
//         *out_handle = handle;
//         return true;
//     }
//     return false;
// }

// float get_current_rms() {
//     int voltage_raw = 0, voltage_mv = 0;
//     int max_mv = 0, min_mv = 5000;
//     uint32_t start_tick = xTaskGetTickCount();
    
//     while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(100)) {
//         adc_oneshot_read(adc_handle, ADC_CHANNEL, &voltage_raw);
//         if (do_calibration) {
//             adc_cali_raw_to_voltage(adc_cali_handle, voltage_raw, &voltage_mv);
//         } else {
//             voltage_mv = voltage_raw * 3300 / 4095;
//         }
//         if (voltage_mv > max_mv) max_mv = voltage_mv;
//         if (voltage_mv < min_mv) min_mv = voltage_mv;
//     }
//     if (min_mv == 5000 || max_mv == 0) return 0.0;
//     float v_pp = (float)(max_mv - min_mv);
//     float current = (v_pp * 0.3535) / SENSITIVITY;
//     current = current * CALIB_FACTOR;
//     return (current < 0.06) ? 0.0 : current;
// }

// // --- MQTT Handler ---
// static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
//     esp_mqtt_event_handle_t event = event_data;
//     char response_payload[128];

//     switch ((esp_mqtt_event_id_t)event_id) {
//         case MQTT_EVENT_CONNECTED:
//             ESP_LOGI(TAG, "✅ MQTT Connected to: %s", ESP_BROKER_URI);
//             ESP_LOGI(TAG, "   Username: %s", MQTT_USERNAME);
//             mqtt_connected = true;
//             esp_mqtt_client_subscribe(mqtt_client, CONTROL_TOPIC, 0);
//             ESP_LOGI(TAG, "📥 Subscribed to: %s", CONTROL_TOPIC);
            
//             float init_current = get_current_rms();
//             float init_power = init_current * 220.0;
//             int init_relay = gpio_get_level(RELAY_GPIO);
            
//             snprintf(response_payload, sizeof(response_payload), 
//                      "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                      init_current, init_power, init_relay);
//             esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, response_payload, 0, 1, 0);
//             break;

//         case MQTT_EVENT_DISCONNECTED:
//             ESP_LOGI(TAG, "❌ MQTT Disconnected");
//             mqtt_connected = false;
//             break;

//         case MQTT_EVENT_ERROR:
//             ESP_LOGE(TAG, "❌ MQTT Error!");
//             if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
//                 ESP_LOGE(TAG, "   TCP Transport error");
//             } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
//                 ESP_LOGE(TAG, "   Connection refused - Check broker/username/password!");
//             }
//             break;

//         case MQTT_EVENT_DATA:
//             if (strncmp(event->topic, CONTROL_TOPIC, event->topic_len) == 0) {
//                 if (event->data_len == 1) {
//                     if (event->data[0] == '1') {
//                         current_relay_state = 1;
//                         gpio_set_level(RELAY_GPIO, 1);
//                         ESP_LOGI(TAG, "💡 Relay ON");
//                     } else if (event->data[0] == '0') {
//                         current_relay_state = 0;
//                         gpio_set_level(RELAY_GPIO, 0);
//                         ESP_LOGI(TAG, "🔴 Relay OFF");
//                     }

//                     float quick_current = get_current_rms();
//                     float quick_power = quick_current * 220.0;
                    
//                     snprintf(response_payload, sizeof(response_payload), 
//                              "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                              quick_current, quick_power, current_relay_state);
                    
//                     esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, response_payload, 0, 1, 0);
//                 }
//             }
//             break;
//         default: 
//             break;
//     }
// }

// // --- QR Code Generator ---
// static void print_qr_code(const char *name, const char *username, const char *pop, const char *transport) {
//     if (!name || !transport) {
//         ESP_LOGW(TAG, "Cannot generate QR code");
//         return;
//     }
    
//     char payload[150] = {0};
//     snprintf(payload, sizeof(payload), 
//              "{\"ver\":\"%s\",\"name\":\"%s\",\"username\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}",
//              PROV_QR_VERSION, name, username, pop, transport);
    
//     ESP_LOGI(TAG, "\n\n========================================");
//     ESP_LOGI(TAG, "📱 QUÉT MÃ QR NÀY BẰNG APP FLUTTER");
//     ESP_LOGI(TAG, "========================================\n");
    
//     esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
//     esp_qrcode_generate(&cfg, payload);
    
//     ESP_LOGI(TAG, "\n========================================");
//     ESP_LOGI(TAG, "📱 Device Name: %s", name);
//     ESP_LOGI(TAG, "🔐 Username: %s", username);
//     ESP_LOGI(TAG, "🔑 Password: %s", pop);
//     ESP_LOGI(TAG, "========================================\n");
//     ESP_LOGI(TAG, "URL: %s?data=%s", QRCODE_BASE_URL, payload);
// }

// // --- Get Unique Device Name ---
// static void get_device_service_name(char *service_name, size_t max) {
//     uint8_t eth_mac[6];
//     const char *ssid_prefix = "PLUG_";
//     esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
//     snprintf(service_name, max, "%s%02X%02X%02X",
//              ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
// }

// // --- Event Handler ---
// static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
//     static int prov_retries = 0;
    
//     if (event_base == WIFI_PROV_EVENT) {
//         switch (event_id) {
//             case WIFI_PROV_START:
//                 ESP_LOGI(TAG, "🔧 Provisioning started");
//                 break;
//             case WIFI_PROV_CRED_RECV: {
//                 wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
//                 ESP_LOGI(TAG, "📡 Received WiFi credentials");
//                 ESP_LOGI(TAG, "   SSID: %s", (const char *)wifi_sta_cfg->ssid);
//                 break;
//             }
//             case WIFI_PROV_CRED_FAIL:
//                 ESP_LOGE(TAG, "❌ Provisioning failed!");
//                 prov_retries++;
//                 if (prov_retries >= MAX_PROV_RETRY) {
//                     ESP_LOGI(TAG, "🔄 Resetting provisioning...");
//                     wifi_prov_mgr_reset_sm_state_on_failure();
//                     prov_retries = 0;
//                 }
//                 break;
//             case WIFI_PROV_CRED_SUCCESS:
//                 ESP_LOGI(TAG, "✅ Provisioning successful!");
//                 prov_retries = 0;
//                 break;
//             case WIFI_PROV_END:
//                 wifi_prov_mgr_deinit();
//                 break;
//             default:
//                 break;
//         }
//     } else if (event_base == WIFI_EVENT) {
//         switch (event_id) {
//             case WIFI_EVENT_STA_START:
//                 esp_wifi_connect();
//                 break;
                
//             case WIFI_EVENT_STA_DISCONNECTED:
//                 ESP_LOGI(TAG, "🔄 WiFi disconnected");
                
//                 if (s_retry_num < MAX_RETRY_NETIF) {
//                     esp_wifi_connect();
//                     s_retry_num++;
//                     ESP_LOGI(TAG, "⚠️ Retry connection %d/%d", s_retry_num, MAX_RETRY_NETIF);
//                 } else {
//                     ESP_LOGW(TAG, "❌ Connection failed too many times. FACTORY RESETTING...");
//                     wifi_prov_mgr_config_t prov_config = {
//                         .scheme = wifi_prov_scheme_ble,
//                     };
//                     wifi_prov_mgr_init(prov_config);
//                     wifi_prov_mgr_reset_provisioning();
//                     ESP_LOGW(TAG, "♻️ Rebooting system...");
//                     esp_restart();
//                 }
//                 break;
                
//             default:
//                 break;
//         }
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
//         ESP_LOGI(TAG, "✅ Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
//         s_retry_num = 0;
//         xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
        
//         // Start MQTT with authentication
//         ESP_LOGI(TAG, "🔌 Connecting to MQTT broker...");
//         esp_mqtt_client_config_t mqtt_cfg = {
//             .broker.address.uri = ESP_BROKER_URI,
//             .credentials = {
//                 .username = MQTT_USERNAME,
//                 .authentication = {
//                     .password = MQTT_PASSWORD,
//                 }
//             }
//         };
//         mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
//         esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
//         esp_mqtt_client_start(mqtt_client);
//     }
// }

// // --- Main Task ---
// void monitoring_task(void *pvParameters) {
//     char payload[128];
    
//     while (1) {
//         float I = get_current_rms();
//         float P = I * 220.0;
//         int relay_stat = gpio_get_level(RELAY_GPIO);

//         if (mqtt_connected) {
//             snprintf(payload, sizeof(payload), 
//                      "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                      I, P, relay_stat);
//             esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, payload, 0, 1, 0);
//         }
        
//         ESP_LOGI(TAG, "📊 I: %.3f A | P: %.1f W | Relay: %d", I, P, relay_stat);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

// // --- MAIN ---
// void app_main(void) {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ESP_ERROR_CHECK(nvs_flash_init());
//     }

//     gpio_config(&(gpio_config_t){
//         .pin_bit_mask = (1ULL << RELAY_GPIO),
//         .mode = GPIO_MODE_INPUT_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     });
//     gpio_set_level(RELAY_GPIO, 0); 
//     current_relay_state = 0;
//     ESP_LOGI(TAG, "🔧 Relay GPIO %d initialized (OFF)", RELAY_GPIO);

//     adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
//     adc_oneshot_new_unit(&init_config, &adc_handle);
//     adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN };
//     adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);
//     do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &adc_cali_handle);
//     ESP_LOGI(TAG, "📊 ADC calibration: %s", do_calibration ? "OK" : "Skipped");

//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     wifi_event_group = xEventGroupCreate();

//     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
//     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
//     ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

//     esp_netif_create_default_wifi_sta();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     wifi_prov_mgr_config_t prov_config = {
//         .scheme = wifi_prov_scheme_ble,
//         .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
//     };
//     ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_config));

//     bool provisioned = false;
//     ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

//     if (!provisioned) {
//         ESP_LOGI(TAG, "🔧 Starting WiFi Provisioning...");

//         char service_name[12];
//         get_device_service_name(service_name, sizeof(service_name));

//         //wifi_prov_security_t security = WIFI_PROV_SECURITY_2;
//         wifi_prov_security_t security = WIFI_PROV_SECURITY_1;  // ✅ SỬA
//         // wifi_prov_security2_params_t sec2_params = {};
//         // ESP_ERROR_CHECK(get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
//         // ESP_ERROR_CHECK(get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));
//         const char *pop = PROV_SEC2_PWD;  // ✅ Sử dụng trực tiếp password

//         uint8_t custom_service_uuid[] = {
//             0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
//             0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
//         };
//         wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

//         // ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)&sec2_params, 
//         //                                                  service_name, NULL));
//         ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, pop,  // ✅ SỬA
//                                                  service_name, NULL));

//         print_qr_code(service_name, PROV_SEC2_USERNAME, PROV_SEC2_PWD, PROV_TRANSPORT_BLE);
//     } else {
//         ESP_LOGI(TAG, "✅ Already provisioned, starting WiFi...");
//         wifi_prov_mgr_deinit();
//         ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//         ESP_ERROR_CHECK(esp_wifi_start());
//     }

//     xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, pdMS_TO_TICKS(10000));
//     ESP_LOGI(TAG, "🚀 Smart Plug Monitoring Started");

//     xTaskCreate(monitoring_task, "monitoring", 4096, NULL, 5, NULL);
// }

//BLUETOOTH 1
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "esp_log.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "mqtt_client.h"
// #include "driver/gpio.h"
// #include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"
// #include "wifi_provisioning/manager.h"
// #include "wifi_provisioning/scheme_ble.h"
// #include "qrcode.h"

// // --- MQTT CONFIG ---
// #define ESP_BROKER_URI      "mqtt://10.169.156.30:1883"
// #define CONTROL_TOPIC       "/plugin/control"
// #define STATUS_TOPIC        "/plugin/status"

// // --- HARDWARE CONFIG ---
// #define ADC_CHANNEL         ADC_CHANNEL_3
// #define ADC_UNIT            ADC_UNIT_1
// #define ADC_ATTEN           ADC_ATTEN_DB_12
// #define SENSITIVITY         185.0
// #define CALIB_FACTOR        0.235
// #define RELAY_GPIO          8

// // --- PROVISIONING CONFIG ---
// #define PROV_QR_VERSION     "v1"
// #define PROV_TRANSPORT_BLE  "ble"
// #define QRCODE_BASE_URL     "https://espressif.github.io/esp-jumpstart/qrcode.html"
// #define PROV_SEC2_USERNAME  "wifiprov"
// #define PROV_SEC2_PWD       "abcd1234"
// #define MAX_PROV_RETRY      5

// // --- AUTO RESET CONFIG (NEW) ---
// #define MAX_RETRY_NETIF     10  // Thử kết nối 10 lần không được thì Reset

// static const char *TAG = "SMART_PLUG";

// // --- Security 2 Parameters ---
// static const char sec2_salt[] = {
//     0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 
//     0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4
// };

// static const char sec2_verifier[] = {
//     0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43,
//     0x78, 0xcf, 0xfd, 0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24,
//     0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9, 0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1, 0x32, 0x24, 0xda, 0x11,
//     0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a, 0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89,
//     0xaa, 0x16, 0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6, 0xe4, 0x3a, 0x71, 0x2e,
//     0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff, 0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87, 0xdc,
//     0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35, 0x6a, 0xc4,
//     0x70, 0xad, 0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e,
//     0xc3, 0xa5, 0xf9, 0x68, 0xdd, 0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e,
//     0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18, 0x34, 0x29, 0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9,
//     0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e, 0x5f, 0x08, 0x33, 0x40, 0x76,
//     0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05, 0x44, 0x17,
//     0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44, 0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23,
//     0x81, 0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17, 0xbd, 0x08, 0xb8, 0x96,
//     0xbc, 0x26, 0xeb, 0x32, 0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc,
//     0xa9, 0x38, 0xa6, 0x67, 0xf0, 0xad, 0x50, 0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2,
//     0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b, 0xb1, 0xe2, 0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64,
//     0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6, 0x21, 0xe7, 0x2d,
//     0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84,
//     0x53, 0x4a, 0xb3, 0x0c, 0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb,
//     0xec, 0xf0, 0x10, 0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84, 0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0,
//     0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7, 0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8,
//     0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71, 0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1,
//     0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62, 0xde, 0x36, 0xb2, 0xba
// };

// // --- Global Variables ---
// adc_oneshot_unit_handle_t adc_handle = NULL;
// adc_cali_handle_t adc_cali_handle = NULL;
// bool do_calibration = false;
// esp_mqtt_client_handle_t mqtt_client = NULL;
// bool mqtt_connected = false;
// int current_relay_state = 0;
// static EventGroupHandle_t wifi_event_group;
// const int WIFI_CONNECTED_EVENT = BIT0;
// static int s_retry_num = 0; // Biến đếm số lần kết nối lại

// // --- Security Functions ---
// static esp_err_t get_sec2_salt(const char **salt, uint16_t *salt_len) {
//     *salt = sec2_salt;
//     *salt_len = sizeof(sec2_salt);
//     return ESP_OK;
// }

// static esp_err_t get_sec2_verifier(const char **verifier, uint16_t *verifier_len) {
//     *verifier = sec2_verifier;
//     *verifier_len = sizeof(sec2_verifier);
//     return ESP_OK;
// }

// // --- ADC Functions ---
// static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
//     adc_cali_handle_t handle = NULL;
//     esp_err_t ret = adc_cali_create_scheme_curve_fitting(&(adc_cali_curve_fitting_config_t){
//         .unit_id = unit, .chan = channel, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT,
//     }, &handle);
//     if (ret == ESP_OK) {
//         *out_handle = handle;
//         return true;
//     }
//     return false;
// }

// float get_current_rms() {
//     int voltage_raw = 0, voltage_mv = 0;
//     int max_mv = 0, min_mv = 5000;
//     uint32_t start_tick = xTaskGetTickCount();
    
//     while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(100)) {
//         adc_oneshot_read(adc_handle, ADC_CHANNEL, &voltage_raw);
//         if (do_calibration) {
//             adc_cali_raw_to_voltage(adc_cali_handle, voltage_raw, &voltage_mv);
//         } else {
//             voltage_mv = voltage_raw * 3300 / 4095;
//         }
//         if (voltage_mv > max_mv) max_mv = voltage_mv;
//         if (voltage_mv < min_mv) min_mv = voltage_mv;
//     }
//     if (min_mv == 5000 || max_mv == 0) return 0.0;
//     float v_pp = (float)(max_mv - min_mv);
//     float current = (v_pp * 0.3535) / SENSITIVITY;
//     current = current * CALIB_FACTOR;
//     return (current < 0.06) ? 0.0 : current;
// }

// // --- MQTT Handler ---
// static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
//     esp_mqtt_event_handle_t event = event_data;
//     char response_payload[128];

//     switch ((esp_mqtt_event_id_t)event_id) {
//         case MQTT_EVENT_CONNECTED:
//             ESP_LOGI(TAG, "✅ MQTT Connected");
//             mqtt_connected = true;
//             esp_mqtt_client_subscribe(mqtt_client, CONTROL_TOPIC, 0);
//             ESP_LOGI(TAG, "📥 Subscribed to: %s", CONTROL_TOPIC);
            
//             float init_current = get_current_rms();
//             float init_power = init_current * 220.0;
//             int init_relay = gpio_get_level(RELAY_GPIO);
            
//             snprintf(response_payload, sizeof(response_payload), 
//                      "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                      init_current, init_power, init_relay);
//             esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, response_payload, 0, 1, 0);
//             break;

//         case MQTT_EVENT_DISCONNECTED:
//             ESP_LOGI(TAG, "❌ MQTT Disconnected");
//             mqtt_connected = false;
//             break;

//         case MQTT_EVENT_DATA:
//             if (strncmp(event->topic, CONTROL_TOPIC, event->topic_len) == 0) {
//                 if (event->data_len == 1) {
//                     if (event->data[0] == '1') {
//                         current_relay_state = 1;
//                         gpio_set_level(RELAY_GPIO, 1);
//                         ESP_LOGI(TAG, "💡 Relay ON");
//                     } else if (event->data[0] == '0') {
//                         current_relay_state = 0;
//                         gpio_set_level(RELAY_GPIO, 0);
//                         ESP_LOGI(TAG, "🔴 Relay OFF");
//                     }

//                     float quick_current = get_current_rms();
//                     float quick_power = quick_current * 220.0;
                    
//                     snprintf(response_payload, sizeof(response_payload), 
//                              "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                              quick_current, quick_power, current_relay_state);
                    
//                     esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, response_payload, 0, 1, 0);
//                 }
//             }
//             break;
//         default: 
//             break;
//     }
// }

// // --- QR Code Generator ---
// static void print_qr_code(const char *name, const char *username, const char *pop, const char *transport) {
//     if (!name || !transport) {
//         ESP_LOGW(TAG, "Cannot generate QR code");
//         return;
//     }
    
//     char payload[150] = {0};
//     snprintf(payload, sizeof(payload), 
//              "{\"ver\":\"%s\",\"name\":\"%s\",\"username\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}",
//              PROV_QR_VERSION, name, username, pop, transport);
    
//     ESP_LOGI(TAG, "\n\n========================================");
//     ESP_LOGI(TAG, "📱 QUÉT MÃ QR NÀY BẰNG APP FLUTTER");
//     ESP_LOGI(TAG, "========================================\n");
    
//     esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
//     esp_qrcode_generate(&cfg, payload);
    
//     ESP_LOGI(TAG, "\n========================================");
//     ESP_LOGI(TAG, "📱 Device Name: %s", name);
//     ESP_LOGI(TAG, "🔐 Username: %s", username);
//     ESP_LOGI(TAG, "🔑 Password: %s", pop);
//     ESP_LOGI(TAG, "========================================\n");
//     ESP_LOGI(TAG, "URL: %s?data=%s", QRCODE_BASE_URL, payload);
// }

// // --- Get Unique Device Name ---
// static void get_device_service_name(char *service_name, size_t max) {
//     uint8_t eth_mac[6];
//     const char *ssid_prefix = "PLUG_";
//     esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
//     snprintf(service_name, max, "%s%02X%02X%02X",
//              ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
// }

// // --- Event Handler ---
// static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
//     static int prov_retries = 0;
    
//     if (event_base == WIFI_PROV_EVENT) {
//         switch (event_id) {
//             case WIFI_PROV_START:
//                 ESP_LOGI(TAG, "🔧 Provisioning started");
//                 break;
//             case WIFI_PROV_CRED_RECV: {
//                 wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
//                 ESP_LOGI(TAG, "📡 Received WiFi credentials");
//                 ESP_LOGI(TAG, "   SSID: %s", (const char *)wifi_sta_cfg->ssid);
//                 break;
//             }
//             case WIFI_PROV_CRED_FAIL:
//                 ESP_LOGE(TAG, "❌ Provisioning failed!");
//                 prov_retries++;
//                 if (prov_retries >= MAX_PROV_RETRY) {
//                     ESP_LOGI(TAG, "🔄 Resetting provisioning...");
//                     wifi_prov_mgr_reset_sm_state_on_failure();
//                     prov_retries = 0;
//                 }
//                 break;
//             case WIFI_PROV_CRED_SUCCESS:
//                 ESP_LOGI(TAG, "✅ Provisioning successful!");
//                 prov_retries = 0;
//                 break;
//             case WIFI_PROV_END:
//                 // Không Deinit ở đây nếu muốn giữ manager để reset sau này,
//                 // nhưng để tiết kiệm RAM thì vẫn deinit, ta sẽ init lại khi cần reset.
//                 wifi_prov_mgr_deinit();
//                 break;
//             default:
//                 break;
//         }
//     } else if (event_base == WIFI_EVENT) {
//         switch (event_id) {
//             case WIFI_EVENT_STA_START:
//                 esp_wifi_connect();
//                 break;
                
//             case WIFI_EVENT_STA_DISCONNECTED:
//                 ESP_LOGI(TAG, "🔄 WiFi disconnected");
                
//                 if (s_retry_num < MAX_RETRY_NETIF) {
//                     esp_wifi_connect();
//                     s_retry_num++;
//                     ESP_LOGI(TAG, "⚠️ Retry connection %d/%d", s_retry_num, MAX_RETRY_NETIF);
//                 } else {
//                     ESP_LOGW(TAG, "❌ Connection failed too many times. FACTORY RESETTING...");
                    
//                     // Chúng ta cần khởi tạo lại Manager để có thể gọi hàm Reset Provisioning
//                     wifi_prov_mgr_config_t prov_config = {
//                         .scheme = wifi_prov_scheme_ble,
//                     };
//                     wifi_prov_mgr_init(prov_config);
                    
//                     // Xóa toàn bộ cấu hình Provisioning
//                     wifi_prov_mgr_reset_provisioning();
                    
//                     ESP_LOGW(TAG, "♻️ Rebooting system...");
//                     esp_restart();
//                 }
//                 break;
                
//             default:
//                 break;
//         }
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
//         ESP_LOGI(TAG, "✅ Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
//         // Reset bộ đếm thử lại khi đã có mạng thành công
//         s_retry_num = 0;
        
//         xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
        
//         // Start MQTT
//         esp_mqtt_client_config_t mqtt_cfg = {
//             .broker.address.uri = ESP_BROKER_URI,
//         };
//         mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
//         esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
//         esp_mqtt_client_start(mqtt_client);
//     }
// }

// // --- Main Task ---
// void monitoring_task(void *pvParameters) {
//     char payload[128];
    
//     while (1) {
//         // Gửi status định kỳ
//         float I = get_current_rms();
//         float P = I * 220.0;
//         int relay_stat = gpio_get_level(RELAY_GPIO);

//         if (mqtt_connected) {
//             snprintf(payload, sizeof(payload), 
//                      "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                      I, P, relay_stat);
//             esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, payload, 0, 1, 0);
//         }
        
//         ESP_LOGI(TAG, "📊 I: %.3f A | P: %.1f W | Relay: %d", I, P, relay_stat);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

// // --- MAIN ---
// void app_main(void) {
//     // 1. NVS Init
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ESP_ERROR_CHECK(nvs_flash_init());
//     }

//     // 2. GPIO Setup - Relay
//     gpio_config(&(gpio_config_t){
//         .pin_bit_mask = (1ULL << RELAY_GPIO),
//         .mode = GPIO_MODE_INPUT_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     });
//     gpio_set_level(RELAY_GPIO, 0); 
//     current_relay_state = 0;
//     ESP_LOGI(TAG, "🔧 Relay GPIO %d initialized (OFF)", RELAY_GPIO);

//     // 3. ADC Setup
//     adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
//     adc_oneshot_new_unit(&init_config, &adc_handle);
//     adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN };
//     adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);
//     do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &adc_cali_handle);
//     ESP_LOGI(TAG, "📊 ADC calibration: %s", do_calibration ? "OK" : "Skipped");

//     // 4. Network Init
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     wifi_event_group = xEventGroupCreate();

//     // 5. Register Event Handlers
//     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
//     ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
//     ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

//     // 6. WiFi Init
//     esp_netif_create_default_wifi_sta();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     // 7. Provisioning Manager Config
//     wifi_prov_mgr_config_t prov_config = {
//         .scheme = wifi_prov_scheme_ble,
//         .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
//     };
//     ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_config));

//     // 8. Check if provisioned
//     bool provisioned = false;
//     ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

//     if (!provisioned) {
//         ESP_LOGI(TAG, "🔧 Starting WiFi Provisioning...");

//         // Get unique device name
//         char service_name[12];
//         get_device_service_name(service_name, sizeof(service_name));

//         // Security setup
//         wifi_prov_security_t security = WIFI_PROV_SECURITY_2;
//         wifi_prov_security2_params_t sec2_params = {};
//         ESP_ERROR_CHECK(get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
//         ESP_ERROR_CHECK(get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

//         // Custom BLE UUID
//         uint8_t custom_service_uuid[] = {
//             0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
//             0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
//         };
//         wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

//         // Start provisioning
//         ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)&sec2_params, 
//                                                          service_name, NULL));

//         // Print QR Code
//         print_qr_code(service_name, PROV_SEC2_USERNAME, PROV_SEC2_PWD, PROV_TRANSPORT_BLE);
//     } else {
//         ESP_LOGI(TAG, "✅ Already provisioned, starting WiFi...");
//         wifi_prov_mgr_deinit();
//         ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//         ESP_ERROR_CHECK(esp_wifi_start());
//     }

//     // 9. Wait for WiFi connection
//     // Lưu ý: Không dùng portMAX_DELAY nữa vì nếu mất mạng nó sẽ reset, 
//     // ta cứ để task này chạy để vào monitoring
//     xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, pdMS_TO_TICKS(10000));
//     ESP_LOGI(TAG, "🚀 Smart Plug Monitoring Started");

//     // 10. Start monitoring task
//     xTaskCreate(monitoring_task, "monitoring", 4096, NULL, 5, NULL);
// }


// //PLUGIN 4
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "esp_log.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "mqtt_client.h"
// #include "driver/gpio.h"
// #include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"

// // --- CAU HINH WIFI & MQTT ---
// #define ESP_WIFI_SSID       "urmom"
// #define ESP_WIFI_PASS       "114125121"
// // #define ESP_WIFI_SSID       "Dai hoc Thuy Loi"
// // #define ESP_WIFI_PASS       "Phanhieu@2022"
// #define ESP_BROKER_URI      "mqtt://10.169.156.30:1883"


// // --- MQTT TOPICS - TÁCH BIỆT RÕ RÀNG ---
// #define CONTROL_TOPIC       "/plugin/control"  // CHỈ NHẬN lệnh 0/1
// #define STATUS_TOPIC        "/plugin/status"   // CHỈ GỬI JSON status

// // --- CAU HINH PHAN CUNG ---
// #define ADC_CHANNEL         ADC_CHANNEL_3
// #define ADC_UNIT            ADC_UNIT_1
// #define ADC_ATTEN           ADC_ATTEN_DB_12
// #define SENSITIVITY         185.0
// #define CALIB_FACTOR        0.235
// #define RELAY_GPIO          8

// static const char *TAG = "SMART_PLUG";

// // Bien toan cuc
// adc_oneshot_unit_handle_t adc_handle = NULL;
// adc_cali_handle_t adc_cali_handle = NULL;
// bool do_calibration = false;
// esp_mqtt_client_handle_t mqtt_client = NULL;
// bool mqtt_connected = false;
// int current_relay_state = 0;

// // --- ADC CALIBRATION ---
// static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
//     adc_cali_handle_t handle = NULL;
//     esp_err_t ret = adc_cali_create_scheme_curve_fitting(&(adc_cali_curve_fitting_config_t){
//         .unit_id = unit, .chan = channel, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT,
//     }, &handle);
//     if (ret == ESP_OK) {
//         *out_handle = handle;
//         return true;
//     }
//     return false;
// }

// // --- DOC DONG DIEN RMS ---
// float get_current_rms() {
//     int voltage_raw = 0, voltage_mv = 0;
//     int max_mv = 0, min_mv = 5000;
//     uint32_t start_tick = xTaskGetTickCount();
    
//     while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(100)) {
//         adc_oneshot_read(adc_handle, ADC_CHANNEL, &voltage_raw);
//         if (do_calibration) {
//             adc_cali_raw_to_voltage(adc_cali_handle, voltage_raw, &voltage_mv);
//         } else {
//             voltage_mv = voltage_raw * 3300 / 4095;
//         }
//         if (voltage_mv > max_mv) max_mv = voltage_mv;
//         if (voltage_mv < min_mv) min_mv = voltage_mv;
//     }
//     if (min_mv == 5000 || max_mv == 0) return 0.0;
//     float v_pp = (float)(max_mv - min_mv);
//     float current = (v_pp * 0.3535) / SENSITIVITY;
//     current = current * CALIB_FACTOR;
//     return (current < 0.06) ? 0.0 : current;
// }

// // --- MQTT EVENT HANDLER - FIXED ---
// static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
//     esp_mqtt_event_handle_t event = event_data;
//     char response_payload[128];

//     switch ((esp_mqtt_event_id_t)event_id) {
//         case MQTT_EVENT_CONNECTED:
//             ESP_LOGI(TAG, "✅ MQTT Connected");
//             mqtt_connected = true;
            
//             // CHỈ SUBSCRIBE CONTROL TOPIC - KHÔNG SUBSCRIBE STATUS
//             esp_mqtt_client_subscribe(mqtt_client, CONTROL_TOPIC, 0);
//             ESP_LOGI(TAG, "📥 Subscribed to: %s", CONTROL_TOPIC);
            
//             // Gửi status ban đầu
//             float init_current = get_current_rms();
//             float init_power = init_current * 220.0;
//             int init_relay = gpio_get_level(RELAY_GPIO);
            
//             snprintf(response_payload, sizeof(response_payload), 
//                      "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                      init_current, init_power, init_relay);
//             esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, response_payload, 0, 1, 0);
//             ESP_LOGI(TAG, "📤 Initial status sent");
//             break;

//         case MQTT_EVENT_DISCONNECTED:
//             ESP_LOGI(TAG, "❌ MQTT Disconnected");
//             mqtt_connected = false;
//             break;

//         case MQTT_EVENT_DATA:
//             ESP_LOGI(TAG, "📩 Received message on topic: %.*s", event->topic_len, event->topic);
            
//             // CHỈ XỬ LÝ MESSAGE TỪ CONTROL_TOPIC
//             if (strncmp(event->topic, CONTROL_TOPIC, event->topic_len) == 0) {
//                 // Chỉ xử lý message đơn giản "0" hoặc "1"
//                 // Bỏ qua tất cả message khác (JSON, số khác, ...)
//                 if (event->data_len == 1) {
//                     if (event->data[0] == '1') {
//                         current_relay_state = 1;
//                         gpio_set_level(RELAY_GPIO, 1);
//                         ESP_LOGI(TAG, "💡 Relay ON");
//                     } 
//                     else if (event->data[0] == '0') {
//                         current_relay_state = 0;
//                         gpio_set_level(RELAY_GPIO, 0);
//                         ESP_LOGI(TAG, "🔴 Relay OFF");
//                     }
//                     else {
//                         ESP_LOGW(TAG, "⚠️ Unknown control: %c", event->data[0]);
//                         break;
//                     }

//                     // PHẢN HỒI NHANH: Gửi status mới VÀO STATUS_TOPIC
//                     float quick_current = get_current_rms();
//                     float quick_power = quick_current * 220.0;
//                     int relay_stat = gpio_get_level(RELAY_GPIO);
                    
//                     snprintf(response_payload, sizeof(response_payload), 
//                              "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                              quick_current, quick_power, relay_stat);
                    
//                     esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, response_payload, 0, 1, 0);
//                     ESP_LOGI(TAG, "📤 Quick response: %s", response_payload);
//                 } else {
//                     // Bỏ qua message dài hơn 1 ký tự (có thể là JSON lạc vào)
//                     ESP_LOGW(TAG, "⚠️ Ignored non-control message (len=%d)", event->data_len);
//                 }
//             }
//             break;

//         default: 
//             break;
//     }
// }

// // --- WIFI EVENT HANDLER ---
// static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         esp_wifi_connect();
//     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         ESP_LOGI(TAG, "🔄 WiFi disconnected, reconnecting...");
//         esp_wifi_connect();
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
//         ESP_LOGI(TAG, "✅ Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
//         esp_mqtt_client_config_t mqtt_cfg = {
//             .broker.address.uri = ESP_BROKER_URI,
//         };
//         mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
//         esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
//         esp_mqtt_client_start(mqtt_client);
//     }
// }

// void wifi_init(void) {
//     esp_netif_init();
//     esp_event_loop_create_default();
//     esp_netif_create_default_wifi_sta();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     esp_wifi_init(&cfg);
//     esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
//     esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
//     wifi_config_t wifi_config = {
//         .sta = { .ssid = ESP_WIFI_SSID, .password = ESP_WIFI_PASS },
//     };
//     esp_wifi_set_mode(WIFI_MODE_STA);
//     esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
//     esp_wifi_start();
// }

// void app_main(void) {
//     // 1. NVS & GPIO & ADC
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // GPIO Setup
//     gpio_config(&(gpio_config_t){
//         .pin_bit_mask = (1ULL << RELAY_GPIO),
//         .mode = GPIO_MODE_INPUT_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     });
//     gpio_set_level(RELAY_GPIO, 0); 
//     current_relay_state = 0;
//     ESP_LOGI(TAG, "🔧 GPIO %d initialized (Relay OFF)", RELAY_GPIO);

//     // ADC Setup
//     adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
//     adc_oneshot_new_unit(&init_config, &adc_handle);
//     adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN };
//     adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);
//     do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &adc_cali_handle);
//     ESP_LOGI(TAG, "📊 ADC calibration: %s", do_calibration ? "OK" : "Skipped");

//     // 2. WiFi
//     wifi_init();

//     // 3. Loop gửi status định kỳ (2 giây)
//     char payload[128];
//     while (1) {
//         float I = get_current_rms();
//         float P = I * 220.0;
//         int relay_stat = gpio_get_level(RELAY_GPIO);

//         if (mqtt_connected) {
//             snprintf(payload, sizeof(payload), 
//                      "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", 
//                      I, P, relay_stat);
            
//             // CHỈ PUBLISH VÀO STATUS_TOPIC
//             esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, payload, 0, 1, 0);
//         }
        
//         ESP_LOGI(TAG, "Status → I: %.3f A | P: %.1f W | Relay: %d", I, P, relay_stat);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }



// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "esp_log.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "protocol_examples_common.h"
// #include "mqtt_client.h"
// #include "driver/gpio.h"
// #include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"

// // --- CAU HINH WIFI & MQTT ---
// // #define ESP_WIFI_SSID       "Dai hoc Thuy Loi"
// // #define ESP_WIFI_PASS       "Phanhieu@2022"
// #define ESP_WIFI_SSID       "urmom"
// #define ESP_WIFI_PASS       "114125121"
// #define ESP_BROKER_URI      "mqtt://10.214.10.30:1883"

// // --- MQTT TOPICS ---
// #define CONTROL_TOPIC       "/plugin/control"
// #define STATUS_TOPIC        "/plugin/status"

// // --- CAU HINH PHAN CUNG ---
// #define ADC_CHANNEL         ADC_CHANNEL_3       // GPIO 2
// #define ADC_UNIT            ADC_UNIT_1
// #define ADC_ATTEN           ADC_ATTEN_DB_12
// #define SENSITIVITY         185.0
// #define CALIB_FACTOR        0.235
// #define RELAY_GPIO          8                   // Relay tren GPIO8

// static const char *TAG = "SMART_PLUG";

// // Bien toan cuc
// adc_oneshot_unit_handle_t adc_handle = NULL;
// adc_cali_handle_t adc_cali_handle = NULL;
// bool do_calibration = false;
// esp_mqtt_client_handle_t mqtt_client = NULL;
// bool mqtt_connected = false;

// // --- ADC CALIBRATION INIT ---
// static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
//     adc_cali_handle_t handle = NULL;
//     esp_err_t ret = adc_cali_create_scheme_curve_fitting(&(adc_cali_curve_fitting_config_t){
//         .unit_id = unit, .chan = channel, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT,
//     }, &handle);
//     if (ret == ESP_OK) {
//         *out_handle = handle;
//         return true;
//     }
//     return false;
// }

// // --- DOC DONG DIEN RMS ---
// float get_current_rms() {
//     int voltage_raw = 0, voltage_mv = 0;
//     int max_mv = 0, min_mv = 5000;
//     uint32_t start_tick = xTaskGetTickCount();
    
//     while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(100)) {
//         adc_oneshot_read(adc_handle, ADC_CHANNEL, &voltage_raw);
//         if (do_calibration) {
//             adc_cali_raw_to_voltage(adc_cali_handle, voltage_raw, &voltage_mv);
//         } else {
//             voltage_mv = voltage_raw * 3300 / 4095;
//         }
//         if (voltage_mv > max_mv) max_mv = voltage_mv;
//         if (voltage_mv < min_mv) min_mv = voltage_mv;
//     }
//     if (min_mv == 5000 || max_mv == 0) return 0.0;
//     float v_pp = (float)(max_mv - min_mv);
//     float current = (v_pp * 0.3535) / SENSITIVITY;
//     current = current * CALIB_FACTOR;
//     return (current < 0.06) ? 0.0 : current;
// }

// // --- MQTT EVENT HANDLER ---
// static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
//     esp_mqtt_event_handle_t event = event_data;
//     switch ((esp_mqtt_event_id_t)event_id) {
//         case MQTT_EVENT_CONNECTED:
//             ESP_LOGI(TAG, "MQTT Connected");
//             mqtt_connected = true;
//             esp_mqtt_client_subscribe(mqtt_client, CONTROL_TOPIC, 0);
//             break;
//         case MQTT_EVENT_DISCONNECTED:
//             mqtt_connected = false;
//             break;
//         case MQTT_EVENT_DATA:
//             if (strncmp(event->topic, CONTROL_TOPIC, event->topic_len) == 0) {
//                 if (strncmp(event->data, "1", event->data_len) == 0) {
//                     gpio_set_level(RELAY_GPIO, 1);
//                     ESP_LOGI(TAG, "Relay ON via MQTT");
//                 } else if (strncmp(event->data, "0", event->data_len) == 0) {
//                     gpio_set_level(RELAY_GPIO, 0);
//                     ESP_LOGI(TAG, "Relay OFF via MQTT");
//                 }
//             }
//             break;
//         default: break;
//     }
// }

// // --- WIFI EVENT HANDLER ---
// static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         esp_wifi_connect();
//     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         esp_wifi_connect();
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         esp_mqtt_client_config_t mqtt_cfg = { .broker.address.uri = ESP_BROKER_URI };
//         mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
//         esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
//         esp_mqtt_client_start(mqtt_client);
//     }
// }

// // --- KHOI TAO WIFI ---
// void wifi_init(void) {
//     esp_netif_init();
//     esp_event_loop_create_default();
//     esp_netif_create_default_wifi_sta();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     esp_wifi_init(&cfg);
//     esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
//     esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
//     wifi_config_t wifi_config = {
//         .sta = { .ssid = ESP_WIFI_SSID, .password = ESP_WIFI_PASS },
//     };
//     esp_wifi_set_mode(WIFI_MODE_STA);
//     esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
//     esp_wifi_start();
// }

// void app_main(void) {
//     // 1. NVS & GPIO & ADC
//     nvs_flash_init();
//     gpio_config(&(gpio_config_t){
//         .pin_bit_mask = (1ULL << RELAY_GPIO),
//         .mode = GPIO_MODE_INPUT_OUTPUT, // BẮT BUỘC phải có INPUT_OUTPUT
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     });
//     gpio_set_level(RELAY_GPIO, 0); // Mac dinh OFF

//     adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
//     adc_oneshot_new_unit(&init_config, &adc_handle);
//     adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN };
//     adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);
//     do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &adc_cali_handle);

//     // 2. WiFi & MQTT
//     wifi_init();

//     // 3. Loop gui du lieu status
//     char payload[64];
//     while (1) {
//         float I = get_current_rms();
//         float P = I * 220.0;
        
//         if (mqtt_connected) {
//             // Thêm biến đọc trạng thái GPIO để gửi kèm vào JSON
//             int relay_stat = gpio_get_level(RELAY_GPIO);
//             snprintf(payload, sizeof(payload), "{\"current\":%.3f,\"power\":%.1f,\"relay\":%d}", I, P, relay_stat);
//             esp_mqtt_client_publish(mqtt_client, STATUS_TOPIC, payload, 0, 1, 0);
//         }
//         ESP_LOGI(TAG, "I: %.3f A | P: %.1f W", I, P);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }




// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "esp_log.h"
// #include "driver/gpio.h"
// #include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"

// // --- CAU HINH PHAN CUNG ADC (ACS712) ---
// #define ADC_UNIT            ADC_UNIT_1
// #define ADC_CHANNEL         ADC_CHANNEL_3       // GPIO 2 (Noi voi chan Analog Output cua ACS712)
// #define ADC_ATTEN           ADC_ATTEN_DB_12
// #define SENSITIVITY         185.0
// #define CALIB_FACTOR        0.235

// // --- CAU HINH RELAY ---
// #define RELAY_GPIO          8                   // GPIO8 tren ESP32-C3

// static const char *TAG = "ACS_METER";

// adc_oneshot_unit_handle_t adc_handle = NULL;
// adc_cali_handle_t adc_cali_handle = NULL;
// bool do_calibration = false;

// static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
//     adc_cali_handle_t handle = NULL;
//     esp_err_t ret = ESP_FAIL;
//     bool calibrated = false;
//     adc_cali_curve_fitting_config_t cali_config = {
//         .unit_id = unit, .chan = channel, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT,
//     };
//     ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
//     if (ret == ESP_OK) {
//         calibrated = true;
//         *out_handle = handle;
//     }
//     return calibrated;
// }

// float get_current_rms() {
//     int voltage_raw = 0;
//     int voltage_mv = 0;
//     int max_mv = 0;
//     int min_mv = 5000;
    
//     uint32_t start_tick = xTaskGetTickCount();
    
//     while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(100)) {
//         ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &voltage_raw));
        
//         if (do_calibration) {
//             ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, voltage_raw, &voltage_mv));
//         } else {
//             voltage_mv = voltage_raw * 3300 / 4095;
//         }

//         if (voltage_mv > max_mv) max_mv = voltage_mv;
//         if (voltage_mv < min_mv) min_mv = voltage_mv;
//     }

//     if (min_mv == 5000 || max_mv == 0) return 0.0;

//     float v_pp = (float)(max_mv - min_mv);
//     float current = (v_pp * 0.3535) / SENSITIVITY;
//     current = current * CALIB_FACTOR;

//     if (current < 0.08) {
//         current = 0.0;
//     }
//     return current;
// }

// void app_main(void) {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 1. Cau hinh ADC
//     adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
//     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
//     adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN };
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));
//     do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &adc_cali_handle);

//     // 2. Cau hinh GPIO cho Relay
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << RELAY_GPIO),
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     };
//     gpio_config(&io_conf);

//     // Mặc định bật relay ngay khi khởi động
//     gpio_set_level(RELAY_GPIO, 1);

//     ESP_LOGI(TAG, "System Initialized. Relay is ON by default.");

//     while (1) {
//         float I = get_current_rms();
//         float P_watt = I * 220.0;

//         ESP_LOGI(TAG, "Current: %.3f A | Power: %.1f W", I, P_watt);

//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }

//     if (do_calibration) adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
//     adc_oneshot_del_unit(adc_handle);
// }


//////// 2


// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "esp_log.h"
// #include "driver/gpio.h"
// #include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"

// // --- CAU HINH PHAN CUNG ADC (ACS712) ---
// #define ADC_UNIT            ADC_UNIT_1
// #define ADC_CHANNEL         ADC_CHANNEL_2       // GPIO 2
// #define ADC_ATTEN           ADC_ATTEN_DB_12
// #define SENSITIVITY         185.0               // Do nhay cho ACS712-5A (185 mV/A)
// #define CALIB_FACTOR        1.0                 // He so hieu chinh (1.0 = khong hieu chinh)
// #define ZERO_CURRENT_MV     2500                // Dien ap OUT khi I=0A (2.5V)

// static const char *TAG = "ACS712";

// // Bien toan cuc
// adc_oneshot_unit_handle_t adc_handle = NULL;
// adc_cali_handle_t adc_cali_handle = NULL;
// bool do_calibration = false;

// // --- HAM ADC VA HIEU CHUAN ---
// static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
//     adc_cali_handle_t handle = NULL;
//     esp_err_t ret = ESP_FAIL;
//     bool calibrated = false;
//     adc_cali_curve_fitting_config_t cali_config = {
//         .unit_id = unit, 
//         .chan = channel, 
//         .atten = atten, 
//         .bitwidth = ADC_BITWIDTH_DEFAULT,
//     };
//     ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
//     if (ret == ESP_OK) {
//         calibrated = true;
//         *out_handle = handle;
//     }
//     return calibrated;
// }

// /**
//  * @brief Do dong dien RMS voi ACS712 - TRU OFFSET 2.5V
//  * @param raw_out: Tra ve gia tri ADC raw trung binh
//  * @param voltage_out: Tra ve dien ap (mV) trung binh
//  * @param vmax_out: Tra ve dien ap MAX (mV)
//  * @param vmin_out: Tra ve dien ap MIN (mV)
//  * @return Dong dien RMS (Ampere)
//  */
// float get_current_rms(int *raw_out, int *voltage_out, int *vmax_out, int *vmin_out) {
//     int voltage_raw = 0;
//     int voltage_mv = 0;
//     int max_mv = 0;
//     int min_mv = 5000; // Khoi tao lon de dam bao cap nhat
    
//     int sum_raw = 0;
//     int sum_mv = 0;
//     int count = 0;
    
//     uint32_t start_tick = xTaskGetTickCount();
    
//     // Do trong 100ms (khoang 5 chu ky AC 50Hz)
//     while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(100)) {
//         ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &voltage_raw));
        
//         if (do_calibration) {
//             ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, voltage_raw, &voltage_mv));
//         } else {
//             voltage_mv = voltage_raw * 3300 / 4095;
//         }

//         // Cap nhat min/max
//         if (voltage_mv > max_mv) max_mv = voltage_mv;
//         if (voltage_mv < min_mv) min_mv = voltage_mv;
        
//         // Tinh trung binh
//         sum_raw += voltage_raw;
//         sum_mv += voltage_mv;
//         count++;
        
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }

//     // Gia tri trung binh
//     *raw_out = (count > 0) ? (sum_raw / count) : 0;
//     *voltage_out = (count > 0) ? (sum_mv / count) : 0;
//     *vmax_out = max_mv;
//     *vmin_out = min_mv;

//     // Kiem tra loi doc ADC
//     if (count == 0 || max_mv == 0) {
//         return 0.0;
//     }

//     // *** PHAN QUAN TRONG: TRU OFFSET 2.5V ***
//     // Tinh V_peak-to-peak (mV) - DA TRU OFFSET
//     float v_pp = (float)(max_mv - min_mv);
    
//     // Tinh V_peak (mV)
//     float v_peak = v_pp / 2.0;
    
//     // Tinh V_rms (mV)
//     float v_rms = v_peak * 0.707; // 1/sqrt(2)
    
//     // Tinh I_rms (A)
//     float current = v_rms / SENSITIVITY;
    
//     // Ap dung he so hieu chinh
//     current = current * CALIB_FACTOR;
    
//     // Loc nhieu - bo qua gia tri qua nho
//     // Nguong 0.25A de tranh nhieu ESP32 ADC
//     if (current < 0.1) {
//         current = 0.0;
//     }

//     return current;
// }

// /**
//  * @brief Lay dong dien RMS co loc trung binh de giam nhieu
//  * @param raw_out: Tra ve gia tri ADC raw
//  * @param voltage_out: Tra ve dien ap (mV)
//  * @param vmax_out: Tra ve dien ap MAX (mV)
//  * @param vmin_out: Tra ve dien ap MIN (mV)
//  * @return Dong dien RMS trung binh (Ampere)
//  */
// float get_filtered_current(int *raw_out, int *voltage_out, int *vmax_out, int *vmin_out) {
//     float sum = 0.0;
//     int samples = 3; // Lay 3 mau va tinh trung binh
    
//     for (int i = 0; i < samples; i++) {
//         sum += get_current_rms(raw_out, voltage_out, vmax_out, vmin_out);
//         if (i < samples - 1) {
//             vTaskDelay(pdMS_TO_TICKS(50)); // Delay 50ms giua cac mau
//         }
//     }
    
//     return sum / samples;
// }

// // --- CHUONG TRINH CHINH ---
// void app_main(void) {
//     // Khoi tao NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 1. Cau hinh ADC
//     adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
//     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    
//     adc_oneshot_chan_cfg_t config = { 
//         .bitwidth = ADC_BITWIDTH_DEFAULT, 
//         .atten = ADC_ATTEN 
//     };
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));
    
//     do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN, &adc_cali_handle);

//     ESP_LOGI(TAG, "System Init OK. Calibration: %s", do_calibration ? "YES" : "NO");

//     // Vong lap do chinh
//     while (1) {
//         int raw = 0;
//         int voltage = 0;
//         int vmax = 0;
//         int vmin = 0;
        
//         // Su dung ham loc trung binh de giam nhieu
//         float I = get_filtered_current(&raw, &voltage, &vmax, &vmin);
        
//         // Tinh Vpp va cong suat
//         int vpp = vmax - vmin;
//         float P = I * 220.0; // Cong suat (W)
        
//         // In log chi tiet
//         if (I == 0.0) {
//             ESP_LOGI(TAG, "raw=%d, V=%d mV (Vpp=%d mV) | I=%.3f A | P=0 W", 
//                      raw, voltage, vpp, I);
//         } else {
//             ESP_LOGI(TAG, "raw=%d, V=%d mV (max=%d, min=%d, Vpp=%d mV) | I=%.3f A | P=%.1f W", 
//                      raw, voltage, vmax, vmin, vpp, I, P);
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }

//     // Cleanup
//     if (do_calibration) {
//         adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
//     }
//     adc_oneshot_del_unit(adc_handle);
// }





// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_adc/adc_oneshot.h"
// #include "driver/gpio.h"
// #include <math.h>

// #define TAG "ACS712"

// /* ================== RELAY CONFIG ================== */
// #define RELAY_PIN GPIO_NUM_8  // Thay đổi theo chân GPIO của bạn

// /* ================== ADC CONFIG ================== */
// #define ACS_ADC_UNIT      ADC_UNIT_1
// #define ACS_ADC_CHANNEL   ADC_CHANNEL_3
// #define ACS_ADC_ATTEN     ADC_ATTEN_DB_11
// #define ADC_VREF_MV       3100.0f
// #define ADC_MAX           4095.0f

// /* ================== ACS712 CONFIG ================= */
// #define ACS_SENS_MV_PER_A 185.0f

// /* ================== NOISE FILTER ================== */
// #define CURRENT_THRESHOLD 0.02f  // Dòng điện dưới 20mA coi như = 0

// void app_main(void)
// {
//     /* ---------- Init Relay ---------- */
//     gpio_reset_pin(RELAY_PIN);
//     gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_level(RELAY_PIN, 1);  // BẬT RELAY
//     ESP_LOGI(TAG, "Relay ON");

//     adc_oneshot_unit_handle_t adc;

//     /* ---------- Init ADC Unit ---------- */
//     adc_oneshot_unit_init_cfg_t unit_cfg = {
//         .unit_id = ACS_ADC_UNIT,
//     };
//     adc_oneshot_new_unit(&unit_cfg, &adc);

//     /* ---------- Config ADC Channel ---------- */
//     adc_oneshot_chan_cfg_t chan_cfg = {
//         .bitwidth = ADC_BITWIDTH_DEFAULT,
//         .atten    = ACS_ADC_ATTEN,
//     };
//     adc_oneshot_config_channel(adc, ACS_ADC_CHANNEL, &chan_cfg);

//     /* ---------- Calibrate Offset (NO CURRENT) ---------- */
//     int offset_raw = 0;

//     for (int i = 0; i < 100; i++) {
//         int raw = 0;
//         adc_oneshot_read(adc, ACS_ADC_CHANNEL, &raw);
//         offset_raw += raw;
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     offset_raw /= 100;
//     float offset_mv = offset_raw * ADC_VREF_MV / ADC_MAX;

//     ESP_LOGI(TAG,
//              "Offset raw=%d, offset_mv=%.2f mV",
//              offset_raw,
//              offset_mv);

//     /* ---------- Read Current ---------- */
//     while (1) {
//         int raw = 0;
//         adc_oneshot_read(adc, ACS_ADC_CHANNEL, &raw);

//         float mv = raw * ADC_VREF_MV / ADC_MAX;
//         float current_A = (mv - offset_mv) / ACS_SENS_MV_PER_A;

//         // ===== LỌC NHIỄU DÒNG ĐIỆN =====
//         if (fabs(current_A) < CURRENT_THRESHOLD) {
//             current_A = 0.0f;
//         }

//         ESP_LOGI(TAG,
//                  "raw=%4d | V=%.1f mV | I=%.3f A",
//                  raw,
//                  mv,
//                  current_A);

//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }


// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_adc/adc_oneshot.h"

// #define TAG "ACS712"

// /* ================== ADC CONFIG ================== */
// // ADC1_CH2 = GPIO3 (ESP32-S3)
// #define ACS_ADC_UNIT      ADC_UNIT_1
// #define ACS_ADC_CHANNEL   ADC_CHANNEL_3
// #define ACS_ADC_ATTEN     ADC_ATTEN_DB_11   // ~0 – 3.1V
// #define ADC_VREF_MV       3100.0f
// #define ADC_MAX           4095.0f

// /* ================== ACS712 CONFIG ================= */
// // 5A  = 185 mV/A
// // 20A = 100 mV/A
// // 30A = 66  mV/A
// #define ACS_SENS_MV_PER_A 185.0f   // ACS712-20A

// void app_main(void)
// {
//     adc_oneshot_unit_handle_t adc;

//     /* ---------- Init ADC Unit ---------- */
//     adc_oneshot_unit_init_cfg_t unit_cfg = {
//         .unit_id = ACS_ADC_UNIT,
//     };
//     adc_oneshot_new_unit(&unit_cfg, &adc);

//     /* ---------- Config ADC Channel ---------- */
//     adc_oneshot_chan_cfg_t chan_cfg = {
//         .bitwidth = ADC_BITWIDTH_DEFAULT,
//         .atten    = ACS_ADC_ATTEN,
//     };
//     adc_oneshot_config_channel(adc, ACS_ADC_CHANNEL, &chan_cfg);

//     /* ---------- Calibrate Offset (NO CURRENT) ---------- */
//     int offset_raw = 0;

//     for (int i = 0; i < 100; i++) {
//         int raw = 0;
//         adc_oneshot_read(adc, ACS_ADC_CHANNEL, &raw);
//         offset_raw += raw;
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     offset_raw /= 100;
//     float offset_mv = offset_raw * ADC_VREF_MV / ADC_MAX;

//     ESP_LOGI(TAG,
//              "Offset raw=%d, offset_mv=%.2f mV",
//              offset_raw,
//              offset_mv);

//     /* ---------- Read Current ---------- */
//     while (1) {
//         int raw = 0;
//         adc_oneshot_read(adc, ACS_ADC_CHANNEL, &raw);

//         float mv = raw * ADC_VREF_MV / ADC_MAX;
//         float current_A = (mv - offset_mv) / ACS_SENS_MV_PER_A;

//         ESP_LOGI(TAG,
//                  "raw=%4d | V=%.1f mV | I=%.3f A",
//                  raw,
//                  mv,
//                  current_A);

//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }


// #include <stdio.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_adc/adc_oneshot.h"

// #define TAG "ACS712"

// /* ================== ADC CONFIG ================== */
// #define ACS_ADC_UNIT      ADC_UNIT_1
// #define ACS_ADC_CHANNEL   ADC_CHANNEL_3   // GPIO4 (ESP32-S3)
// #define ACS_ADC_ATTEN     ADC_ATTEN_DB_11
// #define ADC_VREF_MV       3100.0f
// #define ADC_MAX           4095.0f

// /* ================== ACS712T CONFIG ================== */
// // CHỌN CHẾ ĐỘ NGUỒN (chỉ chọn 1 trong 2):
// #define USE_3V3_MODE      1    // 1 = dùng 3.3V, 0 = dùng 5V

// #if USE_3V3_MODE
//     // ===== CHẾ ĐỘ 3.3V =====
//     #define OFFSET_RAW_FIXED      1900           // Fix cứng offset
//     #define ACS_SENS_MV_PER_A     125.0f         // Sensitivity @ 3.3V
//     #define DEADZONE_CURRENT      0.2f           // < 0.2A = 0A
// #else
//     // ===== CHẾ ĐỘ 5V =====
//     #define OFFSET_RAW_FIXED      2100           // Fix cứng offset
//     #define ACS_SENS_MV_PER_A     185.0f         // Sensitivity @ 5V
//     #define DEADZONE_CURRENT      0.15f          // < 0.15A = 0A
// #endif

// // Tính offset voltage
// #define OFFSET_MV_FIXED   ((OFFSET_RAW_FIXED * ADC_VREF_MV) / ADC_MAX)

// /* ================== BỘ LỌC TRUNG BÌNH ================== */
// float read_current_rms(adc_oneshot_unit_handle_t adc, int samples)
// {
//     float sum_i2 = 0.0f;
//     int valid_samples = 0;
    
//     for (int i = 0; i < samples; i++) {
//         int raw;
//         adc_oneshot_read(adc, ACS_ADC_CHANNEL, &raw);
        
//         // Bỏ qua mẫu bão hòa
//         if (raw <= 10 || raw >= 4000) {
//             continue;
//         }
        
//         // Tính dòng điện
//         float mv = raw * ADC_VREF_MV / ADC_MAX;
//         float delta_mv = mv - OFFSET_MV_FIXED;
//         float current = delta_mv / ACS_SENS_MV_PER_A;
        
//         sum_i2 += current * current;
//         valid_samples++;
        
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
    
//     // Không đủ mẫu hợp lệ
//     if (valid_samples < 10) {
//         return 0.0f;
//     }
    
//     // Tính RMS
//     float rms = sqrtf(sum_i2 / valid_samples);
    
//     // Dead-zone: loại bỏ nhiễu nhỏ
//     if (rms < DEADZONE_CURRENT) {
//         return 0.0f;
//     }
    
//     return rms;
// }

// /* ================== KIỂM TRA PHẦN CỨNG ================== */
// void check_hardware(adc_oneshot_unit_handle_t adc)
// {
//     ESP_LOGI(TAG, "");
//     ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
//     ESP_LOGI(TAG, "║  KIỂM TRA PHẦN CỨNG                  ║");
//     ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
//     // Đọc nhiều mẫu để lấy trung bình
//     int sum_raw = 0;
//     for (int i = 0; i < 50; i++) {
//         int raw;
//         adc_oneshot_read(adc, ACS_ADC_CHANNEL, &raw);
//         sum_raw += raw;
//         vTaskDelay(pdMS_TO_TICKS(2));
//     }
    
//     int avg_raw = sum_raw / 50;
//     float avg_mv = avg_raw * ADC_VREF_MV / ADC_MAX;
    
//     ESP_LOGI(TAG, "ADC hiện tại: %d (%.1f mV)", avg_raw, avg_mv);
//     ESP_LOGI(TAG, "Offset cố định: %d (%.1f mV)", OFFSET_RAW_FIXED, OFFSET_MV_FIXED);
    
//     // Kiểm tra xem có phù hợp không
//     int diff = abs(avg_raw - OFFSET_RAW_FIXED);
    
//     if (diff < 200) {
//         ESP_LOGI(TAG, "✅ Offset phù hợp (chênh lệch: %d)", diff);
//     } else if (diff < 500) {
//         ESP_LOGW(TAG, "⚠️  Offset hơi lệch (chênh lệch: %d)", diff);
//         ESP_LOGW(TAG, "   Có thể điều chỉnh OFFSET_RAW_FIXED = %d", avg_raw);
//     } else {
//         ESP_LOGE(TAG, "❌ Offset lệch quá nhiều (chênh lệch: %d)", diff);
        
// #if USE_3V3_MODE
//         ESP_LOGE(TAG, "   Nguyên nhân có thể:");
//         ESP_LOGE(TAG, "   • ACS712 được cấp 5V thay vì 3.3V");
//         ESP_LOGE(TAG, "   • Dây OUT không nối đúng GPIO4");
//         ESP_LOGE(TAG, "   • Nhiễu điện từ quá mạnh");
// #else
//         ESP_LOGE(TAG, "   Nguyên nhân có thể:");
//         ESP_LOGE(TAG, "   • ACS712 được cấp 3.3V thay vì 5V");
//         ESP_LOGE(TAG, "   • Dây OUT không nối đúng GPIO4");
//         ESP_LOGE(TAG, "   • ACS712 bị hỏng");
// #endif
//     }
    
//     ESP_LOGI(TAG, "");
// }

// /* ================== MAIN ================== */
// void app_main(void)
// {
//     ESP_LOGI(TAG, "");
//     ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
//     ESP_LOGI(TAG, "║  ACS712T ELC-05B FIXED OFFSET        ║");
//     ESP_LOGI(TAG, "║  ESP32-S3 | GPIO4 | No Calibration   ║");
//     ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
// #if USE_3V3_MODE
//     ESP_LOGI(TAG, "⚙️  Chế độ: 3.3V");
//     ESP_LOGI(TAG, "   • Offset: %d RAW (%.1f mV)", OFFSET_RAW_FIXED, OFFSET_MV_FIXED);
//     ESP_LOGI(TAG, "   • Sensitivity: %.0f mV/A", ACS_SENS_MV_PER_A);
//     ESP_LOGI(TAG, "   • Dead-zone: %.2f A", DEADZONE_CURRENT);
// #else
//     ESP_LOGI(TAG, "⚙️  Chế độ: 5.0V");
//     ESP_LOGI(TAG, "   • Offset: %d RAW (%.1f mV)", OFFSET_RAW_FIXED, OFFSET_MV_FIXED);
//     ESP_LOGI(TAG, "   • Sensitivity: %.0f mV/A", ACS_SENS_MV_PER_A);
//     ESP_LOGI(TAG, "   • Dead-zone: %.2f A", DEADZONE_CURRENT);
// #endif
    
//     ESP_LOGI(TAG, "");
    
//     /* ---------- Init ADC ---------- */
//     adc_oneshot_unit_handle_t adc;
    
//     adc_oneshot_unit_init_cfg_t unit_cfg = {
//         .unit_id = ACS_ADC_UNIT,
//     };
//     adc_oneshot_new_unit(&unit_cfg, &adc);
    
//     adc_oneshot_chan_cfg_t chan_cfg = {
//         .bitwidth = ADC_BITWIDTH_DEFAULT,
//         .atten = ACS_ADC_ATTEN,
//     };
//     adc_oneshot_config_channel(adc, ACS_ADC_CHANNEL, &chan_cfg);
    
//     /* ---------- Kiểm tra phần cứng ---------- */
//     check_hardware(adc);
    
//     /* ---------- Bắt đầu đo ---------- */
//     ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
//     ESP_LOGI(TAG, "║  BẮT ĐẦU ĐO DÒNG ĐIỆN                ║");
//     ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
//     ESP_LOGI(TAG, "");
    
//     uint32_t count = 0;
    
//     while (1) {
//         // Đo dòng điện RMS
//         float current = read_current_rms(adc, 120);
        
//         // Đọc ADC để debug (chỉ khi có dòng)
//         if (current > 0.05f) {
//             int raw;
//             adc_oneshot_read(adc, ACS_ADC_CHANNEL, &raw);
//             float mv = raw * ADC_VREF_MV / ADC_MAX;
            
//             ESP_LOGI(TAG, "⚡ I = %.3f A  [RAW=%d, %.1fmV, Δ=%.1fmV]", 
//                      current, raw, mv, mv - OFFSET_MV_FIXED);
//         } else {
//             // Chỉ hiển thị 0A mỗi 10 lần
//             if (count % 10 == 0) {
//                 int raw;
//                 adc_oneshot_read(adc, ACS_ADC_CHANNEL, &raw);
//                 float mv = raw * ADC_VREF_MV / ADC_MAX;
                
//                 ESP_LOGI(TAG, "💤 I = 0.000 A  [RAW=%d, %.1fmV]", raw, mv);
//             }
//         }
        
//         count++;
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }