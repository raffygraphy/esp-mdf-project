// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mdf_common.h"
#include "mwifi.h"
#include "driver/gpio.h"

#define GPIO_INPUT_IO_15    15
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_15)

#define GPIO_OUTPUT_IO_2    2
#define GPIO_OUTPUT_PIN_SEL_2  (1ULL<<GPIO_OUTPUT_IO_2)


#define MACSTR_LEN 18 

#define GPIO_OUTPUT_IO_12    12
#define GPIO_OUTPUT_PIN_SEL_12  (1ULL<<GPIO_OUTPUT_IO_12)

#define GPIO_OUTPUT_IO_4    4
#define GPIO_OUTPUT_PIN_SEL_4  (1ULL<<GPIO_OUTPUT_IO_4)


#define GPIO_OUTPUT_IO_14    14
#define GPIO_OUTPUT_PIN_SEL_14  (1ULL<<GPIO_OUTPUT_IO_14)

#define GPIO_OUTPUT_IO_16    16
#define GPIO_OUTPUT_PIN_SEL_16  (1ULL<<GPIO_OUTPUT_IO_16)



// #define MEMORY_DEBUG

static const char *TAG = "get_started";

static void read_gpio(int *value)
{
    *value = gpio_get_level(GPIO_INPUT_IO_15);
}

static void root_task(void *arg)
{
    mdf_err_t ret                    = MDF_OK;
    char *data                       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                      = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0};

    char connected_node_str[MACSTR_LEN];

    MDF_LOGI("Root is running");

    gpio_config_t io_conf_output;
    io_conf_output.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf_output.mode = GPIO_MODE_OUTPUT;
    io_conf_output.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_2;
    io_conf_output.pull_up_en = 0;
    io_conf_output.pull_down_en = 0;
    gpio_config(&io_conf_output);

    gpio_config_t io_conf_output12;
    io_conf_output12.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf_output12.mode = GPIO_MODE_OUTPUT;
    io_conf_output12.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_12;
    io_conf_output12.pull_up_en = 0;
    io_conf_output12.pull_down_en = 0;
    gpio_config(&io_conf_output12);

    gpio_config_t io_conf_output4;
    io_conf_output4.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf_output4.mode = GPIO_MODE_OUTPUT;
    io_conf_output4.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_4;
    io_conf_output4.pull_up_en = 0;
    io_conf_output4.pull_down_en = 0;
    gpio_config(&io_conf_output4);

    gpio_config_t io_conf_output14;
    io_conf_output14.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf_output14.mode = GPIO_MODE_OUTPUT;
    io_conf_output14.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_14;
    io_conf_output14.pull_up_en = 0;
    io_conf_output14.pull_down_en = 0;
    gpio_config(&io_conf_output14);
    
    
    gpio_config_t io_conf_output16;
    io_conf_output16.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf_output16.mode = GPIO_MODE_OUTPUT;
    io_conf_output16.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_16;
    io_conf_output16.pull_up_en = 0;
    io_conf_output16.pull_down_en = 0;
    gpio_config(&io_conf_output16);


    for (int i = 0;; ++i) {
        if (!mwifi_is_started()) {
            MDF_LOGI("No node connected, setting all GPIO to 0");

            // Set all GPIO to 0
            gpio_set_level(GPIO_OUTPUT_IO_2, 0);
            gpio_set_level(GPIO_OUTPUT_IO_12, 0);
            gpio_set_level(GPIO_OUTPUT_IO_4, 0);
            gpio_set_level(GPIO_OUTPUT_IO_14, 0);
            gpio_set_level(GPIO_OUTPUT_IO_16, 0);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_root_read(src_addr, &data_type, data, &size, 0);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        MDF_LOGI("Root receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);


        char src_addr_str[18];
        snprintf(src_addr_str, sizeof(src_addr_str), MACSTR, MAC2STR(src_addr));
        if(strcmp(src_addr_str, "a4:cf:12:75:01:f4") == 0)
            (strcmp(data, "On") == 0) ? gpio_set_level(GPIO_OUTPUT_IO_4, 1) : gpio_set_level(GPIO_OUTPUT_IO_4, 0);
        if(strcmp(src_addr_str, "24:6f:28:ab:1d:d4") == 0) 
            (strcmp(data, "On") == 0) ? gpio_set_level(GPIO_OUTPUT_IO_16, 1) : gpio_set_level(GPIO_OUTPUT_IO_16, 0);
        

        size = sprintf(data, "(%d) Hello node!", i);
        ret = mwifi_root_write(src_addr, 1, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_root_recv, ret: %x", ret);
        MDF_LOGI("Root send, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

        mesh_addr_t parent_bssid = {0};
        esp_mesh_get_parent_bssid(&parent_bssid);
        
        MDF_LOGI("Connected Child Nodes HELLO THIS IS ME CONNECTED:");
        snprintf(connected_node_str, sizeof(connected_node_str), MACSTR, MAC2STR(src_addr));
        MDF_LOGI("%s", connected_node_str);


        if (strcmp(connected_node_str, "a4:cf:12:75:01:f4") == 0) {
            MDF_LOGI("CHILD 1 Connected");
            gpio_set_level(GPIO_OUTPUT_IO_12, 1);

        } else {
            MDF_LOGI("CHILD 1 Not Connected");
            gpio_set_level(GPIO_OUTPUT_IO_12, 0);
        }

        if (strcmp(connected_node_str, "24:6f:28:ab:1d:d4") == 0) {
            MDF_LOGI("CHILD 2 Connected");
            gpio_set_level(GPIO_OUTPUT_IO_14, 1);

        } else {
            MDF_LOGI("CHILD 2 Not Connected");
            gpio_set_level(GPIO_OUTPUT_IO_14, 0);
        }

        // MDF_LOGI(MACSTR, MAC2STR(src_addr));
        wifi_sta_list_t wifi_sta_list = {0x0};
        esp_wifi_ap_get_sta_list(&wifi_sta_list);
    }

    MDF_LOGW("Root is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}


static void node_read_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data    = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size   = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type      = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    int gpio_value = 0;

    // Configuration for GPIO15
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    MDF_LOGI("Node read task is running");

    for (;;) {
        if (!mwifi_is_connected()) {
            continue;
        }

        read_gpio(&gpio_value);

        // Display the state of GPIO15 on the monitor
        MDF_LOGI("GPIO15 State: %s", gpio_value ? "Off" : "On");

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);

        if (gpio_value == 1) {
            size = sprintf(data, "Off");
        } else {
            size = sprintf(data, "On");
        }

        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_write, ret: %x", ret);

    }

    MDF_LOGW("Note read task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

void node_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    int count     = 0;
    size_t size   = 0;
    char *data    = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    mwifi_data_type_t data_type = {0x0};

    MDF_LOGI("Node write task is running");

    for (;;) {
        if (!mwifi_is_connected()) {
            continue;
        }

        size = sprintf(data, "(%d) Hello root!", count++);
        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_write, ret: %x", ret);

    }

    MDF_LOGW("Node write task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(void *timer)
{
    uint8_t primary                 = 0;
    wifi_second_chan_t second       = 0;
    mesh_addr_t parent_bssid        = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    wifi_sta_list_t wifi_sta_list   = {0x0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_mesh_get_parent_bssid(&parent_bssid);

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u", primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mwifi_get_parent_rssi(), esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

#ifdef MEMORY_DEBUG

    if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
#endif /**< MEMORY_DEBUG */
}

static mdf_err_t wifi_init()
{
    mdf_err_t ret          = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default());
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            break;

        default:
            break;
    }

    return MDF_OK;
}

void app_main()
{
    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config   = {
        .channel   = CONFIG_MESH_CHANNEL,
        .mesh_id   = CONFIG_MESH_ID,
        .mesh_type = CONFIG_DEVICE_TYPE,
    };

    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    /**
     * @brief Data transfer between wifi mesh devices
     */
    if (config.mesh_type == MESH_ROOT) {
        xTaskCreate(root_task, "root_task", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    } else {
        xTaskCreate(node_write_task, "node_write_task", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
        xTaskCreate(node_read_task, "node_read_task", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    }

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}
