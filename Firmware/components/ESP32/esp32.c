/******************************************************************************
 * @file       esp32.c
 * @brief      (Generic) ESP32 board implementation code needed for port file.
 *             Source file.
 * 
 * @author     Andrés Alberto Andreo Acosta
 * @version    V2.0.0
 * @date       January 2025
 * 
 * @par        Revision History:
 *              * Added support for other ESP32 boards.
 *              * Implemented Wi-Fi and MQTT functionality.
 *              * Checked for redundant includes.
 * 
 * @todo       * Implement Bluetooth functionality.
 *             * Re-implement SPI transactions to use tx/rx data instead of buffers.
 *             * Analyse the need for SPI lazy init.
 *             * Implement UART-INTR functionality if needed.
 * 
 * 
 * It is intended to include all the declarations, definitions, macros and
 * functions needed for the port file of the ESP32 board family as suggested
 * by Analog Devices, Inc. in the AD594x porting guide.
 *              
 * ****************************************************************************
 * This software is built in collaboration with JLM Innovation GmbH.
 * 
 * It is intended for use in conjunction of the ESP-IDF framework.
 * ----------------------------------------------------------------------------
 * @license: http://www.apache.org/licenses/LICENSE-2.0 
 *           (accessed on January 31, 2025)
 * 
 * Copyright (c) 2025 Andrés Alberto
 * 
 * Based on ESP-IDF examples (Apache 2.0 Licensed) from:
 * - `wifi/getting-started/station`
 * - `protocols/mqtt5`
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
******************************************************************************/

// Include guard
#ifndef ESP32_C
#define ESP32_C

// Include dependencies
#include "esp32.h"

static const char TAG[] = "esp32-AD594x";

/* Define the SPI handle, static to this source file */
static spi_device_handle_t spi = NULL;

spi_device_handle_t esp_getSPI(void) {
    if (spi == NULL) {
        esp_initSPI();  // Initialise the SPI if it not done yet
    }
    return spi;
}

void esp_initSPI(void){
    esp_err_t ret;
    ESP_LOGI(TAG, "[SPI] Initializing bus SPI%d...", AD594x_HOST+1); // '+1' as `SPI_HOST` is 0-based (SPI1=0, SPI2=1,...)
    if (spi != NULL) {
        // If SPI is already initialised, return early
        return;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
    };

    spi_device_interface_config_t devcfg = {
        .clock_source = SPI_CLK_SRC_DEFAULT,   // Cannot use XTAL as it is not connected to the ESP32, just to AFE
        .queue_size = 1,                       // No 'queing' is intended as per Datasheet
        .mode = 0,                             // SPI mode 0 (SCLK idles low, data clocked on SCLK falling edge)
        .clock_speed_hz = SPI_MASTER_FREQ_16M, // Clock out at 16 MHz
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(AD594x_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Attach the AFE to the SPI bus
    ret = spi_bus_add_device(AD594x_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void esp_spi_ReadWriteNBytes(uint8_t *pSendBuffer, uint8_t *pRecvBuff, unsigned long length){
    if (length == 0) return;           // No need to send anything

    spi_transaction_t t = {
        .length = length*8,            // `length` parameter in bytes, transaction `t.length` is in bits.
        .tx_buffer = pSendBuffer,      // TX Data to be sent (Write)
        .rx_buffer = pRecvBuff,        // RX Data to be received (Read)
        // TODO: could be more efficient to use tx and rx data instead of buffer | no need to alloc memory.
    };

    esp_err_t ret = spi_device_polling_transmit(esp_getSPI(), &t); // Transmit!
    assert(ret == ESP_OK); // Should have had no issues.
}

void esp_initGPIO(void){
    gpio_reset_pin(AD5940_CS_PIN);
    gpio_set_intr_type(AD5940_CS_PIN, GPIO_INTR_DISABLE);     // Disable interrupt (only enabled for GPIO0)
    gpio_set_direction(AD5940_CS_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode(AD5940_CS_PIN, GPIO_PULLUP_ONLY);

    gpio_reset_pin(AD5940_RST_PIN);
    gpio_set_intr_type(AD5940_RST_PIN, GPIO_INTR_DISABLE);    // Disable interrupt (only enabled for GPIO0)
    gpio_set_direction(AD5940_RST_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode(AD5940_RST_PIN, GPIO_PULLUP_ONLY);

    gpio_reset_pin(AD5940_GP0INT_PIN);
    gpio_set_direction(AD5940_GP0INT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(AD5940_GP0INT_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(AD5940_GP0INT_PIN, GPIO_INTR_NEGEDGE); // Interrupt on falling edge

    gpio_reset_pin(AD5940_GP1_PIN);
    gpio_set_direction(AD5940_GP1_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(AD5940_GP1_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(AD5940_GP1_PIN, GPIO_INTR_DISABLE);    // Disable interrupt (only enabled for GPIO0)
}

/******************************************************************************
*******************************************************************************
 * @brief Implementation of ESP32 Wi-Fi protocol and connection to network AP.
 *        Extracted and adapted from ESP-IDF examples: 
 *        `wifi/getting-started/station`.
 * 
*******************************************************************************
******************************************************************************/
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

/******************************************************************************
 * @brief Event handler registered to receive Wi-Fi events
 *
 *  This function is called by the Wi-Fi driver events loop.
 *
 * @param arg user data registered to the event.
 * @param event_base Event base for the handler(always WIFI Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_event_base_t.
 ******************************************************************************/
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "[WIFI] Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"[WIFI] Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "[WIFI] Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void esp_initWIFI(void){
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    //ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to DEPRECATED WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "[WIFI] `esp_initWIFI` finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "[WIFI] Connected to AP. SSID: %s, Passwd: %s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "[WIFI] Failed to connect to AP. SSID: %s, Passwd: %s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "[WIFI] UNEXPECTED EVENT");
    }
}

/******************************************************************************
*******************************************************************************
 * @brief Implementation of ESP32 MQTT protocol and connection to broker.
 *        Extracted and adapted from ESP-IDF examples: `protocols/mqtt5`.
 * 
*******************************************************************************
******************************************************************************/
esp_mqtt_client_handle_t client;

void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "[MQTT] Last error %s: 0x%x", message, error_code);
    }
}

static esp_mqtt5_user_property_item_t user_property_arr[] = {
        {"board", "esp32"},
        {"u", "user"},
        {"p", "password"}
    };

#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)

static esp_mqtt5_publish_property_config_t publish_property = {
    .payload_format_indicator = 1,
    .message_expiry_interval = 1000,
    .topic_alias = 0,
    .response_topic = "/topic/test/response",
    .correlation_data = "123456",
    .correlation_data_len = 6,
};

static esp_mqtt5_subscribe_property_config_t subscribe_property = {
    .subscribe_id = 25555,
    .no_local_flag = false,
    .retain_as_published_flag = false,
    .retain_handle = 0,
    .is_share_subscribe = true,
    .share_name = "group1",
};

static esp_mqtt5_subscribe_property_config_t subscribe1_property = {
    .subscribe_id = 25555,
    .no_local_flag = true,
    .retain_as_published_flag = false,
    .retain_handle = 0,
};

static esp_mqtt5_unsubscribe_property_config_t unsubscribe_property = {
    .is_share_subscribe = true,
    .share_name = "group1",
};

static esp_mqtt5_disconnect_property_config_t disconnect_property = {
    .session_expiry_interval = 60,
    .disconnect_reason = 0,
};

void print_user_property(mqtt5_user_property_handle_t user_property)
{
    if (user_property) {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count) {
            esp_mqtt5_user_property_item_t *item = malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK) {
                for (int i = 0; i < count; i ++) {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(TAG, "[MQTT] key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}
/******************************************************************************
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 ******************************************************************************/
void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "[MQTT] Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    ESP_LOGD(TAG, "Free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "payload_format_indicator is %d", event->property->payload_format_indicator);
        ESP_LOGI(TAG, "response_topic is %.*s", event->property->response_topic_len, event->property->response_topic);
        ESP_LOGI(TAG, "correlation_data is %.*s", event->property->correlation_data_len, event->property->correlation_data);
        ESP_LOGI(TAG, "content_type is %.*s", event->property->content_type_len, event->property->content_type);
        ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "MQTT5 return code is %d", event->error_handle->connect_return_code);
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "[MQTT] Other event id:%d", event->event_id);
        break;
    }
}

void esp_initMQTT(void){
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = "/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = false,
    };

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt5_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt5_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "[MQTT] Configuration mismatch: wrong Broker URL");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Set connection properties and user properties */
    esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_connect_property(client, &connect_property);

    /* If you call esp_mqtt5_client_set_user_property to set user properties, DO NOT forget to delete them.
     * esp_mqtt5_client_set_connect_property will malloc buffer to store the user_property and you can delete it after
     */
    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void esp_sendMQTT(char *topic, char *payload){
    esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
}

/******************************************************************************
******************************************************************************/

void esp_initLOG(void){
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // esp_log_level_set("*", ESP_LOG_INFO); // Normal log verbosity level
    esp_log_level_set("*", ESP_LOG_NONE); // Display no logs

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
}

#endif // ESP32_C