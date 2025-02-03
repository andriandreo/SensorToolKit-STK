/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define CONFIG_MQTT_PROTOCOL_5 // Pasar si no como DEFAULT via `idf.py menuconfig` [!!!]

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"

/* 
    This code uses MQTT configuration that you can set 
    via project configuration menu `idf.py menuconfig`.

    If you'd rather not, just change the below entries to strings with
    the config you want - i.e. `#define ESP_MQTT_USER "myuser"`
*/
#define ESP_MQTT_USER      CONFIG_ESP_MQTT_USER
#define ESP_MQTT_PASS      CONFIG_ESP_MQTT_PASSWORD

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

void log_error_if_nonzero(const char *message, int error_code);

void print_user_property(mqtt5_user_property_handle_t user_property);

void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

void esp_initMQTT(void);

void esp_sendMQTT(char *topic, char *payload);

/******************************************************************************
******************************************************************************/
