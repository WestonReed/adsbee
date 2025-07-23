#include <string.h>

#include <functional>  // for std::bind

#include "cc.h"  // For endiannness swapping.
#include "comms.hh"
#include "esp_event.h"
#include "esp_mac.h"
#include "hal.hh"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "task_priorities.hh"
#include "utils/task_utils.hh"  // For delayed reconnect callbacks.

static const uint16_t kWiFiNumRetries = 3;
static const uint16_t kWiFiRetryWaitTimeMs = 100;
static const uint16_t kWiFiStaMaxNumReconnectAttempts = 5;
static const uint16_t kWiFiScanDefaultListSize = 20;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/** "Pass-Through" functions used to access member functions in callbacks. **/
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    comms_manager.WiFiEventHandler(arg, event_base, event_id, event_data);
}

void wifi_access_point_task(void* pvParameters) { comms_manager.WiFiAccessPointTask(pvParameters); }
void foreflight_discovery_listener_task(void* pvParameters) { comms_manager.ForeFlightDiscoveryListenerTask(pvParameters); }
void foreflight_client_sender_task(void* pvParameters) { comms_manager.ForeFlightClientSenderTask(pvParameters); }
inline void connect_to_wifi(void* arg = nullptr) {
    if (esp_wifi_connect() != ESP_OK) {
        CONSOLE_ERROR("connect_to_wifi", "Failed to connect to WiFi.");
    }
}
/** End "Pass-Through" functions. **/

void CommsManager::WiFiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    switch (event_id) {
        case WIFI_EVENT_AP_STACONNECTED: {
            // A new station has connected to the ADSBee's softAP network.
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
            CONSOLE_INFO("CommsManager::WiFiEventHandler", "Station " MACSTR " joined, AID=%d", MAC2STR(event->mac),
                         event->aid);
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            // A station has disconnected from the ADSBee's softAP network.
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
            CONSOLE_INFO("CommsManager::WiFiEventHandler", "Station " MACSTR " left, AID=%d", MAC2STR(event->mac),
                         event->aid);
            WiFiRemoveClient(event->mac);
            break;
        }
        case WIFI_EVENT_STA_START: {
            // The ADSBee is attempting to connect to an external network.
            char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen];
            SettingsManager::RedactPassword(wifi_sta_password, redacted_password,
                                            SettingsManager::Settings::kWiFiPasswordMaxLen);
            CONSOLE_INFO("CommsManager::WiFiInit", "WiFi Station started. SSID:%s password:%s", wifi_sta_ssid,
                         redacted_password);
            ESP_ERROR_CHECK(esp_wifi_connect());
            // Note: wifi_sta_has_ip_ will get filled in by the IP event handler if an IP is issued.
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            // The ADSBee has disconnected from an external network.
            wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
            CONSOLE_ERROR("CommsManager::WiFiEventHandler",
                          "Disconnected from (or failed to connect to) ap SSID:%s password:%s - Disconnect reason : %d",
                          wifi_sta_ssid, wifi_sta_password, event->reason);
            wifi_sta_connected_ = false;
            wifi_sta_has_ip_ = false;
            if (wifi_sta_enabled) {
                ScheduleDelayedFunctionCall(kWiFiStaReconnectIntervalMs, &connect_to_wifi);
            }
            break;
        }
        case WIFI_EVENT_STA_CONNECTED:
            char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen];
            SettingsManager::RedactPassword(wifi_sta_password, redacted_password,
                                            SettingsManager::Settings::kWiFiPasswordMaxLen);
            CONSOLE_INFO("CommsManager::WiFiInit", "Connected to ap SSID:%s password:%s", wifi_sta_ssid,
                         redacted_password);
            wifi_sta_connected_ = true;
            wifi_sta_connected_timestamp_ms_ = get_time_since_boot_ms();
            break;
    }
}

void CommsManager::WiFiAccessPointTask(void* pvParameters) {
    NetworkMessage message;

    // Create socket.
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        CONSOLE_ERROR("CommsManager::WiFiAccessPointTask", "Unable to create socket: errno %d", errno);
        return;
    }

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    while (true) {
        if (xQueueReceive(wifi_ap_message_queue_, &message, portMAX_DELAY) == pdTRUE) {
            struct sockaddr_in dest_addr;
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(message.port);

            xSemaphoreTake(wifi_clients_list_mutex_, portMAX_DELAY);
            for (int i = 0; i < SettingsManager::Settings::kWiFiMaxNumClients; i++) {
                if (wifi_clients_list_[i].active) {
                    dest_addr.sin_addr.s_addr = wifi_clients_list_[i].ip.addr;
                    int ret = 0;
                    uint16_t num_tries;
                    for (num_tries = 0; num_tries < kWiFiNumRetries; num_tries++) {
                        ret =
                            sendto(sock, message.data, message.len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                        // ENOMEM (errno=12) resolution: https://github.com/espressif/esp-idf/issues/390
                        // Increased the number of UDP control blocks (LWIP_MAX_UDP_PCBS) in SDK menuconfig
                        // from 16 to 96. Changed TCP/IP stack size from 3072 to 12288.
                        if (ret >= 0 || errno != ENOMEM) {
                            break;
                        }
                        vTaskDelay(kWiFiRetryWaitTimeMs /
                                   portTICK_PERIOD_MS);  // Let packet send to avoid an ENOMEM error.
                    }

                    if (ret < 0) {
                        // CONSOLE_ERROR("CommsManager::WiFiAccessPointTask", "Error occurred during sending:
                        // errno %d.", errno);
                        CONSOLE_ERROR("CommsManager::WiFiAccessPointTask",
                                      "Error occurred during sending: errno %d. Tried %d times.", errno, num_tries);
                    }
                }
            }
            xSemaphoreGive(wifi_clients_list_mutex_);

            // CONSOLE_INFO("CommsManager::WiFiUDPServerTask", "Message sent to %d clients.",
            // num_wifi_clients_);
        }
    }
    shutdown(sock, 0);
    close(sock);
}

bool CommsManager::WiFiInit() {
    esp_netif_t* wifi_ap_netif_ = esp_netif_create_default_wifi_ap();
    assert(wifi_ap_netif_);
    esp_netif_t* wifi_sta_netif_ = esp_netif_create_default_wifi_sta();
    assert(wifi_sta_netif_);

    ESP_ERROR_CHECK(esp_netif_set_hostname(wifi_sta_netif_, hostname));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    if (!ip_event_handler_was_initialized_) {
        IPInit();
    }

    wifi_mode_t wifi_mode;
    if (wifi_ap_enabled && wifi_sta_enabled) {
        wifi_mode = WIFI_MODE_APSTA;
    } else if (wifi_ap_enabled) {
        wifi_mode = WIFI_MODE_AP;
    } else {
        wifi_mode = WIFI_MODE_STA;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode));

    wifi_was_initialized_ = true;

    if (wifi_ap_enabled) {
        // Access Point Configuration
        wifi_config_t wifi_config_ap = {0};

        strncpy((char*)(wifi_config_ap.ap.ssid), wifi_ap_ssid, SettingsManager::Settings::kWiFiSSIDMaxLen + 1);
        strncpy((char*)(wifi_config_ap.ap.password), wifi_ap_password,
                SettingsManager::Settings::kWiFiPasswordMaxLen + 1);
        wifi_config_ap.ap.channel = wifi_ap_channel;
        wifi_config_ap.ap.ssid_len = (uint8_t)strlen(wifi_ap_ssid);
        if (strlen(wifi_ap_password) == 0) {
            wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
        } else {
            wifi_config_ap.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        }
        wifi_config_ap.ap.max_connection = SettingsManager::Settings::kWiFiMaxNumClients;

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));
    }

    if (wifi_sta_enabled) {
        // Station Configuration
        wifi_config_t wifi_config_sta = {0};

        strncpy((char*)(wifi_config_sta.sta.ssid), wifi_sta_ssid, SettingsManager::Settings::kWiFiSSIDMaxLen + 1);
        strncpy((char*)(wifi_config_sta.sta.password), wifi_sta_password,
                SettingsManager::Settings::kWiFiPasswordMaxLen + 1);

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta));
    }

    if (!wifi_ap_enabled && !wifi_sta_enabled) {
        ESP_ERROR_CHECK(esp_wifi_stop());
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi disabled.");
        return true;
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    if (wifi_ap_enabled) {
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi AP started. SSID:%s password:%s", wifi_ap_ssid, wifi_ap_password);
        xTaskCreatePinnedToCore(wifi_access_point_task, "wifi_ap_task", 4096, &wifi_ap_task_handle, kWiFiAPTaskPriority,
                                NULL, kWiFiAPTaskCore);
    }
    if (wifi_sta_enabled) {
        char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen];
        SettingsManager::RedactPassword(wifi_sta_password, redacted_password,
                                        SettingsManager::Settings::kWiFiPasswordMaxLen);
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi Station started. SSID:%s password:%s", wifi_sta_ssid,
                     redacted_password);
    }

    // Start ForeFlight discovery and client management tasks when WiFi is enabled
    if (wifi_ap_enabled || wifi_sta_enabled) {
        xTaskCreatePinnedToCore(foreflight_discovery_listener_task, "foreflight_discovery", 4096, 
                                &foreflight_discovery_task_handle, kWiFiAPTaskPriority, NULL, kWiFiAPTaskCore);
        xTaskCreatePinnedToCore(foreflight_client_sender_task, "foreflight_sender", 4096, 
                                &foreflight_sender_task_handle, kWiFiAPTaskPriority, NULL, kWiFiAPTaskCore);
        CONSOLE_INFO("CommsManager::WiFiInit", "ForeFlight discovery and client sender tasks started");
    }

    return true;
}

bool CommsManager::WiFiDeInit() {
    if (!wifi_was_initialized_) return true;  // Don't try de-initializing if it was never initialized.

    // The de-init functions are not yet supported by ESP IDF, so the best bet is to just restart.
    esp_restart();  // Software reset.
    return false;   // abort didn't work
}

bool CommsManager::IPWANSendDecoded1090Packet(Decoded1090Packet& decoded_packet) {
    if (!wifi_sta_has_ip_ && !ethernet_has_ip_) {
        CONSOLE_WARNING(
            "CommsManager::IPWANSendDecoded1090Packet",
            "Can't push to WAN transponder packet queue if WiFi station is not running and Ethernet is disconnected.");
        return false;  // Task not started yet, queue not created yet. Pushing to queue would cause an abort.
    }
    int err = xQueueSend(ip_wan_decoded_transponder_packet_queue_, &decoded_packet, 0);
    if (err == errQUEUE_FULL) {
        CONSOLE_WARNING("CommsManager::IPWANSendDecoded1090Packet", "Overflowed WAN transponder packet queue.");
        xQueueReset(ip_wan_decoded_transponder_packet_queue_);
        return false;
    } else if (err != pdTRUE) {
        CONSOLE_WARNING("CommsManager::IPWANSendDecoded1090Packet",
                        "Pushing transponder packet to WAN queue resulted in error code %d.", err);
        return false;
    }
    return true;
}

// Helper function to parse ForeFlight discovery message
bool CommsManager::ParseForeFlightDiscoveryMessage(const char* message, uint16_t* gdl90_port) {
    // Simple parser for: { "App":"ForeFlight", "GDL90":{ "port":4000 } }
    if (strstr(message, "\"App\":\"ForeFlight\"") == nullptr) {
        return false;  // Not a ForeFlight message
    }
    
    // Extract GDL90 port if present
    const char* port_start = strstr(message, "\"port\":");
    if (port_start != nullptr) {
        port_start += 7;  // Skip "port":
        while (*port_start == ' ' || *port_start == '\t') port_start++;  // Skip whitespace
        *gdl90_port = (uint16_t)strtoul(port_start, nullptr, 10);
    } else {
        *gdl90_port = 4000;  // Default GDL90 port
    }
    
    return true;
}

// Add or update a ForeFlight client in the list
void CommsManager::ForeFlightAddOrUpdateClient(esp_ip4_addr_t client_ip, uint16_t gdl90_port) {
    uint32_t current_time_ms = get_time_since_boot_ms();
    
    xSemaphoreTake(foreflight_clients_list_mutex_, portMAX_DELAY);
    
    // First check if client already exists (update timestamp)
    for (int i = 0; i < kForeFlightMaxNumClients; i++) {
        if (foreflight_clients_list_[i].active && 
            foreflight_clients_list_[i].ip.addr == client_ip.addr) {
            foreflight_clients_list_[i].last_seen_timestamp_ms = current_time_ms;
            foreflight_clients_list_[i].gdl90_port = gdl90_port;
            xSemaphoreGive(foreflight_clients_list_mutex_);
            return;
        }
    }
    
    // Client doesn't exist, find an empty slot
    for (int i = 0; i < kForeFlightMaxNumClients; i++) {
        if (!foreflight_clients_list_[i].active) {
            foreflight_clients_list_[i].ip = client_ip;
            foreflight_clients_list_[i].last_seen_timestamp_ms = current_time_ms;
            foreflight_clients_list_[i].gdl90_port = gdl90_port;
            foreflight_clients_list_[i].active = true;
            num_foreflight_clients_++;
            char client_ip_str[SettingsManager::Settings::kIPAddrStrLen + 1];
            snprintf(client_ip_str, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&client_ip));
            CONSOLE_INFO("CommsManager::ForeFlightAddOrUpdateClient", 
                        "Added ForeFlight client %s, GDL90 port %d", client_ip_str, gdl90_port);
            break;
        }
    }
    
    xSemaphoreGive(foreflight_clients_list_mutex_);
}

// Remove expired ForeFlight clients
void CommsManager::ForeFlightRemoveExpiredClients() {
    uint32_t current_time_ms = get_time_since_boot_ms();
    
    xSemaphoreTake(foreflight_clients_list_mutex_, portMAX_DELAY);
    
    for (int i = 0; i < kForeFlightMaxNumClients; i++) {
        if (foreflight_clients_list_[i].active && 
            (current_time_ms - foreflight_clients_list_[i].last_seen_timestamp_ms) > kForeFlightClientTimeoutMs) {
            char client_ip_str[SettingsManager::Settings::kIPAddrStrLen + 1];
            snprintf(client_ip_str, SettingsManager::Settings::kIPAddrStrLen, IPSTR, 
                    IP2STR(&foreflight_clients_list_[i].ip));
            CONSOLE_INFO("CommsManager::ForeFlightRemoveExpiredClients", 
                        "Removed expired ForeFlight client %s", client_ip_str);
            foreflight_clients_list_[i].active = false;
            num_foreflight_clients_--;
        }
    }
    
    xSemaphoreGive(foreflight_clients_list_mutex_);
}

void CommsManager::ForeFlightDiscoveryListenerTask(void* pvParameters) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char rx_buffer[256];
    
    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        CONSOLE_ERROR("CommsManager::ForeFlightDiscoveryListenerTask", "Unable to create socket: errno %d", errno);
        return;
    }
    
    // Set socket to reuse address
    int enable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        CONSOLE_ERROR("CommsManager::ForeFlightDiscoveryListenerTask", "setsockopt(SO_REUSEADDR) failed: errno %d", errno);
    }
    
    // Bind to ForeFlight discovery port
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(kForeFlightDiscoveryPort);
    
    if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        CONSOLE_ERROR("CommsManager::ForeFlightDiscoveryListenerTask", "Socket bind failed: errno %d", errno);
        close(sock);
        return;
    }
    
    CONSOLE_INFO("CommsManager::ForeFlightDiscoveryListenerTask", "Listening for ForeFlight broadcasts on port %d", kForeFlightDiscoveryPort);
    
    // Set receive timeout
    struct timeval timeout;
    timeout.tv_sec = 5;  // 5 second timeout for cleanup
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    while (true) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, 
                          (struct sockaddr*)&client_addr, &client_addr_len);
        
        if (len > 0) {
            rx_buffer[len] = '\0';  // Null terminate
            
            uint16_t gdl90_port;
            if (ParseForeFlightDiscoveryMessage(rx_buffer, &gdl90_port)) {
                esp_ip4_addr_t client_ip;
                client_ip.addr = client_addr.sin_addr.s_addr;
                ForeFlightAddOrUpdateClient(client_ip, gdl90_port);
            }
        } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            // Real error occurred
            CONSOLE_ERROR("CommsManager::ForeFlightDiscoveryListenerTask", "recvfrom failed: errno %d", errno);
        }
        
        // Periodically clean up expired clients
        static uint32_t last_cleanup_time_ms = 0;
        uint32_t current_time_ms = get_time_since_boot_ms();
        if (current_time_ms - last_cleanup_time_ms > 10000) {  // Every 10 seconds
            ForeFlightRemoveExpiredClients();
            last_cleanup_time_ms = current_time_ms;
        }
    }
    
    close(sock);
}

void CommsManager::ForeFlightClientSenderTask(void* pvParameters) {
    NetworkMessage message;
    
    // Create socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        CONSOLE_ERROR("CommsManager::ForeFlightClientSenderTask", "Unable to create socket: errno %d", errno);
        return;
    }
    
    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
    
    while (true) {
        if (xQueueReceive(foreflight_client_message_queue_, &message, portMAX_DELAY) == pdTRUE) {
            struct sockaddr_in dest_addr;
            dest_addr.sin_family = AF_INET;
            
            xSemaphoreTake(foreflight_clients_list_mutex_, portMAX_DELAY);
            for (int i = 0; i < kForeFlightMaxNumClients; i++) {
                if (foreflight_clients_list_[i].active) {
                    dest_addr.sin_addr.s_addr = foreflight_clients_list_[i].ip.addr;
                    dest_addr.sin_port = htons(foreflight_clients_list_[i].gdl90_port);
                    
                    int ret = 0;
                    uint16_t num_tries;
                    for (num_tries = 0; num_tries < kWiFiNumRetries; num_tries++) {
                        ret = sendto(sock, message.data, message.len, 0, 
                                   (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                        if (ret >= 0 || errno != ENOMEM) {
                            break;
                        }
                        vTaskDelay(kWiFiRetryWaitTimeMs / portTICK_PERIOD_MS);
                    }
                    
                    if (ret < 0) {
                        CONSOLE_ERROR("CommsManager::ForeFlightClientSenderTask",
                                     "Error occurred during sending: errno %d. Tried %d times.", errno, num_tries);
                    }
                }
            }
            xSemaphoreGive(foreflight_clients_list_mutex_);
        }
    }
    shutdown(sock, 0);
    close(sock);
}

bool CommsManager::WiFiClientSendMessageToAllClients(NetworkMessage& message) {
    if (num_foreflight_clients_ == 0) {
        CONSOLE_WARNING("CommsManager::WiFiClientSendMessageToAllClients",
                        "No ForeFlight clients discovered yet.");
        return false;
    }
    
    int err = xQueueSend(foreflight_client_message_queue_, &message, 0);
    if (err == errQUEUE_FULL) {
        CONSOLE_WARNING("CommsManager::WiFiClientSendMessageToAllClients", "Overflowed ForeFlight client message queue.");
        xQueueReset(foreflight_client_message_queue_);
        return false;
    } else if (err != pdTRUE) {
        CONSOLE_WARNING("CommsManager::WiFiClientSendMessageToAllClients",
                        "Pushing message to ForeFlight client message queue resulted in error code %d.", err);
        return false;
    }
    return true;
}

bool CommsManager::WiFiAccessPointSendMessageToAllStations(NetworkMessage& message) {
    if (!wifi_ap_enabled) {
        CONSOLE_WARNING("CommsManager::WiFiAccessPointSendMessageToAllStations",
                        "Can't push to WiFi AP message queue if AP is not running.");
        return false;  // Task not started yet, pushing to queue could create an overflow.
    }
    int err = xQueueSend(wifi_ap_message_queue_, &message, 0);
    if (err == errQUEUE_FULL) {
        CONSOLE_WARNING("CommsManager::WiFiAccessPointSendMessageToAllStations", "Overflowed WiFi AP message queue.");
        xQueueReset(wifi_ap_message_queue_);
        return false;
    } else if (err != pdTRUE) {
        CONSOLE_WARNING("CommsManager::WiFiAccessPointSendMessageToAllStations",
                        "Pushing message to WiFi AP message queue resulted in error code %d.", err);
        return false;
    }
    return true;
}