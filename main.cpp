#include <iostream>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

// Linux Bluetooth Headers
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

// --- CONFIGURATION ---
const char* TARGET_MAC = "48:87:2d:7c:f1:07"; // <--- Put your ONE beacon MAC here
float RSSI_AT_1M = -57.0f;
float N_VALUE = 2.0f;

// --- STATE ---
std::mutex stateMutex;
float current_distance = 0.0f;
int current_rssi = 0;
bool is_running = true;

// --- KALMAN FILTER ---
class PureKalman {
public:
    float e_mea = 2.0f, e_est = 2.0f, q = 1.2f, last_est = 0.0f;
    float update(float measurement) {
        float kalman_gain = e_est / (e_est + e_mea);
        float current_est = last_est + kalman_gain * (measurement - last_est);
        e_est = (1.0f - kalman_gain) * e_est + std::abs(last_est - current_est) * q;
        last_est = current_est;
        return current_est;
    }
};
PureKalman filter;

float rssiToMeters(int rssi) {
    if (rssi >= 0 || rssi < -120) return 0.0f; 
    return std::pow(10.0f, (RSSI_AT_1M - (float)rssi) / (10.0f * N_VALUE));
}

// --- BLUETOOTH CORE ---
void bluetoothSnifferThread() {
    int device_id = hci_get_route(NULL);
    int dd = hci_open_dev(device_id);
    
    if (device_id < 0 || dd < 0) {
        std::cerr << "CRITICAL: Could not open Bluetooth Device." << std::endl;
        return;
    }

    std::cout << "[BLE] Configuring Radio for Active Unfiltered Scanning..." << std::endl;

    // 1. Force the scanner off before configuring
    hci_le_set_scan_enable(dd, 0x00, 0x00, 1000); 

    // 2. Set scan parameters
    uint16_t interval = htobs(0x0010); // Very aggressive scan interval
    uint16_t window = htobs(0x0010);
    hci_le_set_scan_parameters(dd, 0x00, interval, window, 0x00, 0x00, 1000);

    // 3. Enable scanner WITH DUPLICATE FILTERING DISABLED (0x00)
    if (hci_le_set_scan_enable(dd, 0x01, 0x00, 1000) < 0) {
        std::cerr << "CRITICAL: Failed to enable hardware scanner. Is bluetoothctl running?" << std::endl;
        return;
    }

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    setsockopt(dd, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));

    uint8_t buf[HCI_MAX_FRAME_SIZE];
    std::cout << "[BLE] Success. Listening for MAC: " << TARGET_MAC << std::endl;

    while (true) {
        int len = read(dd, buf, sizeof(buf));
        if (len <= 0) continue;

        evt_le_meta_event* meta = (evt_le_meta_event*)(buf + (1 + HCI_EVENT_HDR_SIZE));
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        // Parse through the raw LE report memory block properly
        uint8_t num_reports = meta->data[0];
        uint8_t* ptr = meta->data + 1;

        for (int i = 0; i < num_reports; i++) {
            le_advertising_info* info = (le_advertising_info*)ptr;
            char mac[18];
            ba2str(&info->bdaddr, mac);

            // RSSI is always exactly one byte after the payload data ends
            int8_t rawRssi = (int8_t)info->data[info->length];

            if (strcasecmp(mac, TARGET_MAC) == 0) {
                std::lock_guard<std::mutex> lock(stateMutex);
                current_rssi = rawRssi;
                float rawDist = rssiToMeters((int)rawRssi);
                current_distance = filter.update(rawDist);

                std::cout << "🎯 BEACON FOUND! RSSI: " << current_rssi 
                          << " | Smooth Dist: " << current_distance << "m" << std::endl;
            }

            // Jump memory pointer to the next report in the packet
            ptr += (sizeof(le_advertising_info) + info->length + 1);
        }
    }
    close(dd);
}

// --- MINIMAL WEB UI ---
const std::string HTML_PAGE = 
    "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
    "<title>Single Beacon Test</title><style>"
    "body{background-color:#0A0A0A;color:#00FF66;font-family:monospace;margin:30px;font-size:24px;text-align:center;}"
    "</style></head><body>"
    "<h1>📡 Beacon Tracker</h1>"
    "<div>Distance: <span id='d'>0.00</span>m</div>"
    "<div style='font-size:16px;color:#888;'>RSSI: <span id='r'>0</span> dBm</div>"
    "<script>"
    "async function poll(){"
    "try{let res=await fetch('/data');let data=await res.json();"
    "document.getElementById('d').innerText=data.dist.toFixed(2);"
    "document.getElementById('r').innerText=data.rssi;"
    "}catch(e){}}setInterval(poll,100);"
    "</script></body></html>";

void httpServerThread() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);
    bind(server_fd, (struct sockaddr*)&address, sizeof(address));
    listen(server_fd, 10);

    while (true) {
        int client_socket = accept(server_fd, nullptr, nullptr);
        if (client_socket < 0) continue;

        char reqBuf[1024] = {0};
        read(client_socket, reqBuf, sizeof(reqBuf));
        std::string req(reqBuf);

        if (req.find("GET /data") != std::string::npos) {
            std::lock_guard<std::mutex> lock(stateMutex);
            std::string json = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
                               "{\"dist\":" + std::to_string(current_distance) + ",\"rssi\":" + std::to_string(current_rssi) + "}";
            write(client_socket, json.c_str(), json.length());
        } else {
            write(client_socket, HTML_PAGE.c_str(), HTML_PAGE.length());
        }
        close(client_socket);
    }
}

int main() {
    std::thread ble(bluetoothSnifferThread);
    std::thread http(httpServerThread);
    ble.join();
    http.join();
    return 0;
}