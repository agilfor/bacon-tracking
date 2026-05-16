#include <iostream>
#include <cmath>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

// Linux Bluetooth Headers
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

// --- CONFIGURATION ---
const char* BEACON_1_MAC = "48:87:2d:7c:f1:07";
const char* BEACON_2_MAC = "48:87:2d:7c:f0:fc"; // Change to your Beacon 2
const char* BEACON_3_MAC = "48:87:2d:7c:f0:ff"; // Change to your Beacon 3

float RSSI_AT_1M = -57.0f;
float N_VALUE = 2.0f;

// --- STRUCTURES & STATE ---
class PureKalman {
public:
    float e_mea, e_est, q, last_est;
    PureKalman(float mea = 2.0f, float est = 2.0f, float q_val = 1.2f)
        : e_mea(mea), e_est(est), q(q_val), last_est(0.0f) {}

    float update(float measurement) {
        float kalman_gain = e_est / (e_est + e_mea);
        float current_est = last_est + kalman_gain * (measurement - last_est);
        e_est = (1.0f - kalman_gain) * e_est + std::abs(last_est - current_est) * q;
        last_est = current_est;
        return current_est;
    }
};

struct TrackingState {
    bool running = true;
    float x = 0.0f;
    float y = 0.0f;
    float d1 = 0.0f, d2 = 0.0f, d3 = 0.0f;
    int rssi1 = 0, rssi2 = 0, rssi3 = 0;
};

TrackingState state;
std::mutex stateMutex;

PureKalman filter1, filter2, filter3;

// --- MATH ---
float rssiToMeters(int rssi) {
    if (rssi == 0) return 0.0f;
    return std::pow(10.0f, (RSSI_AT_1M - (float)rssi) / (10.0f * N_VALUE));
}

void executeTrilateration() {
    float r1 = state.d1;
    float r2 = state.d2;
    float r3 = state.d3;

    if (r1 == 0.0f || r2 == 0.0f || r3 == 0.0f) return;

    // Fixed Stage coordinates for your anchors
    float x1 = 0.0f, y1 = 0.0f;
    float x2 = 8.0f, y2 = 0.0f;
    float x3 = 4.0f, y3 = 6.0f;

    float A = 2.0f * x2 - 2.0f * x1;
    float B = 2.0f * y2 - 2.0f * y1;
    float C = (r1*r1) - (r2*r2) - (x1*x1) + (x2*x2) - (y1*y1) + (y2*y2);
    float D = 2.0f * x3 - 2.0f * x2;
    float E = 2.0f * y3 - 2.0f * y2;
    float F = (r2*r2) - (r3*r3) - (x2*x2) + (x3*x3) - (y2*y2) + (y3*y3);

    float denominator = (E * A - B * D);
    if (std::abs(denominator) < 0.0001f) return;

    state.x = (C * E - F * B) / denominator;
    state.y = (C * D - A * F) / (B * D - A * E);
}

// --- BLUETOOTH SNIFFER CORE ---
void bluetoothSnifferThread() {
    int device_id = hci_get_route(NULL);
    int dd = hci_open_dev(device_id);
    
    if (device_id < 0 || dd < 0) {
        std::cerr << "CRITICAL: Could not open Bluetooth Device Interface." << std::endl;
        return;
    }

    // Set HCI Filter to only intercept LE Advertising Packets
    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    
    if (setsockopt(dd, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
        std::cerr << "CRITICAL: Failed to apply socket filter configuration." << std::endl;
        return;
    }

    uint8_t buf[HCI_MAX_FRAME_SIZE];
    std::cout << "[BLE] Raw HCI Listener activated on background core." << std::endl;

    while (true) {
        int len = read(dd, buf, sizeof(buf));
        if (len <= 0) continue;

        // Navigate the binary byte structure of an LE advertising report packet
        evt_le_meta_event* meta = (evt_le_meta_event*)(buf + (1 + HCI_EVENT_HDR_SIZE));
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        le_advertising_info* info = (le_advertising_info*)(meta->data + 1);
        
        char mac[18];
        ba2str(&info->bdaddr, mac);
        
        // Read the last data byte layout position which explicitly contains RSSI signature
        int8_t rawRssi = (int8_t)(*(meta->data + 1 + (1 + info->length)));

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            if (!state.running) continue;

            float rawDist = rssiToMeters((int)rawRssi);

            if (strcasecmp(mac, BEACON_1_MAC) == 0) {
                state.rssi1 = rawRssi;
                state.d1 = filter1.update(rawDist);
            } else if (strcasecmp(mac, BEACON_2_MAC) == 0) {
                state.rssi2 = rawRssi;
                state.d2 = filter2.update(rawDist);
            } else if (strcasecmp(mac, BEACON_3_MAC) == 0) {
                state.rssi3 = rawRssi;
                state.d3 = filter3.update(rawDist);
            } else {
                continue; // Ignore any background non-beacon devices
            }

            executeTrilateration();
        }
    }
    close(dd);
}

// --- NATIVE UI & HTTP WEB CONTEXT ---
const std::string HTML_PAGE = 
    "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
    "<title>C++ Tracker Engine</title><style>"
    "body{background-color:#0A0A0A;color:#EAEAEA;font-family:monospace;margin:30px;}"
    ".box{background-color:#141414;padding:25px;border-radius:6px;margin-bottom:15px;border:1px solid #222;}"
    "h1{color:#00FF66;font-size:20px;margin-top:0;}"
    ".metric{font-size:32px;font-weight:bold;color:#00E5FF;margin:10px 0;}"
    "button{background-color:#FF3366;color:white;border:none;padding:12px;width:100%;font-weight:bold;cursor:pointer;border-radius:4px;}"
    ".run{background-color:#00FF66;color:black;}"
    "</style></head><body><div style='max-width:500px;margin:0 auto;'>"
    "<h1>⚡ LumenTrack Native C++ Engine</h1>"
    "<div class='box'><h2>Stage Metrics</h2>"
    "<div>X Coord: <span class='metric' id='x'>0.00m</span></div>"
    "<div>Y Coord: <span class='metric' id='y'>0.00m</span></div></div>"
    "<div class='box'><h2>Engine Raw State</h2><div id='b'>Connecting data channel...</div></div>"
    "<button id='t' onclick='toggle()'>TOGGLE STATE</button></div>"
    "<script>"
    "async function poll(){"
    "try{let r=await fetch('/data');let d=await r.json();"
    "document.getElementById('x').innerText=d.x.toFixed(2)+'m';"
    "document.getElementById('y').innerText=d.y.toFixed(2)+'m';"
    "document.getElementById('b').innerHTML="
    "`B1: ${d.d1.toFixed(2)}m (${d.r1}dBm)<br>B2: ${d.d2.toFixed(2)}m (${d.r2}dBm)<br>B3: ${d.d3.toFixed(2)}m (${d.r3}dBm)`;"
    "let btn=document.getElementById('t');"
    "if(d.running){btn.innerText='ENGINE: ACTIVE';btn.className='run';}else{btn.innerText='ENGINE: HALTED';btn.className='';}"
    "}catch(e){}}"
    "async function toggle(){await fetch('/toggle',{method:'POST'});poll();}"
    "setInterval(poll,80);"
    "</script></body></html>";

void httpServerThread() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0 || listen(server_fd, 10) < 0) {
        std::cerr << "CRITICAL: Web Server port 8080 allocation failed." << std::endl;
        return;
    }

    std::cout << "[HTTP] Performance Web dashboard bound to port 8080." << std::endl;

    while (true) {
        int client_socket = accept(server_fd, nullptr, nullptr);
        if (client_socket < 0) continue;

        char requestBuffer[1024] = {0};
        read(client_socket, requestBuffer, sizeof(requestBuffer));
        std::string req(requestBuffer);

        if (req.find("GET /data") != std::string::npos) {
            std::lock_guard<std::mutex> lock(stateMutex);
            std::string json = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n{"
                               "\"running\":" + std::to_string(state.running) + ","
                               "\"x\":" + std::to_string(state.x) + ","
                               "\"y\":" + std::to_string(state.y) + ","
                               "\"d1\":" + std::to_string(state.d1) + ","
                               "\"d2\":" + std::to_string(state.d2) + ","
                               "\"d3\":" + std::to_string(state.d3) + ","
                               "\"r1\":" + std::to_string(state.rssi1) + ","
                               "\"r2\":" + std::to_string(state.rssi2) + ","
                               "\"r3\":" + std::to_string(state.rssi3) + "}";
            write(client_socket, json.c_str(), json.length());
        } 
        else if (req.find("POST /toggle") != std::string::npos) {
            std::lock_guard<std::mutex> lock(stateMutex);
            state.running = !state.running;
            std::string resp = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{\"status\":\"ok\"}";
            write(client_socket, resp.c_str(), resp.length());
        } 
        else {
            write(client_socket, HTML_PAGE.c_str(), HTML_PAGE.length());
        }
        close(client_socket);
    }
}

int main() {
    std::cout << "Initializing Native C++ Coordinate Translation Infrastructure..." << std::endl;

    // Fire processing core threads
    std::thread bleWorker(bluetoothSnifferThread);
    std::thread httpWorker(httpServerThread);

    // Keep the main binary context executing indefinitely
    bleWorker.join();
    httpWorker.join();

    return 0;
}