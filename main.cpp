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

// ==========================================
// 1. HARDWARE CONFIGURATION
// ==========================================
const char* BEACONS[3] = {
    "48:87:2D:7C:F1:07", // Beacon 1
    "48:87:2D:7C:F0:FF", // Beacon 2
    "48:87:2D:7C:F0:FC"  // Beacon 3
};

// Beacon physical locations in the room (X, Y, Z in meters)
float BX[3] = {2.87f, 0.00f, 1.71f};
float BY[3] = {0.00f, 0.00f, 1.71f};
float BZ[3] = {2.03f, 4.67f, 1.77f}; // Height of the beacons (e.g., 5 meters high)

// RF Calibration Constants
float RSSI_AT_1M = -57.0f;
float N_VALUE = 2.0f;

// ==========================================
// 2. ENGINE STATE & FILTERS
// ==========================================
struct TrackingState {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float d1 = 0.0f, d2 = 0.0f, d3 = 0.0f;
    int rssi1 = 0, rssi2 = 0, rssi3 = 0;
};
TrackingState state;
std::mutex stateMutex;

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
PureKalman filter1, filter2, filter3;

float rssiToMeters(int rssi) {
    if (rssi >= 0 || rssi < -120) return 0.0f; 
    return std::pow(10.0f, (RSSI_AT_1M - (float)rssi) / (10.0f * N_VALUE));
}

// ==========================================
// 3. 3D GRADIENT DESCENT MATH ENGINE
// ==========================================
void executeTrilateration() {
    float r[3] = {state.d1, state.d2, state.d3};
    
    // Require all 3 beacons to be active before solving 3D space
    if (r[0] == 0.0f || r[1] == 0.0f || r[2] == 0.0f) return;

    // Warm Start: Begin sliding from the last known coordinate
    float px = state.x, py = state.y, pz = state.z;
    
    int iterations = 50; 
    float learning_rate = 0.05f;

    for (int step = 0; step < iterations; step++) {
        float grad_x = 0.0f, grad_y = 0.0f, grad_z = 0.0f;

        for (int i = 0; i < 3; i++) {
            float dx = px - BX[i];
            float dy = py - BY[i];
            float dz = pz - BZ[i];
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // NaN Protection
            if (dist < 0.001f) dist = 0.001f; 
            
            float error = dist - r[i];
            
            grad_x += 2.0f * error * (dx / dist);
            grad_y += 2.0f * error * (dy / dist);
            grad_z += 2.0f * error * (dz / dist);
        }

        px -= learning_rate * grad_x;
        py -= learning_rate * grad_y;
        pz -= learning_rate * grad_z;

        // GRAVITY / CEILING CONSTRAINT
        // Force the coordinate to remain below the lowest hanging beacon
        float lowest_beacon = BZ[0];
        if (BZ[1] < lowest_beacon) lowest_beacon = BZ[1];
        if (BZ[2] < lowest_beacon) lowest_beacon = BZ[2];
        
        if (pz > lowest_beacon) {
            pz = lowest_beacon; 
        }
    }

    state.x = px;
    state.y = py;
    state.z = pz;
}

// ==========================================
// 4. BARE-METAL BLUETOOTH LISTENER
// ==========================================
void bluetoothSnifferThread() {
    int device_id = hci_get_route(NULL);
    int dd = hci_open_dev(device_id);
    
    if (device_id < 0 || dd < 0) {
        std::cerr << "CRITICAL: Could not open Bluetooth Device." << std::endl;
        return;
    }

    // Force radio to unfiltered active scanning mode
    hci_le_set_scan_enable(dd, 0x00, 0x00, 1000); 
    uint16_t interval = htobs(0x0010), window = htobs(0x0010);
    hci_le_set_scan_parameters(dd, 0x00, interval, window, 0x00, 0x00, 1000);
    hci_le_set_scan_enable(dd, 0x01, 0x00, 1000); // 0x00 explicitly disables duplicate filtering

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    setsockopt(dd, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));

    uint8_t buf[HCI_MAX_FRAME_SIZE];
    std::cout << "[BLE] 3D Engine running. Dashboard available on port 8080.\n";

    while (true) {
        int len = read(dd, buf, sizeof(buf));
        if (len <= 0) continue;

        evt_le_meta_event* meta = (evt_le_meta_event*)(buf + (1 + HCI_EVENT_HDR_SIZE));
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t num_reports = meta->data[0];
        uint8_t* ptr = meta->data + 1;

        for (int i = 0; i < num_reports; i++) {
            le_advertising_info* info = (le_advertising_info*)ptr;
            char mac[18];
            ba2str(&info->bdaddr, mac);
            
            // RSSI signature is located precisely 1 byte after payload data
            int8_t rawRssi = (int8_t)info->data[info->length];

            std::lock_guard<std::mutex> lock(stateMutex);
            float rawDist = rssiToMeters((int)rawRssi);
            bool updated = false;

            if (strcasecmp(mac, BEACONS[0]) == 0) {
                state.rssi1 = rawRssi; state.d1 = filter1.update(rawDist); updated = true;
            } else if (strcasecmp(mac, BEACONS[1]) == 0) {
                state.rssi2 = rawRssi; state.d2 = filter2.update(rawDist); updated = true;
            } else if (strcasecmp(mac, BEACONS[2]) == 0) {
                state.rssi3 = rawRssi; state.d3 = filter3.update(rawDist); updated = true;
            }

            if (updated) {
                executeTrilateration();
            }

            ptr += (sizeof(le_advertising_info) + info->length + 1);
        }
    }
}

// ==========================================
// 5. SMART WEB UI & HTTP SERVER
// ==========================================
const std::string HTML_PAGE = 
    "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
    "<title>3D Tracker Dashboard</title><style>"
    "body{background-color:#121212;color:#E0E0E0;font-family:-apple-system,BlinkMacSystemFont,sans-serif;margin:20px;}"
    ".box{background-color:#1E1E1E;padding:20px;border-radius:12px;margin-bottom:15px;border:1px solid #333;}"
    "h1, h2{margin-top:0; color:#BB86FC;} .val{font-weight:bold;font-size:28px; color:#03DAC6;}"
    "#status{padding:10px; border-radius:8px; font-weight:bold; text-align:center; margin-bottom:15px;}"
    ".status-wait{background-color:#4A3500; color:#FFB300; border:1px solid #FFB300;}"
    ".status-ready{background-color:#003314; color:#00FF66; border:1px solid #00FF66;}"
    ".b-off{color:#CF6679; font-weight:bold;} .b-on{color:#E0E0E0;}"
    "</style></head><body>"
    "<h1>🔮 LumenTrack 3D Engine</h1>"
    "<div id='status' class='status-wait'>⚠️ WAITING FOR BEACONS</div>"
    "<div class='box'><h2>Stage Coordinates</h2>"
    "X: <span id='x' class='val'>---</span><br>"
    "Y: <span id='y' class='val'>---</span><br>"
    "Z: <span id='z' class='val'>---</span></div>"
    "<div class='box'><h2>Hardware Status</h2><div id='b' style='font-family:monospace; font-size:18px; line-height:1.5;'>Loading...</div></div>"
    "<script>"
    "function formatB(name, dist, rssi) {"
    "  if(dist === 0) return `<span class='b-off'>${name} [OFFLINE]</span><br>`;"
    "  return `<span class='b-on'>${name} [ACTIVE] : ${dist.toFixed(2)}m (${rssi}dBm)</span><br>`;"
    "}"
    "async function poll(){"
    "try{let r=await fetch('/data');let d=await r.json();"
    "let ready = (d.d1 > 0 && d.d2 > 0 && d.d3 > 0);"
    "let stat = document.getElementById('status');"
    "if(ready) {"
    "  stat.className='status-ready'; stat.innerText='✅ 3D TRILATERATION ACTIVE';"
    "  document.getElementById('x').innerText=d.x.toFixed(2) + 'm';"
    "  document.getElementById('y').innerText=d.y.toFixed(2) + 'm';"
    "  document.getElementById('z').innerText=d.z.toFixed(2) + 'm';"
    "} else {"
    "  stat.className='status-wait'; stat.innerText='⚠️ WAITING FOR BEACONS';"
    "  document.getElementById('x').innerText='---';"
    "  document.getElementById('y').innerText='---';"
    "  document.getElementById('z').innerText='---';"
    "}"
    "document.getElementById('b').innerHTML = formatB('B1', d.d1, d.r1) + formatB('B2', d.d2, d.r2) + formatB('B3', d.d3, d.r3);"
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
    
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0 || listen(server_fd, 10) < 0) {
        std::cerr << "CRITICAL: Web Server port 8080 allocation failed." << std::endl;
        return;
    }

    while (true) {
        int client_socket = accept(server_fd, nullptr, nullptr);
        if (client_socket < 0) continue;

        char reqBuf[1024] = {0};
        read(client_socket, reqBuf, sizeof(reqBuf));
        std::string req(reqBuf);

        if (req.find("GET /data") != std::string::npos) {
            std::lock_guard<std::mutex> lock(stateMutex);
            char json[512]; 
            snprintf(json, sizeof(json), 
                     "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
                     "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"d1\":%.2f,\"d2\":%.2f,\"d3\":%.2f,\"r1\":%d,\"r2\":%d,\"r3\":%d}",
                     state.x, state.y, state.z, state.d1, state.d2, state.d3, state.rssi1, state.rssi2, state.rssi3);
            write(client_socket, json, strlen(json));
        } else {
            write(client_socket, HTML_PAGE.c_str(), HTML_PAGE.length());
        }
        close(client_socket);
    }
}

// ==========================================
// 6. MAIN EXECUTION
// ==========================================
int main() {
    std::cout << "Initializing LumenTrack Native 3D Infrastructure..." << std::endl;

    std::thread bleWorker(bluetoothSnifferThread);
    std::thread httpWorker(httpServerThread);

    bleWorker.join();
    httpWorker.join();

    return 0;
}