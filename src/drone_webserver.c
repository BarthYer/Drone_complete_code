/* drone_webserver.c — WiFi AP + slider control + real-time telemetry
 *
 * Single POST /rc endpoint: browser sends slider values, server responds
 * with telemetry. One round-trip every 200 ms (5 Hz) — low enough to
 * avoid sysworkq stack overflow (was 10 Hz before).
 *
 * Slider → DataPackage mapping (RC_MAX=1023, RC_CENTER=512):
 *   Gaz    (t  0..100 )  → y_left  = t * 1023 / 100
 *   Lacet  (y -100..100) → x_left  = 512 + y * 512 / 100
 *   Tangage(p -100..100) → y_right = 512 + p * 512 / 100
 *   Roulis (r -100..100) → x_right = 512 + r * 512 / 100
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/dhcpv4_server.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "drone_webserver.h"
#include "drone_mpu9250.h"
#include "drone_fsm.h"
#include "drone_mc_controller.h"  /* RC_MAX, RC_CENTER */

LOG_MODULE_REGISTER(drone_webserver, LOG_LEVEL_INF);

/* ── Config ─────────────────────────────────────────────────────── */
#define WIFI_SSID        "DroneControl"
#define WIFI_CHANNEL     6
#define AP_IP            "192.168.4.1"
#define DHCP_START       "192.168.4.100"
#define HTTP_PORT        80
#define HTTP_STACK_SIZE  8192
#define HTTP_PRIORITY    7
/* RC goes stale after this many ms without a POST /rc request.
 * Must be > JS poll interval (200 ms) by a comfortable margin.  */
#define RC_STALE_MS      2000U

/* ── Thread ─────────────────────────────────────────────────────── */
K_THREAD_STACK_DEFINE(http_stack, HTTP_STACK_SIZE);
static struct k_thread http_thread;

/* ── Shared RC (written by HTTP thread, read by FSM thread) ─────── */
static K_MUTEX_DEFINE(rc_mutex);
static struct DataPackage wifi_rc;
static int64_t rc_last_ms;

/* ── AP-ready semaphore ──────────────────────────────────────────── */
static K_SEM_DEFINE(ap_ready, 0, 1);
static struct net_mgmt_event_callback wifi_cb;

/* ── Static buffers (HTTP is single-threaded) ────────────────────── */
#define REQ_BUF_SIZE 512
static char req_buf[REQ_BUF_SIZE];
static char resp_buf[96];

/* ════════════════════════════════════════════════════════════════════
 * HTML control page — stored in flash, not RAM
 * ════════════════════════════════════════════════════════════════════ */
static const char HTML[] =
    "<!DOCTYPE html><html><head>"
    "<meta charset=UTF-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1,user-scalable=no'>"
    "<title>Drone</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{background:#0d1117;color:#e6edf3;font-family:system-ui,sans-serif;"
         "max-width:420px;margin:0 auto;padding:12px}"
    "h2{text-align:center;padding:10px 0;letter-spacing:2px}"
    "#state{text-align:center;font-size:1.8em;font-weight:700;padding:12px;"
           "border-radius:8px;background:#161b22;margin:8px 0}"
    "#tel{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin:8px 0}"
    ".tc{background:#161b22;border-radius:6px;padding:8px;text-align:center}"
    ".tv{font-size:1.2em;font-weight:700;color:#58a6ff}"
    ".tk{font-size:.7em;color:#8b949e;margin-top:2px}"
    ".sl{margin:12px 0}"
    ".slr{display:flex;justify-content:space-between;margin-bottom:5px;font-size:.9em}"
    ".slr small{color:#58a6ff;font-weight:600;min-width:48px;text-align:right}"
    "input[type=range]{-webkit-appearance:none;width:100%;height:10px;"
                      "background:#21262d;border-radius:5px;outline:none}"
    "input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:24px;"
      "height:24px;border-radius:50%;background:#58a6ff;cursor:pointer;"
      "box-shadow:0 0 6px #58a6ff66}"
    ".btns{display:flex;gap:10px;margin:16px 0}"
    "button{flex:1;padding:16px;font-size:1.05em;border:0;border-radius:8px;"
           "cursor:pointer;font-weight:700;letter-spacing:1px}"
    "#ba{background:#238636;color:#fff}"
    "#bd{background:#da3633;color:#fff}"
    "#cn{text-align:center;font-size:.8em;color:#8b949e;padding:4px 0}"
    "</style></head><body>"
    "<h2>DRONE</h2>"
    "<div id=state>OFFLINE</div>"
    "<div id=tel>"
    "<div class=tc><div class=tv id=vr>--</div><div class=tk>ROLL</div></div>"
    "<div class=tc><div class=tv id=vp>--</div><div class=tk>PITCH</div></div>"
    "<div class=tc><div class=tv id=vg>--</div><div class=tk>YAW/s</div></div>"
    "</div>"
    "<div class=sl>"
    "<div class=slr><b>Gaz (Throttle)</b><small id=lt>0 %</small></div>"
    "<input type=range id=thr min=0 max=100 value=0>"
    "</div>"
    "<div class=sl>"
    "<div class=slr><b>Lacet (Yaw)</b><small id=ly>0</small></div>"
    "<input type=range id=syaw min=-100 max=100 value=0>"
    "</div>"
    "<div class=sl>"
    "<div class=slr><b>Tangage (Pitch)</b><small id=lp>0</small></div>"
    "<input type=range id=spit min=-100 max=100 value=0>"
    "</div>"
    "<div class=sl>"
    "<div class=slr><b>Roulis (Roll)</b><small id=lr>0</small></div>"
    "<input type=range id=srol min=-100 max=100 value=0>"
    "</div>"
    "<div class=btns>"
    "<button id=ba onclick='arm()'>ARM</button>"
    "<button id=bd onclick='dis()'>DISARM</button>"
    "</div>"
    "<div id=cn>offline</div>"
    "<script>"
    "let active=false;"
    "const g=id=>document.getElementById(id);"
    "g('thr').oninput=e=>g('lt').textContent=e.target.value+' %';"
    "g('syaw').oninput=e=>g('ly').textContent=e.target.value;"
    "g('spit').oninput=e=>g('lp').textContent=e.target.value;"
    "g('srol').oninput=e=>g('lr').textContent=e.target.value;"
    "function arm(){"
    "active=true;"
    "g('ba').style.boxShadow='0 0 12px #238636';"
    "g('bd').style.boxShadow='none'}"
    "function dis(){"
    "active=false;"
    "g('thr').value=0;g('lt').textContent='0 %';"
    "g('ba').style.boxShadow='none';"
    "g('bd').style.boxShadow='0 0 12px #da3633'}"
    "async function poll(){"
    "try{"
    "const res=await fetch('/rc',{method:'POST',"
    "headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify({"
    "t:+g('thr').value,"
    "y:+g('syaw').value,"
    "p:+g('spit').value,"
    "r:+g('srol').value,"
    "a:active?1:0"
    "})});"
    "const d=await res.json();"
    "g('state').textContent=d.st;"
    "g('vr').textContent=(d.rol/10).toFixed(1);"
    "g('vp').textContent=(d.pit/10).toFixed(1);"
    "g('vg').textContent=(d.gz/10).toFixed(1);"
    "g('cn').textContent='connecte';"
    "}catch(e){g('cn').textContent='offline'}"
    "setTimeout(poll,200)}"
    "poll();"
    "</script></body></html>";

/* ════════════════════════════════════════════════════════════════════
 * WiFi AP setup — runs on the HTTP thread's own 8 KB stack
 * ════════════════════════════════════════════════════════════════════ */

static void on_wifi_event(struct net_mgmt_event_callback *cb,
                          uint64_t event, struct net_if *iface)
{
    if (event == NET_EVENT_WIFI_AP_ENABLE_RESULT) {
        LOG_INF("WiFi AP actif — SSID: %s  IP: %s", WIFI_SSID, AP_IP);
        k_sem_give(&ap_ready);
    }
}

static void wifi_ap_start(void)
{
    struct net_if *iface = net_if_get_wifi_sap();
    if (!iface) {
        LOG_ERR("Interface WiFi SAP introuvable");
        return;
    }

    net_mgmt_init_event_callback(&wifi_cb, on_wifi_event,
                                 NET_EVENT_WIFI_AP_ENABLE_RESULT);
    net_mgmt_add_event_callback(&wifi_cb);

    struct net_in_addr ip, gw, nm;
    net_addr_pton(AF_INET, AP_IP,           &ip);
    net_addr_pton(AF_INET, AP_IP,           &gw);
    net_addr_pton(AF_INET, "255.255.255.0", &nm);
    net_if_ipv4_addr_add(iface, &ip, NET_ADDR_MANUAL, 0);
    net_if_ipv4_set_gw(iface, &gw);
    net_if_ipv4_set_netmask_by_addr(iface, &ip, &nm);

    struct net_in_addr pool;
    net_addr_pton(AF_INET, DHCP_START, &pool);
    int r = net_dhcpv4_server_start(iface, &pool);
    if (r && r != -EALREADY) {
        LOG_WRN("DHCP server: %d", r);
    }

    struct wifi_connect_req_params p = {
        .ssid        = (const uint8_t *)WIFI_SSID,
        .ssid_length = sizeof(WIFI_SSID) - 1,
        .channel     = WIFI_CHANNEL,
        .security    = WIFI_SECURITY_TYPE_NONE,
        .band        = WIFI_FREQ_BAND_2_4_GHZ,
        .timeout     = SYS_FOREVER_MS,
    };
    net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, iface, &p, sizeof(p));
}

/* ════════════════════════════════════════════════════════════════════
 * HTTP helpers
 * ════════════════════════════════════════════════════════════════════ */

static void send_all(int s, const char *buf, int len)
{
    while (len > 0) {
        int n = zsock_send(s, buf, len, 0);
        if (n <= 0) break;
        buf += n;
        len -= n;
    }
}

/* Find integer value of key in a JSON string. Returns 0 if not found. */
static int jget_int(const char *json, const char *key)
{
    char k[16];
    snprintf(k, sizeof(k), "\"%s\":", key);
    const char *p = strstr(json, k);
    if (!p) return 0;
    p += strlen(k);
    while (*p == ' ') p++;
    return atoi(p);
}

static int16_t clamp16(int v, int lo, int hi)
{
    if (v < lo) return (int16_t)lo;
    if (v > hi) return (int16_t)hi;
    return (int16_t)v;
}

/* ════════════════════════════════════════════════════════════════════
 * Endpoint handlers
 * ════════════════════════════════════════════════════════════════════ */

/* POST /rc — receive slider values, respond with telemetry JSON */
static void handle_rc(int s, const char *body)
{
    int t = jget_int(body, "t");  /* throttle  0 .. 100   */
    int y = jget_int(body, "y");  /* yaw     -100 .. 100  */
    int p = jget_int(body, "p");  /* pitch   -100 .. 100  */
    int r = jget_int(body, "r");  /* roll    -100 .. 100  */
    int a = jget_int(body, "a");  /* active  0 or 1       */

    struct DataPackage rc = {
        /* Throttle: 0-100 → 0-RC_MAX                          */
        .y_left       = clamp16(t * RC_MAX / 100,     0, RC_MAX),
        /* Yaw/Pitch/Roll: -100..100 → 0..RC_MAX, center=RC_CENTER */
        .x_left       = clamp16(RC_CENTER + y * RC_CENTER / 100, 0, RC_MAX),
        .y_right      = clamp16(RC_CENTER + p * RC_CENTER / 100, 0, RC_MAX),
        .x_right      = clamp16(RC_CENTER + r * RC_CENTER / 100, 0, RC_MAX),
        .drone_active = (a != 0),
    };

    k_mutex_lock(&rc_mutex, K_FOREVER);
    wifi_rc    = rc;
    rc_last_ms = k_uptime_get();
    k_mutex_unlock(&rc_mutex);

    /* Telemetry response — values ×10 so JS can show one decimal */
    int rlen = snprintf(resp_buf, sizeof(resp_buf),
        "{\"rol\":%d,\"pit\":%d,\"gz\":%d,\"st\":\"%s\"}",
        (int)(att.roll_deg  * 10.0f),
        (int)(att.pitch_deg * 10.0f),
        (int)(gyro_z_dps    * 10.0f),
        drone_fsm_state_name(drone_fsm_get_state()));

    char hdr[128];
    int hlen = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n", rlen);
    send_all(s, hdr, hlen);
    send_all(s, resp_buf, rlen);
}

/* GET / — serve the HTML control page (stored in flash) */
static void handle_page(int s)
{
    char hdr[128];
    int hlen = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-cache\r\n"
        "\r\n", (int)sizeof(HTML) - 1);
    send_all(s, hdr, hlen);
    send_all(s, HTML, sizeof(HTML) - 1);
}

/* ════════════════════════════════════════════════════════════════════
 * Connection dispatcher
 * ════════════════════════════════════════════════════════════════════ */
static void handle_client(int s)
{
    /* Read request headers byte-by-byte until \r\n\r\n */
    int total = 0, hdr_end = -1;
    while (total < REQ_BUF_SIZE - 1) {
        int n = zsock_recv(s, req_buf + total, 1, 0);
        if (n <= 0) goto done;
        total++;
        if (total >= 4 &&
            memcmp(req_buf + total - 4, "\r\n\r\n", 4) == 0) {
            hdr_end = total;
            break;
        }
    }
    if (hdr_end < 0) goto done;
    req_buf[total] = '\0';

    char method[8], path[32];
    if (sscanf(req_buf, "%7s %31s", method, path) != 2) goto done;

    /* Read POST body based on Content-Length */
    if (strcmp(method, "POST") == 0) {
        const char *cl = strstr(req_buf, "Content-Length:");
        if (!cl) cl = strstr(req_buf, "content-length:");
        if (cl) {
            int blen = atoi(cl + 15);
            int got = 0;
            while (got < blen && total < REQ_BUF_SIZE - 1) {
                int n = zsock_recv(s, req_buf + total, 1, 0);
                if (n <= 0) break;
                total++; got++;
            }
            req_buf[total] = '\0';
        }
    }

    if (strcmp(method, "POST") == 0 && strcmp(path, "/rc") == 0) {
        handle_rc(s, req_buf + hdr_end);
    } else {
        handle_page(s);
    }

done:
    zsock_close(s);
}

/* ════════════════════════════════════════════════════════════════════
 * HTTP server thread
 * ════════════════════════════════════════════════════════════════════ */
static void http_server_thread(void *a, void *b, void *c)
{
    wifi_ap_start();
    k_sem_take(&ap_ready, K_FOREVER);

    int srv = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (srv < 0) {
        LOG_ERR("socket() echec: %d", srv);
        return;
    }

    int opt = 1;
    zsock_setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(HTTP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (zsock_bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERR("bind() echec");
        zsock_close(srv);
        return;
    }
    zsock_listen(srv, 2);
    LOG_INF("HTTP pret — http://%s/", AP_IP);

    while (1) {
        struct sockaddr_in cli;
        socklen_t clen = sizeof(cli);
        int c = zsock_accept(srv, (struct sockaddr *)&cli, &clen);
        if (c >= 0) {
            handle_client(c);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════
 * Public API
 * ════════════════════════════════════════════════════════════════════ */
struct DataPackage drone_webserver_get_rc(void)
{
    struct DataPackage rc;
    k_mutex_lock(&rc_mutex, K_FOREVER);
    rc = wifi_rc;
    if ((k_uptime_get() - rc_last_ms) > RC_STALE_MS) {
        rc.drone_active = false;
    }
    k_mutex_unlock(&rc_mutex);
    return rc;
}

void drone_webserver_start(void)
{
    k_thread_create(&http_thread, http_stack,
                    K_THREAD_STACK_SIZEOF(http_stack),
                    http_server_thread,
                    NULL, NULL, NULL,
                    HTTP_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&http_thread, "http_srv");
}
