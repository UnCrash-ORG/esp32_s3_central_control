/**
 * robot_web.c - Web 控制台核心
 *
 * 结构：
 *   - 一个 FreeRTOS 任务 `web_console_task`：每 100ms 采集 IMU 并通过 WS 广播；
 *     同时阻塞等待动作队列，消费 HTTP handler 投递的副作用请求。
 *   - HTTP server：三个静态页面 + /api/pid(GET/POST) + /api/pid/all(POST)
 *     + /api/battery(GET) + /api/calibrate(POST) + /api/ota/* (上传)
 *   - WebSocket server：注册 ws_handler，client 上线/下线时维护 fd 集合，
 *     主任务通过 `ws_broadcast_json` 推给所有在线 client。
 *
 * 重要：所有硬件写操作（PID、IMU 校准、OTA 写入）都被投递到动作队列，
 * 最终在主任务的上下文里执行 —— 这样满足"所有逻辑集中在一个任务"的约束，
 * 同时也避免了 HTTP / WS 回调里阻塞硬件。
 */

#include "robot_web.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "cJSON.h"

static const char *TAG = "robot_web";

/* ---------- 内部状态 ---------- */

#define CONSOLE_TASK_STACK   (4096)
#define CONSOLE_TASK_PRIO    (5)
#define ACTION_QUEUE_LEN     (8)
#define WS_MAX_CLIENTS       (4)
#define WS_PUSH_INTERVAL_MS  (100)

static TaskHandle_t   s_console_task = NULL;
static QueueHandle_t  s_action_queue = NULL;
static SemaphoreHandle_t s_imu_mutex = NULL;

/* 最新 IMU 快照，任何 handler 都可以无锁读快照（加 mutex） */
static imu_snapshot_t s_imu_cache;
static pid_set_t      s_pid_cache;
static float          s_battery_v = 11.8f;

/* WS 在线 fd 集合（HTTP handler 与主任务并发访问，用同一个 mutex 保护） */
static int s_ws_fds[WS_MAX_CLIENTS];
static int s_ws_count = 0;

/* ---------- 工具函数 ---------- */

static inline int fd_in_set(int fd) {
    for (int i = 0; i < s_ws_count; ++i) {
        if (s_ws_fds[i] == fd) return 1;
    }
    return 0;
}

static inline void fd_add(int fd) {
    if (s_ws_count >= WS_MAX_CLIENTS) return;
    if (fd_in_set(fd)) return;
    s_ws_fds[s_ws_count++] = fd;
}

static inline void fd_remove(int fd) {
    int w = 0;
    for (int i = 0; i < s_ws_count; ++i) {
        if (s_ws_fds[i] != fd) s_ws_fds[w++] = s_ws_fds[i];
    }
    s_ws_count = w;
}

/* 从 EMIT_FILES 里读出嵌入的 HTML，由链接器提供符号 */
#define DECLARE_EMBED(name) \
    extern const char name[] asm("_binary_" #name "_html_start"); \
    extern const char name ## _len[] asm("_binary_" #name "_html_length");

DECLARE_EMBED(index)
DECLARE_EMBED(pid)
DECLARE_EMBED(tools)

/* ---------- WS 广播 ---------- */

static void ws_broadcast_json(const char *json) {
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    httpd_handle_t server = NULL; /* 不需要 server 句柄，直接用 fd */
    /* esp_http_server 的 WS 广播需要 server + fd；这里用公开的 ws 扩展 API */
    extern bool httpd_ws_get_fd_info(httpd_handle_t, int, int *);
    extern esp_err_t httpd_ws_send_frame_to_all_clients(httpd_handle_t, httpd_ws_frame_t *);

    /* 拿到 server 句柄：我们在 start 时保存 */
    extern httpd_handle_t s_httpd_handle_for_ws;
    server = s_httpd_handle_for_ws;
    if (server == NULL) {
        xSemaphoreGive(s_imu_mutex);
        return;
    }

    httpd_ws_frame_t frame = {0};
    frame.payload = (uint8_t *)json;
    frame.len = strlen(json);
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.is_finished = true;

    httpd_ws_send_frame_to_all_clients(server, &frame);

    xSemaphoreGive(s_imu_mutex);
}

/* ---------- 主任务 ---------- */

static void web_console_task(void *arg) {
    ESP_LOGI(TAG, "web_console_task started, WS push every %d ms", WS_PUSH_INTERVAL_MS);

    TickType_t last_push = xTaskGetTickCount();
    float      battery_v;

    while (1) {
        /* 1) 处理动作队列（非阻塞，每次循环都清空） */
        action_t act;
        while (xQueueReceive(s_action_queue, &act, 0) == pdPASS) {
            switch (act.type) {
            case ACT_SET_PID: {
                ESP_LOGI(TAG, "ACT_SET_PID wheel=%d kp=%.3f ki=%.3f kd=%.3f",
                         act.wheel_idx, act.one_pid.kp, act.one_pid.ki, act.one_pid.kd);
                if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    s_pid_cache.wheels[act.wheel_idx] = act.one_pid;
                    xSemaphoreGive(s_imu_mutex);
                }
                robot_pid_set_one(act.wheel_idx, &act.one_pid);
                break;
            }
            case ACT_SET_ALL_PID: {
                ESP_LOGI(TAG, "ACT_SET_ALL_PID");
                if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    s_pid_cache = act.pid;
                    xSemaphoreGive(s_imu_mutex);
                }
                for (int i = 0; i < 4; ++i) {
                    robot_pid_set_one(i, &act.pid.wheels[i]);
                }
                break;
            }
            case ACT_CALIBRATE_IMU: {
                ESP_LOGI(TAG, "ACT_CALIBRATE_IMU");
                robot_imu_calibrate();
                break;
            }
            case ACT_REBOOT: {
                ESP_LOGI(TAG, "ACT_REBOOT (stub)");
                break;
            }
            default: break;
            }
        }

        /* 2) 定时 WS 推送：读 IMU → 组 JSON → 广播 */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_push) >= pdMS_TO_TICKS(WS_PUSH_INTERVAL_MS)) {
            last_push = now;

            imu_snapshot_t snap;
            robot_imu_read(&snap);

            /* 更新 cache 供 API GET */
            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_imu_cache = snap;
                xSemaphoreGive(s_imu_mutex);
            }

            battery_v = robot_battery_read_v();
            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_battery_v = battery_v;
                xSemaphoreGive(s_imu_mutex);
            }

            /* 拼 JSON —— 直接栈上 snprintf，避免动态分配 */
            char buf[512];
            int n = snprintf(buf, sizeof(buf),
                "{\"type\":\"imu\",\"ts\":%lu,"
                "\"acc\":[%.3f,%.3f,%.3f],"
                "\"gyr\":[%.3f,%.3f,%.3f],"
                "\"att\":[%.2f,%.2f,%.2f],"
                "\"bat\":%.2f}",
                (unsigned long)snap.tick_ms,
                snap.acc_x, snap.acc_y, snap.acc_z,
                snap.gyr_x, snap.gyr_y, snap.gyr_z,
                snap.att_pitch, snap.att_roll, snap.att_yaw,
                battery_v);
            if (n > 0 && (size_t)n < sizeof(buf)) {
                ws_broadcast_json(buf);
            }
        }

        /* 3) 小睡眠，让出 CPU。动作队列用非阻塞读，推送用 tick 差判断 */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ---------- HTTP handlers ---------- */

#define RESP_FROM_EMBED(req, name) do { \
    httpd_resp_set_type(req, "text/html; charset=utf-8"); \
    httpd_resp_send(req, name, strlen(name)); \
} while (0)

static esp_err_t h_index(httpd_req_t *req) {
    RESP_FROM_EMBED(req, index);
    return ESP_OK;
}
static esp_err_t h_pid(httpd_req_t *req) {
    RESP_FROM_EMBED(req, pid);
    return ESP_OK;
}
static esp_err_t h_tools(httpd_req_t *req) {
    RESP_FROM_EMBED(req, tools);
    return ESP_OK;
}

/* GET /api/pid  返回当前 PID JSON */
static esp_err_t h_api_pid_get(httpd_req_t *req) {
    char buf[384];
    pid_set_t copy;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return httpd_resp_send_500(req);
    }
    copy = s_pid_cache;
    xSemaphoreGive(s_imu_mutex);

    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"wheels\":["
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"out_min\":%.2f,\"out_max\":%.2f},"
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"out_min\":%.2f,\"out_max\":%.2f},"
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"out_min\":%.2f,\"out_max\":%.2f},"
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"out_min\":%.2f,\"out_max\":%.2f}"
        "]"
        "}",
        copy.wheels[0].kp, copy.wheels[0].ki, copy.wheels[0].kd, copy.wheels[0].out_min, copy.wheels[0].out_max,
        copy.wheels[1].kp, copy.wheels[1].ki, copy.wheels[1].kd, copy.wheels[1].out_min, copy.wheels[1].out_max,
        copy.wheels[2].kp, copy.wheels[2].ki, copy.wheels[2].kd, copy.wheels[2].out_min, copy.wheels[2].out_max,
        copy.wheels[3].kp, copy.wheels[3].ki, copy.wheels[3].kd, copy.wheels[3].out_min, copy.wheels[3].out_max);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

/* POST /api/pid?wheel=0   body: {"kp":..,"ki":..,"kd":..,"out_min":..,"out_max":..} */
static esp_err_t h_api_pid_post(httpd_req_t *req) {
    char qs[64];
    char body[256];

    int qs_len = httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char wheel_str[8] = "0";
    if (qs_len == ESP_OK) {
        httpd_query_key_value(qs, "wheel", wheel_str, sizeof(wheel_str));
    }
    uint8_t wi = (uint8_t)atoi(wheel_str);
    if (wi > 3) wi = 0;

    int recv = httpd_req_recv(req, body, req->content_len > (int)sizeof(body) - 1 ? (int)sizeof(body) - 1 : req->content_len);
    if (recv <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    body[recv] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json"); return ESP_FAIL; }

    action_t act = { .type = ACT_SET_PID, .wheel_idx = wi, .one_pid = {0} };
    cJSON *f;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "kp")))       act.one_pid.kp       = (float)f->valuedouble;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "ki")))       act.one_pid.ki       = (float)f->valuedouble;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "kd")))       act.one_pid.kd       = (float)f->valuedouble;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "out_min")))  act.one_pid.out_min  = (float)f->valuedouble;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "out_max")))  act.one_pid.out_max  = (float)f->valuedouble;
    if (act.one_pid.out_max <= act.one_pid.out_min) {
        act.one_pid.out_min = -300.0f; act.one_pid.out_max = 300.0f;
    }
    cJSON_Delete(root);

    xQueueSend(s_action_queue, &act, 0);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/pid/all  body: {"wheels": [ {...}, {...}, {...}, {...} ]} */
static esp_err_t h_api_pid_all(httpd_req_t *req) {
    char body[1024];
    int recv = httpd_req_recv(req, body, req->content_len > (int)sizeof(body) - 1 ? (int)sizeof(body) - 1 : req->content_len);
    if (recv <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    body[recv] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json"); return ESP_FAIL; }

    cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "wheels");
    action_t act = { .type = ACT_SET_ALL_PID };
    if (arr && cJSON_IsArray(arr)) {
        int i = 0;
        cJSON *w;
        cJSON_ArrayForEach(w, arr) {
            if (i >= 4) break;
            cJSON *f;
            if ((f = cJSON_GetObjectItemCaseSensitive(w, "kp")))      act.pid.wheels[i].kp      = (float)f->valuedouble;
            if ((f = cJSON_GetObjectItemCaseSensitive(w, "ki")))      act.pid.wheels[i].ki      = (float)f->valuedouble;
            if ((f = cJSON_GetObjectItemCaseSensitive(w, "kd")))      act.pid.wheels[i].kd      = (float)f->valuedouble;
            if ((f = cJSON_GetObjectItemCaseSensitive(w, "out_min"))) act.pid.wheels[i].out_min = (float)f->valuedouble;
            if ((f = cJSON_GetObjectItemCaseSensitive(w, "out_max"))) act.pid.wheels[i].out_max = (float)f->valuedouble;
            ++i;
        }
    }
    cJSON_Delete(root);

    xQueueSend(s_action_queue, &act, 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* GET /api/battery */
static esp_err_t h_api_battery(httpd_req_t *req) {
    float v;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        v = s_battery_v;
        xSemaphoreGive(s_imu_mutex);
    } else {
        v = robot_battery_read_v();
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"voltage\":%.2f}", v);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* POST /api/calibrate */
static esp_err_t h_api_calibrate(httpd_req_t *req) {
    action_t act = { .type = ACT_CALIBRATE_IMU };
    xQueueSend(s_action_queue, &act, 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* POST /api/ota  (multipart 固件上传 —— 打桩，真正 OTA 由 firmware_upload.c 实现) */
static esp_err_t h_api_ota(httpd_req_t *req) {
    ESP_LOGW(TAG, "/api/ota stub: len=%d", req->content_len);
    /* 这里先不真正读 multipart body，打桩返回 ok；真正实现参考 firmware_upload.c */
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"note\":\"stub, body_len=%d\"}", req->content_len);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* ---------- WebSocket handler ---------- */

/* 全局 server 句柄，供 ws_broadcast_json 里的 httpd_ws_send_frame_to_all_clients 使用 */
httpd_handle_t s_httpd_handle_for_ws = NULL;

static esp_err_t h_ws(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WS client connected fd=%d", httpd_req_to_sockfd(req));
        /* 注意：连接事件要在握手完成后（HTTP_GET 返回之后）才会被 httpd 内部处理 */
        /* 所以这里只打日志，真正维护 fd 在 frame handler 里 */
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    uint8_t  rx_buf[256];
    frame.payload = rx_buf;
    frame.max_len = sizeof(rx_buf);
    httpd_ws_recv_frame(req, &frame, 0); /* 先拿长度 */

    int fd = httpd_req_to_sockfd(req);
    if (!fd_in_set(fd)) {
        if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            fd_add(fd);
            xSemaphoreGive(s_imu_mutex);
            ESP_LOGI(TAG, "WS fd=%d registered, total=%d", fd, s_ws_count);
        }
    }

    /* 前端可能发一个订阅/心跳；目前忽略任何下行数据 */
    if (frame.len > 0) {
        if (frame.len < sizeof(rx_buf)) {
            httpd_ws_recv_frame(req, &frame, frame.len);
            rx_buf[frame.len] = '\0';
            ESP_LOGD(TAG, "WS recv: %s", rx_buf);
        }
    }
    return ESP_OK;
}

/* WS 关闭时 httpd 会给 handler 发一个特殊帧（fin 且 len==0），这里我们让 fd 集合保持
   直到下次 frame 到来；但为了准确，我们 hook httpd 的 close 回调——
   ESP-IDF v6.1 的 httpd 有 httpd_register_close_cb。 */

static void ws_close_cb(httpd_handle_t server, int sockfd) {
    (void)server;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        fd_remove(sockfd);
        xSemaphoreGive(s_imu_mutex);
        ESP_LOGI(TAG, "WS fd=%d closed, remaining=%d", sockfd, s_ws_count);
    }
}

/* ---------- 对外入口 ---------- */

esp_err_t robot_web_start(void) {
    /* 1) 同步原语 */
    s_imu_mutex   = xSemaphoreCreateMutex();
    s_action_queue = xQueueCreate(ACTION_QUEUE_LEN, sizeof(action_t));
    if (!s_imu_mutex || !s_action_queue) return ESP_ERR_NO_MEM;

    /* 2) 初始化默认 PID（打桩给一个合理值） */
    for (int i = 0; i < 4; ++i) {
        s_pid_cache.wheels[i].kp      = 1.20f;
        s_pid_cache.wheels[i].ki      = 0.15f;
        s_pid_cache.wheels[i].kd      = 0.05f;
        s_pid_cache.wheels[i].out_min = -300.0f;
        s_pid_cache.wheels[i].out_max =  300.0f;
    }
    s_battery_v = 11.8f;

    /* 3) 启动控制台任务 */
    xTaskCreatePinnedToCore(web_console_task, "web_console",
                            CONSOLE_TASK_STACK, NULL,
                            CONSOLE_TASK_PRIO, &s_console_task, 0);

    /* 4) 启动 HTTP Server */
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets    = 6;
    cfg.max_uri_handlers    = 12;
    cfg.ctrl_port           = 32768;
    cfg.task_priority       = 4;
    cfg.server_port         = 80;

    esp_err_t err = httpd_start(&s_httpd_handle_for_ws, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    /* close cb —— 保证 WS fd 集合在 client 断开时被清理 */
    httpd_register_close_cb(s_httpd_handle_for_ws, ws_close_cb);

    /* 5) 注册 URI handlers */
    httpd_uri_t u = { .uri = "/",          .method = HTTP_GET,  .handler = h_index,            .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);
    u.uri = "/pid";     u.handler = h_pid;     httpd_register_uri_handler(s_httpd_handle_for_ws, &u);
    u.uri = "/tools";   u.handler = h_tools;   httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/pid",      .method = HTTP_GET,    .handler = h_api_pid_get,   .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);
    u.method = HTTP_POST; u.handler = h_api_pid_post; httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/pid/all",  .method = HTTP_POST,   .handler = h_api_pid_all,  .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/battery",  .method = HTTP_GET,    .handler = h_api_battery,  .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/calibrate",.method = HTTP_POST,   .handler = h_api_calibrate,.user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/ota",      .method = HTTP_POST,   .handler = h_api_ota,      .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    /* WS 端点 */
    u = (httpd_uri_t){ .uri = "/ws",           .method = HTTP_GET,
                       .handler = h_ws,         .user_ctx = NULL,
                       .is_websocket = true };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    ESP_LOGI(TAG, "Web console ready on :80 (WS /ws, pages / /pid /tools)");
    return ESP_OK;
}
