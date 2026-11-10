/**
 * robot_web.c - Web 控制台核心（ESP-IDF 6.1 兼容版）
 *
 * 结构：
 *   - 一个 FreeRTOS 任务 web_console_task：每 100ms 采集 IMU 并通过 WS 广播
 *   - HTTP server：三个静态页面 + /api/pid(GET/POST) + /api/pid/all(POST)
 *     + /api/battery(GET) + /api/calibrate(POST) + /api/ota 固件上传
 *   - WebSocket server：注册 ws_handler，维护在线 fd 集合，
 *     主任务通过 ws_broadcast_json 推给所有在线 client
 *
 * ESP-IDF 6.1 变更：
 *   - 整个 WS API 被 CONFIG_HTTPD_WS_SUPPORT 包起来，sdkconfig 里必须开启
 *   - httpd_ws_send_frame_to_all_clients 被移除 —— 自己遍历 fd 数组发送
 *   - httpd_register_close_cb 被移除 —— 用 WS 关闭帧检测 client 断开
 *   - httpd_ws_frame_t.is_finished → .final（字段改名）
 *   - index.html 符号名和 libc index() 冲突 —— 改名为 home.html
 */
/**
 * robot_web.c - Web 控制台核心（ESP-IDF 6.1 最终版）
 */

/* 强制开启 WS API —— PlatformIO sdkconfig 不生效 */
#ifndef CONFIG_HTTPD_WS_SUPPORT
#define CONFIG_HTTPD_WS_SUPPORT 1
#endif

/* 前向声明 —— 定义在文件末尾 */
extern httpd_handle_t s_httpd_handle_for_ws;
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

#define CONSOLE_TASK_STACK   (4096)
#define CONSOLE_TASK_PRIO    (5)
#define ACTION_QUEUE_LEN     (8)
#define WS_MAX_CLIENTS       (8)
#define WS_PUSH_INTERVAL_MS  (100)

static TaskHandle_t   s_console_task = NULL;
static QueueHandle_t  s_action_queue = NULL;
static SemaphoreHandle_t s_imu_mutex = NULL;

static imu_snapshot_t s_imu_cache;
static pid_set_t      s_pid_cache;
static float          s_battery_v = 11.8f;

static int s_ws_fds[WS_MAX_CLIENTS];
static int s_ws_count = 0;

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

/* 从 EMBED_FILES 读 HTML，链接器提供符号 */
#define DECLARE_EMBED(name) \
    extern const char name[] asm("_binary_" #name "_html_start"); \
    extern const char name ## _len[] asm("_binary_" #name "_html_length");

DECLARE_EMBED(home)
DECLARE_EMBED(pid)
DECLARE_EMBED(tools)

/* WS 广播：ESP-IDF 6.1 自己遍历 fd 发送 */
static void ws_broadcast_json(const char *json) {
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    httpd_handle_t server = s_httpd_handle_for_ws;
    if (server == NULL) {
        xSemaphoreGive(s_imu_mutex);
        return;
    }

    for (int i = 0; i < s_ws_count; ++i) {
        int fd = s_ws_fds[i];
        /* 检查 fd 还活着不 */
        if (httpd_ws_get_fd_info(server, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
            fd_remove(fd);
            --i;
            continue;
        }

        httpd_ws_frame_t frame;
        memset(&frame, 0, sizeof(frame));
        frame.payload  = (uint8_t *)json;
        frame.len      = strlen(json);
        frame.type     = HTTPD_WS_TYPE_TEXT;
        frame.final    = true;

        httpd_ws_send_data(server, fd, &frame);
    }

    xSemaphoreGive(s_imu_mutex);
}

/* 主任务：所有逻辑集中在这里 */
static void web_console_task(void *arg) {
    ESP_LOGI(TAG, "web_console_task started, WS push every %d ms", WS_PUSH_INTERVAL_MS);

    TickType_t last_push = xTaskGetTickCount();

    while (1) {
        /* 1) 消费动作队列（非阻塞，一次清完） */
        action_t act;
        while (xQueueReceive(s_action_queue, &act, 0) == pdPASS) {
            switch (act.type) {
            case ACT_SET_PID:
                ESP_LOGI(TAG, "ACT_SET_PID wheel=%d", act.wheel_idx);
                if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    s_pid_cache.wheels[act.wheel_idx] = act.one_pid;
                    xSemaphoreGive(s_imu_mutex);
                }
                robot_pid_set_one(act.wheel_idx, &act.one_pid);
                break;
            case ACT_SET_ALL_PID:
                ESP_LOGI(TAG, "ACT_SET_ALL_PID");
                if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    s_pid_cache = act.pid;
                    xSemaphoreGive(s_imu_mutex);
                }
                for (int i = 0; i < 4; ++i) robot_pid_set_one(i, &act.pid.wheels[i]);
                break;
            case ACT_CALIBRATE_IMU:
                ESP_LOGI(TAG, "ACT_CALIBRATE_IMU");
                robot_imu_calibrate();
                break;
            default: break;
            }
        }

        /* 2) 定时推 WS：读 IMU → 组 JSON → 广播 */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_push) >= pdMS_TO_TICKS(WS_PUSH_INTERVAL_MS)) {
            last_push = now;

            imu_snapshot_t snap;
            robot_imu_read(&snap);

            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_imu_cache = snap;
                xSemaphoreGive(s_imu_mutex);
            }

            float battery_v = robot_battery_read_v();
            if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_battery_v = battery_v;
                xSemaphoreGive(s_imu_mutex);
            }

            char buf[512];
            int n = snprintf(buf, sizeof(buf),
                "{\"ts\":%lu,"
                "\"acc\":[%.3f,%.3f,%.3f],"
                "\"gyr\":[%.3f,%.3f,%.3f],"
                "\"att\":[%.2f,%.2f,%.2f],"
                "\"bat\":%.2f}",
                (unsigned long)snap.tick_ms,
                snap.acc_x, snap.acc_y, snap.acc_z,
                snap.gyr_x, snap.gyr_y, snap.gyr_z,
                snap.att_pitch, snap.att_roll, snap.att_yaw,
                battery_v);
            if (n > 0 && (size_t)n < sizeof(buf)) ws_broadcast_json(buf);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* HTTP helpers */
#define RESP_FROM_EMBED(req, name) do { \
    httpd_resp_set_type(req, "text/html; charset=utf-8"); \
    httpd_resp_send(req, name, strlen(name)); \
} while (0)

static esp_err_t h_home(httpd_req_t *req)   { RESP_FROM_EMBED(req, home);   return ESP_OK; }
static esp_err_t h_pid(httpd_req_t *req)    { RESP_FROM_EMBED(req, pid);    return ESP_OK; }
static esp_err_t h_tools(httpd_req_t *req)  { RESP_FROM_EMBED(req, tools);  return ESP_OK; }

static esp_err_t h_api_pid_get(httpd_req_t *req) {
    char buf[384];
    pid_set_t copy;
    if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return httpd_resp_send_500(req);
    copy = s_pid_cache;
    xSemaphoreGive(s_imu_mutex);

    int n = snprintf(buf, sizeof(buf),
        "{\"wheels\":["
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"out_min\":%.2f,\"out_max\":%.2f},"
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"out_min\":%.2f,\"out_max\":%.2f},"
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"out_min\":%.2f,\"out_max\":%.2f},"
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"out_min\":%.2f,\"out_max\":%.2f}"
        "]}",
        copy.wheels[0].kp, copy.wheels[0].ki, copy.wheels[0].kd, copy.wheels[0].out_min, copy.wheels[0].out_max,
        copy.wheels[1].kp, copy.wheels[1].ki, copy.wheels[1].kd, copy.wheels[1].out_min, copy.wheels[1].out_max,
        copy.wheels[2].kp, copy.wheels[2].ki, copy.wheels[2].kd, copy.wheels[2].out_min, copy.wheels[2].out_max,
        copy.wheels[3].kp, copy.wheels[3].ki, copy.wheels[3].kd, copy.wheels[3].out_min, copy.wheels[3].out_max);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

static esp_err_t h_api_pid_post(httpd_req_t *req) {
    char qs[64], body[256];
    int qs_len = httpd_req_get_url_query_str(req, qs, sizeof(qs));
    char wheel_str[8] = "0";
    if (qs_len == ESP_OK) httpd_query_key_value(qs, "wheel", wheel_str, sizeof(wheel_str));
    uint8_t wi = (uint8_t)atoi(wheel_str);
    if (wi > 3) wi = 0;

    int recv = httpd_req_recv(req, body,
        req->content_len > (int)sizeof(body) - 1 ? (int)sizeof(body) - 1 : req->content_len);
    if (recv <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    body[recv] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (!root) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json"); return ESP_FAIL; }

    action_t act = { .type = ACT_SET_PID, .wheel_idx = wi, .one_pid = {0} };
    cJSON *f;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "kp")))      act.one_pid.kp      = (float)f->valuedouble;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "ki")))      act.one_pid.ki      = (float)f->valuedouble;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "kd")))      act.one_pid.kd      = (float)f->valuedouble;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "out_min"))) act.one_pid.out_min = (float)f->valuedouble;
    if ((f = cJSON_GetObjectItemCaseSensitive(root, "out_max"))) act.one_pid.out_max = (float)f->valuedouble;
    if (act.one_pid.out_max <= act.one_pid.out_min) {
        act.one_pid.out_min = -300.0f; act.one_pid.out_max = 300.0f;
    }
    cJSON_Delete(root);
    xQueueSend(s_action_queue, &act, 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t h_api_pid_all(httpd_req_t *req) {
    char body[1024];
    int recv = httpd_req_recv(req, body,
        req->content_len > (int)sizeof(body) - 1 ? (int)sizeof(body) - 1 : req->content_len);
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

static esp_err_t h_api_calibrate(httpd_req_t *req) {
    action_t act = { .type = ACT_CALIBRATE_IMU };
    xQueueSend(s_action_queue, &act, 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t h_api_ota(httpd_req_t *req) {
    ESP_LOGW(TAG, "/api/ota stub: len=%d", req->content_len);
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ok\":true,\"note\":\"stub, body_len=%d\"}", req->content_len);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* WS handler */
httpd_handle_t s_httpd_handle_for_ws = NULL;

static esp_err_t h_ws(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WS handshake fd=%d", fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    uint8_t rx_buf[256];
    frame.payload = rx_buf;


    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        fd_remove(fd);
        return ESP_FAIL;
    }

    if (!fd_in_set(fd)) {
        if (xSemaphoreTake(s_imu_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            fd_add(fd);
            xSemaphoreGive(s_imu_mutex);
            ESP_LOGI(TAG, "WS fd=%d registered total=%d", fd, s_ws_count);
        }
    }

    /* 关闭帧检测 */
    if (frame.type == HTTPD_WS_TYPE_CLOSE || (frame.len == 0 && frame.final)) {
        ESP_LOGI(TAG, "WS fd=%d closed", fd);
        fd_remove(fd);
        return ESP_OK;
    }

    if (frame.len > 0 && frame.len < sizeof(rx_buf)) {
        httpd_ws_recv_frame(req, &frame, frame.len);
        rx_buf[frame.len] = '\0';
        ESP_LOGD(TAG, "WS recv: %s", rx_buf);
    }
    return ESP_OK;
}

esp_err_t robot_web_start(void) {
    s_imu_mutex   = xSemaphoreCreateMutex();
    s_action_queue = xQueueCreate(ACTION_QUEUE_LEN, sizeof(action_t));
    if (!s_imu_mutex || !s_action_queue) return ESP_ERR_NO_MEM;

    for (int i = 0; i < 4; ++i) {
        s_pid_cache.wheels[i].kp      = 1.20f;
        s_pid_cache.wheels[i].ki      = 0.15f;
        s_pid_cache.wheels[i].kd      = 0.05f;
        s_pid_cache.wheels[i].out_min = -300.0f;
        s_pid_cache.wheels[i].out_max =  300.0f;
    }
    s_battery_v = 11.8f;

    xTaskCreatePinnedToCore(web_console_task, "web_console",
                            CONSOLE_TASK_STACK, NULL,
                            CONSOLE_TASK_PRIO, &s_console_task, 0);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = 8;
    cfg.max_uri_handlers = 12;
    cfg.server_port      = 80;

    esp_err_t err = httpd_start(&s_httpd_handle_for_ws, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t u;
    u = (httpd_uri_t){ .uri = "/",         .method = HTTP_GET, .handler = h_home,   .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);
    u = (httpd_uri_t){ .uri = "/pid",      .method = HTTP_GET, .handler = h_pid,    .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);
    u = (httpd_uri_t){ .uri = "/tools",    .method = HTTP_GET, .handler = h_tools,  .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/pid",     .method = HTTP_GET,  .handler = h_api_pid_get,   .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);
    u = (httpd_uri_t){ .uri = "/api/pid",     .method = HTTP_POST, .handler = h_api_pid_post,  .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/pid/all", .method = HTTP_POST, .handler = h_api_pid_all,   .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/battery", .method = HTTP_GET,  .handler = h_api_battery,   .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/calibrate", .method = HTTP_POST, .handler = h_api_calibrate, .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/api/ota",     .method = HTTP_POST, .handler = h_api_ota,       .user_ctx = NULL };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    u = (httpd_uri_t){ .uri = "/ws",          .method = HTTP_GET,
                       .handler = h_ws,        .user_ctx = NULL,
                       .is_websocket = true };
    httpd_register_uri_handler(s_httpd_handle_for_ws, &u);

    ESP_LOGI(TAG, "Web console ready on :80");
    return ESP_OK;
}
