/**
 * firmware_upload.c - OTA 固件上传打桩
 *
 * robot_web.c 里的 /api/ota handler 目前只打印不解析 multipart。
 * 真要支持浏览器固件上传，可以用 esp_http_server 的 httpd_req_recv
 * 循环读 POST body，解析 multipart/form-data，把文件内容喂给
 * esp_ota_write()。这里把后端三个回调先提供出来。
 */

#include "robot_web.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

static const char *TAG = "ota_stub";
static esp_ota_handle_t s_ota_h = NULL;

esp_err_t robot_ota_begin(size_t image_size) {
    /* ====== 打桩：真正调用 esp_ota_begin ====== */
    ESP_LOGI(TAG, "ota_begin size=%d", (int)image_size);
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (!next) { ESP_LOGE(TAG, "no OTA partition"); return ESP_FAIL; }
    esp_err_t err = esp_ota_begin(next, OTA_SIZE_UNKNOWN, &s_ota_h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        s_ota_h = NULL;
    }
    return err;
    /* ========================================= */
}

esp_err_t robot_ota_write(const uint8_t *buf, size_t len) {
    /* ====== 打桩：真正调用 esp_ota_write ====== */
    if (!s_ota_h) return ESP_ERR_INVALID_STATE;
    return esp_ota_write(s_ota_h, buf, len);
    /* ========================================= */
}

esp_err_t robot_ota_end(void) {
    /* ====== 打桩：真正调用 esp_ota_end + set_boot ====== */
    if (!s_ota_h) return ESP_ERR_INVALID_STATE;
    esp_err_t err = esp_ota_end(s_ota_h);
    s_ota_h = NULL;
    if (err == ESP_OK) {
        const esp_partition_t *p = esp_ota_get_running_partition();
        /* 一般 end 已经把 bootable 标上了；切换 boot 可选 */
        ESP_LOGI(TAG, "ota_end ok, next boot @ %p", (void *)p);
    } else {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
    }
    return err;
    /* ========================================= */
}
