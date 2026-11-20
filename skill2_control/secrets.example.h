// ============================================================
//  配置模板 —— 复制本文件为 secrets.h，然后填入你自己的值
//  secrets.h 已在 .gitignore 中，不会被上传到 GitHub
// ============================================================
#pragma once

// ① 你的 WiFi（必须是 2.4GHz，ESP32 不支持 5GHz）
#define WIFI_SSID  "YOUR_WIFI_SSID"
#define WIFI_PASS  "YOUR_WIFI_PASSWORD"

// ② 这台 ESP32 在 CIA 物联网平台的「设备密码」
//    在 CIA IoT Gateway App（白色界面）→ 设备 → 设备注册 → 生成 获得
//    注意：是「设备密码」，不是 WiFi 密码，也不是平台账号密码
#define CIA_UUID   "YOUR_DEVICE_PASSWORD"
