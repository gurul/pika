// ELEGOO ESP32-WROVER-Camera V1.2 pin map.
// Verified against the board schematic (FPC J2 and header P1), 2026-09-22.
#pragma once

#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  15
#define XCLK_GPIO_NUM   27
#define SIOD_GPIO_NUM   22
#define SIOC_GPIO_NUM   23

#define Y9_GPIO_NUM     19
#define Y8_GPIO_NUM     36
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     39
#define Y5_GPIO_NUM      5
#define Y4_GPIO_NUM     34
#define Y3_GPIO_NUM     35
#define Y2_GPIO_NUM     32
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   26
#define PCLK_GPIO_NUM   21

// UART to the UNO shield (header P1).
#define UNO_RX_PIN      33
#define UNO_TX_PIN       4

// Green status LED, lit when a command client is connected.
#define LED_PIN         13
