#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"

// Cấu hình chân cho module AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_GPIO     4

// Cấu hình điều khiển độ sáng Flash LED bằng PWM (LEDC)
// Mặc định ban đầu bật mức HIGH (100% ~ 255) rất chói, gây sụt áp và quá nhiệt vi mạch.
// Dùng kênh PWM riêng (Channel 7, Timer 1) để điều chỉnh độ sáng dịu vừa đủ cho camera.
#define FLASH_LEDC_CHANNEL 7       // Kênh PWM riêng (Camera XCLK dùng LEDC_CHANNEL_0)
#define FLASH_LEDC_FREQ    5000    // Tần số PWM 5kHz
#define FLASH_LEDC_RES     8       // Độ phân giải 8-bit (0 - 255)
#define FLASH_BRIGHTNESS   25      // Mức sáng dịu (~10% công suất: 25/255, bù sáng tốt, không chói, không nóng)

static uint8_t currentFlashBrightness = FLASH_BRIGHTNESS;

void setFlashBrightness(uint8_t duty) {
    currentFlashBrightness = duty;
    ledcWrite(FLASH_LEDC_CHANNEL, duty);
}

void flashOn(uint8_t duty = currentFlashBrightness) {
    ledcWrite(FLASH_LEDC_CHANNEL, duty);
}

void flashOff() {
    ledcWrite(FLASH_LEDC_CHANNEL, 0);
}

// UART2 trên ESP32-CAM: Giao tiếp với ESP32 chính (OBC)
// Dùng GPIO 14 (TX) và GPIO 15 (RX) thay vì UART0
#define OBC_UART_TX   14
#define OBC_UART_RX   15

// Xung nhịp XCLK cho cảm biến OV2640 (Chạy mức tối đa 20MHz khi chụp, ngắt sạch khi xong)
#define CAMERA_XCLK_FREQ  20000000 // 20MHz (Tối đa của cảm biến OV2640)

static bool isCameraRunning = false;

static void sendCaptureError(const char* message) {
    Serial2.print("ERROR:");
    Serial2.println(message);
}

// =========================================================================
// QUẢN LÝ NGUỒN VÀ XUNG NHỊP CAMERA THEO CƠ CHẾ ON-DEMAND (CHỐNG NHIỄU GPS)
// =========================================================================

// Ngắt hoàn toàn cảm biến Camera OV2640 và tắt xung dao động XCLK
void stopCamera() {
    if (isCameraRunning) {
        esp_camera_deinit();
        isCameraRunning = false;
        delay(20); // Chờ giải phóng tài nguyên DMA và ngắt
    }

    // Đưa chân PWDN lên mức HIGH (OV2640 Power-Down active HIGH)
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH);

    // Kéo chân XCLK (GPIO 0) về mức LOW để dập tắt xung dao động cao tần
    pinMode(XCLK_GPIO_NUM, INPUT_PULLDOWN);

}

// Khởi tạo và đánh thức camera khi có yêu cầu chụp ảnh
bool startCamera() {
    if (isCameraRunning) {
        return true;
    }

    // Đánh thức cảm biến OV2640 khỏi Power-Down và chờ ổn định nguồn
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, LOW);
    delay(20);

    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = CAMERA_XCLK_FREQ;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("ERROR: Camera init that bai: 0x%x\n", err);
        return false;
    }

    isCameraRunning = true;
    Serial.println("CAM: Camera khoi tao thanh cong. Dang on dinh AGC/AEC...");

    // Cảm biến CMOS cần 1 frame nháp để tự cân bằng phơi sáng/trắng sau khi thức dậy
    delay(60);
    camera_fb_t* dummy = esp_camera_fb_get();
    if (dummy) {
        esp_camera_fb_return(dummy);
    }

    return true;
}

// =========================================================================
// CÁC HÀM XỬ LÝ LỆNH CHỤP ẢNH
// =========================================================================

// Hàm debug test camera độc lập qua Serial0 (Serial Monitor trên máy tính)
void testCaptureViaSerial0() {
    Serial.println("CAM: Khoi dong che do Test...");
    
    if (!startCamera()) {
        Serial.println("DEBUG ERROR: Khong the khoi tao camera!");
        stopCamera();
        
        return;
    }

    Serial.println("DEBUG: Dang chup anh test tu cam bien OV2640...");
    unsigned long tStart = millis();
    flashOn();
    camera_fb_t* fb = esp_camera_fb_get();
    flashOff();
    unsigned long tDuration = millis() - tStart;

    if (!fb) {
        Serial.println("DEBUG ERROR: Khong the chup anh! (fb_get tra ve NULL)");
        stopCamera();
        Serial.println("===================================================\n");
        return;
    }

    Serial.printf("DEBUG SUCCESS: Chup anh test thanh cong!\n");
    Serial.printf("  - Kich thuoc anh : %u bytes (%.2f KB)\n", fb->len, fb->len / 1024.0);
    Serial.printf("  - Do phan giai   : %u x %u\n", fb->width, fb->height);
    Serial.printf("  - Dinh dang      : %s\n", (fb->format == PIXFORMAT_JPEG) ? "JPEG" : "RAW/OTHER");
    Serial.printf("  - Thoi gian chup : %lu ms\n", tDuration);

    if (fb->len >= 4 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
        Serial.println("  - Header JPEG    : Hop le (0xFF 0xD8 - Start of Image)");
    } else {
        Serial.println("  - Header JPEG    : CANH BAO: Khong dung header JPEG!");
    }

    if (fb->len >= 2 && fb->buf[fb->len - 2] == 0xFF && fb->buf[fb->len - 1] == 0xD9) {
        Serial.println("  - Footer JPEG    : Hop le (0xFF 0xD9 - End of Image)");
    } else {
        Serial.println("  - Footer JPEG    : CANH BAO: Thieu footer JPEG!");
    }

    esp_camera_fb_return(fb);

    // Chụp xong -> Tắt camera ngay lập tức để GPS không bị nhiễu
    stopCamera();
    Serial.println("===================================================\n");
}

// Xử lý lệnh chụp và gửi ảnh sang OBC qua UART2
void handleOBCCommand() {

    if (!startCamera()) {
        sendCaptureError("INIT_FAIL");
        stopCamera();
        return;
    }

    // Nháy đèn Flash với độ sáng dịu trong thời gian cảm biến OV2640 phơi sáng và chụp ảnh
    flashOn();
    camera_fb_t* frame = esp_camera_fb_get();
    flashOff();

    if (frame == nullptr) {
        sendCaptureError("FRAME_NULL");
        Serial.println("CAM: Loi: Khong chup duoc anh!");
        stopCamera();
        return;
    }

    Serial2.printf("SIZE:%u\n", frame->len);
    Serial.printf("CAM: Kich thuoc anh gui OBC: %u bytes\n", frame->len);

    // Chờ OBC xác nhận sẵn sàng nhận dữ liệu
    unsigned long startTime = millis();
    bool ready = false;
    while (millis() - startTime < 5000) {
        if (Serial2.available()) {
            String response = Serial2.readStringUntil('\n');
            response.trim();
            if (response == "OK") {
                ready = true;
                break;
            }
        }
        delay(1);
    }

    if (!ready) {
        sendCaptureError("ACK_TIMEOUT");
        Serial.println("CAM: Loi: OBC khong phan hoi ACK!");
        esp_camera_fb_return(frame);
        stopCamera();
        while (Serial2.available()) Serial2.read();
        return;
    }

    // Gửi dữ liệu ảnh raw qua UART2 đến OBC
    Serial2.write(frame->buf, frame->len);
    Serial2.flush();

    // Chờ OBC xác nhận hoàn tất (tối đa 1 giây, tránh nghẽn luồng nếu OBC ngắt MUX)
    startTime = millis();
    while (millis() - startTime < 1000) {
        if (Serial2.available()) {
            String response = Serial2.readStringUntil('\n');
            response.trim();
            if (response == "DONE") {
                Serial.printf("CAM: Sent %u bytes to OBC successfully\n", frame->len);
                break;
            }
        }
        delay(1);
    }

    esp_camera_fb_return(frame);

    // Truyền xong -> Tắt camera ngay lập tức để GPS tiếp tục định vị bình thường
    stopCamera();

    // Dọn sạch đệm Serial2 để sẵn sàng nhận lệnh tiếp theo ngay lập tức
    while (Serial2.available()) Serial2.read();
}

void setup() {
    // 1. Chạy CPU ở xung nhịp tối đa 240MHz để xử lý ảnh và nén JPEG nhanh nhất
    setCpuFrequencyMhz(240);

    // 2. Tắt triệt để Wi-Fi và Bluetooth để tránh rò sóng 2.4GHz
    WiFi.mode(WIFI_OFF);
    btStop();

    // Cấu hình PWM cho Flash LED để điều khiển độ sáng dịu (chống chói, giảm nhiệt)
    ledcSetup(FLASH_LEDC_CHANNEL, FLASH_LEDC_FREQ, FLASH_LEDC_RES);
    ledcAttachPin(FLASH_LED_GPIO, FLASH_LEDC_CHANNEL);
    flashOff();

    // 3. Mặc định khởi động: Đưa camera vào trạng thái SLEEP ngay lập tức
    stopCamera();

    // UART0 (Serial) kết nối với PC qua USB/FTDI để debug
    Serial.begin(115200);
    delay(500);

    // UART2 (Serial2) kết nối với OBC
    Serial2.begin(115200, SERIAL_8N1, OBC_UART_RX, OBC_UART_TX);
    Serial2.setTimeout(100);

    Serial2.println("CAMERA_READY");
}

void loop() {
    // 1. Kiểm tra lệnh debug từ Serial0 (máy tính qua USB/FTDI)
    if (Serial.available() > 0) {
        char debugCmd = Serial.read();
        if (debugCmd == 't' || debugCmd == 'T' || debugCmd == 'p' || debugCmd == 'P') {
            testCaptureViaSerial0();
        } else if (debugCmd == 'f' || debugCmd == 'F') {
            static bool flashState = false;
            flashState = !flashState;
            if (flashState) flashOn();
            else flashOff();
            Serial.printf("DEBUG: Flash LED: %s (Brightness: %d/255 ~ %.1f%%)\n",
                          flashState ? "ON" : "OFF", currentFlashBrightness, (currentFlashBrightness * 100.0f) / 255.0f);
        } else if (debugCmd == '+' || debugCmd == '=') {
            if (currentFlashBrightness <= 235) currentFlashBrightness += 20;
            else currentFlashBrightness = 255;
            flashOn();
            Serial.printf("DEBUG: Tang do sang Flash: %d/255\n", currentFlashBrightness);
        } else if (debugCmd == '-' || debugCmd == '_') {
            if (currentFlashBrightness >= 25) currentFlashBrightness -= 20;
            else currentFlashBrightness = 5;
            flashOn();
            Serial.printf("DEBUG: Giam do sang Flash: %d/255\n", currentFlashBrightness);
        } else if (debugCmd == 's' || debugCmd == 'S') {
            Serial.println("\n--- [SYSTEM & EMI STATUS] ---");
            Serial.printf("  - CPU Frequency : %d MHz\n", getCpuFrequencyMhz());
            Serial.printf("  - Camera Status : %s\n", isCameraRunning ? "RUNNING (ACTIVE)" : "SLEEP / DEINIT (ZERO EMI)");
            Serial.printf("  - Camera XCLK   : %d MHz\n", CAMERA_XCLK_FREQ / 1000000);
            Serial.println("  - Wi-Fi Radio   : OFF");
            Serial.println("  - Bluetooth     : OFF");
            
        }
    }

    // 2. Chờ lệnh từ ESP32 chính (OBC) qua UART2
    if (Serial2.available() > 0) {
        String command = Serial2.readStringUntil('\n');
        command.trim();

        if (command == "p" || command == "P" || command.equalsIgnoreCase("capture") || command.equalsIgnoreCase("photo")) {
            handleOBCCommand();
        }
    }
}
