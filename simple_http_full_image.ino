////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reference :                                                                                                                                                //
// - esp32cam-gdrive : https://github.com/gsampallo/esp32cam-gdrive                                                                                           //
// - ESP32 CAM Send Images to Google Drive, IoT Security Camera : https://www.electroniclinic.com/esp32-cam-send-images-to-google-drive-iot-security-camera/  //
// - esp32cam-google-drive : https://github.com/RobertSasak/esp32cam-google-drive                                                                             //
//                                                                                                                                                            //
// When the ESP32-CAM takes photos or the process of sending photos to Google Drive is in progress, the ESP32-CAM requires a large amount of power.           //
// So I suggest that you use a 5V power supply with a current of approximately 2A.        

// ngrok http --scheme=http 8080
// python proxy_server.py

// PIN = 8578
// Mob = +306976530203
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//======================================== Including the libraries.
//========================================
// #include "soc/soc.h"
// #include "soc/rtc_cntl_reg.h"
#include "Base64.h"
#include <Arduino.h>
#include "esp_camera.h"
#include <HardwareSerial.h>
//======================================== 
//======================================== Enter your WiFi ssid and password.
#define USE_WIFI false
#define CHUNKED true
// #if USE_WIFI
// #endif
#include <WiFi.h>
#include <WiFiClientSecure.h>               
WiFiClientSecure client;                        // Initialize WiFiClientSecure.
const char* ssid = "COSMOTE-0Akati-2.4Ghz";      
const char* password = "skins2023";

// const char *ssid = "Electronics-Lab G";
// const char *password = "ElectronicsLab";

// Pin definition for ESP32-S3-SIM7670G-4G
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     34
#define SIOD_GPIO_NUM     15
#define SIOC_GPIO_NUM     16
#define Y9_GPIO_NUM       14
#define Y8_GPIO_NUM       13
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       11
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       9
#define Y3_GPIO_NUM       8
#define Y2_GPIO_NUM       7
#define VSYNC_GPIO_NUM    36
#define HREF_GPIO_NUM     35
#define PCLK_GPIO_NUM     37
//======================================== 

#define uS_TO_S_FACTOR 1000000ULL  // Convert microseconds to seconds
#define TIME_TO_SLEEP 86400        // Sleep for 24 hours

// 📡 Modem Serial Pins
#define TX_PIN 18
#define RX_PIN 17
#define PWRKEY 4  // Controls A7670E Power Key
HardwareSerial modemSerial(2);


// const int gsmResetPin = 7; // Example pin connected to GSM RST

//======================================== Replace with your "Deployment ID" and Folder Name.
String myDeploymentID = "AKfycbyYNT6qnNAIhtb_9cfscGDbtwG3_ohSYogX5lxrM4_KdruKmSNtSgayK61uiJU3Od-Z"; 
// String myDeploymentID = "AKfycbyQ4drDz8IwYEQqxZdBkS_9uSMRhdYvsJzTyar6lhTnFsD5VmHizIVBo8gQO2eUIvrw"; //AKfycbyYNT6qnNAIhtb_9cfscGDbtwG3_ohSYogX5lxrM4_KdruKmSNtSgayK61uiJU3Od-Z

String myMainFolderName = "ESP32-Data-Logging";
//======================================== 

//________________________________________________________________________________ Test_Con()
// This subroutine is to test the connection to "script.google.com".
void Test_Con() {
  const char* host = "script.google.com";
  while(1) {
    Serial.println("-----------");
    Serial.println("Connection Test...");
    Serial.println("Connect to " + String(host));
  
    client.setInsecure();
  
    if (client.connect(host, 443)) {
      Serial.println("Connection successful.");
      Serial.println("-----------");
      client.stop();
      break;
    } else {
      Serial.println("Connected to " + String(host) + " failed.");
      Serial.println("Wait a moment for reconnecting.");
      Serial.println("-----------");
      client.stop();
    }
  
    delay(1000);
  }
}
//________________________________________________________________________________ 

//________________________________________________________________________________ SendCapturedPhotos()
// Subroutine for capturing and sending photos to Google Drive.
void SendCapturedPhotos() {
  const char* host = "script.google.com";
  Serial.println();
  Serial.println("-----------");
  Serial.println("Connect to " + String(host));

  client.setInsecure();  

  if (client.connect(host, 443)) {
    Serial.println("✅ WiFi connection successful.");

    Serial.println();
    Serial.println("📷 Taking a photo...");

    for (int i = 0; i <= 3; i++) {
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("❌ Camera capture failed");
        delay(1000);
        // ESP.restart();
        return;
      }
      esp_camera_fb_return(fb);
      delay(200);
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("❌ Final photo capture failed");
      delay(1000);
      // ESP.restart();
      return;
    }

    Serial.println("✅ Photo capture complete.");
    Serial.println("Image Size: " + String(fb->len) + " bytes");

    String url = "/macros/s/" + myDeploymentID + "/exec?folder=" + myMainFolderName;

    client.println("POST " + url + " HTTP/1.1");
    client.println("Host: " + String(host));
    client.println("Transfer-Encoding: chunked");
    client.println();

    int fbLen = fb->len;
    char *input = (char *)fb->buf;
    int chunkSize = 3 * 1000;
    int chunkBase64Size = base64_enc_len(chunkSize);
    char output[chunkBase64Size + 1];

    Serial.println("📤 Sending (and logging) raw HTTP chunked data:");

    int chunk = 0;
    for (int i = 0; i < fbLen; i += chunkSize) {
      int len = min(fbLen - i, chunkSize);
      int base64Len = base64_encode(output, input, len);
      output[base64Len] = '\0';

      // 🔽 This is the RAW DATA sent
      String hexLen = String(base64Len, HEX);
      client.print(hexLen); client.print("\r\n");
      client.print(output); client.print("\r\n");

      // 🪵 Serial debug: Raw HTTP chunk
      Serial.println("⬇️ --------- Raw Chunk " + String(chunk) + " ---------");
      Serial.println(hexLen);
      Serial.println(output);
      Serial.println();
      
      input += chunkSize;
      delay(100);
      chunk++;
    }

    // End of chunked transfer
    client.print("0\r\n\r\n");
    Serial.println("0\r\n");

    Serial.println("✅ Finished sending all chunks.");
    esp_camera_fb_return(fb);

    // Wait for response
    Serial.println("⏳ Waiting for Google Drive response...");
    long int StartTime = millis();
    while (!client.available()) {
      Serial.print(".");
      delay(100);
      if ((StartTime + 10000) < millis()) {
        Serial.println();
        Serial.println("❌ Timeout: No response from server.");
        break;
      }
    }

    Serial.println();
    while (client.available()) {
      Serial.print(char(client.read()));
    }

  } else {
    Serial.println("❌ Failed to connect to " + String(host));
  }

  Serial.println("-----------");
  client.stop();
}
//________________________________________________________________________________ 

String readModemResponse(unsigned long timeout = 1000) {
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (modemSerial.available()) {
      response += (char)modemSerial.read();
    }
  }
  return response;
}

bool checkModemResponse(String expectedResponse) {
    String response = "";
    unsigned long startMillis = millis();

    while (millis() - startMillis < 5000) { // Read response for 5 seconds
        while (modemSerial.available()) {
            response += (char)modemSerial.read();
        }
    }
    response.trim();
    Serial.println("Modem Response: " + response);

    return response.indexOf(expectedResponse) != -1;
}

bool connectToGSM(const char* apn, const char* pin) {

  unsigned long startTime;
  Serial.println("📶 GSM module setup...");
  // Power on the modem
    // powerOnModem(); delay(2000); 
  // Send AT
    //Serial.println("- Sending AT..."); 
    modemSerial.println("AT"); delay(50);  Serial.println(readModemResponse());
  // Set Full functionality  
    //Serial.println("- Setting full functionality (AT+CFUN=1)...");  
    modemSerial.println("AT+CFUN=1"); delay(50);  Serial.println(readModemResponse());
  // Check SIM status
    //Serial.println("- Checking SIM status (AT+CPIN?)...");
    modemSerial.println("AT+CPIN?"); delay(50); 
    String simStatus = readModemResponse(); Serial.println(simStatus);
    if (simStatus.indexOf("SIM PIN") != -1) {    
      //Serial.println("🔐 SIM requires PIN. Unlocking...");
      modemSerial.println("AT+CPIN=\"" + String(pin) + "\"");
      delay(3000);
      //Serial.println("🔓 SIM unlocked.");
      Serial.println(readModemResponse());
  }


  // Check signal quality
    //Serial.println("- Checking signal quality (AT+CSQ)...");
    // modemSerial.println("AT+CSQ?");  delay(1000); Serial.println(readModemResponse());

  // Wait for network registration
    //Serial.println("- Waiting for network registration (AT+CREG?)...");

  const int maxAttempts = 3;
  int attempt = 0;

  while (attempt < maxAttempts) {    
    modemSerial.println("AT+CREG?"); delay(50); String reg = readModemResponse(); //Serial.println(reg);

    if (reg.indexOf("+CREG: 0,1") != -1 || reg.indexOf("+CREG: 0,5") != -1) {
      Serial.println("✅ Network registered."); Serial.println();
      break;
    }
    // Get signal info on failure
      modemSerial.println("AT+CSQ"); delay(50);
      String signal = readModemResponse(); Serial.println("📶 Signal: " + signal);
    
    Serial.println("⚠️ Retrying... (Attempt=" + String(attempt+1) +")");     
    delay(1000);
    attempt++;  
  }

  // Final check
  if (attempt >= maxAttempts) {
    Serial.println("❌ Network registration failed.");
    // ESP.restart();
    // return false; 
  } 
  
  // Set APN
  //Serial.println("- Setting APN: " + String(apn));
  modemSerial.println("AT+CGDCONT=1,\"IP\",\"" + String(apn) + "\"");
  delay(50); Serial.println(readModemResponse());

  // Activate PDP context
  //Serial.println("- Activating PDP context (AT+CGACT=1,1)...");
  modemSerial.println("AT+CGACT=1,1");
  delay(50); Serial.println(readModemResponse());

  // // Optional: check IP
  // modemSerial.println("AT+CGPADDR=1");
  // delay(1000); Serial.println(readModemResponse());

  Serial.println("✅ PDP context activated. Ready for HTTP/MQTT/etc."); Serial.println(""); 

  Serial.println("✅ GSM module is ready to use!"); Serial.println(""); 

  return true;
}

  // const String url = "http://618a-2a02-587-2a10-d400-f483-83cd-5a0e-9795.ngrok-free.app/upload"; // add "/upload"

// int calculateChunkedPostLength(size_t imgLen, int chunkSize) {
//   int base64TotalLen = base64_enc_len(imgLen);  // Total encoded image length
//   int totalLen = 0;

//   // Add "image=" as a chunk
//   String imagePrefix = "image=";
//   totalLen += String(imagePrefix.length(), HEX).length();  // e.g. "6"
//   totalLen += 2; // \r\n after length
//   totalLen += imagePrefix.length(); // "image="
//   totalLen += 2; // \r\n after data

//   // Now add base64 chunks
//   size_t remaining = imgLen;
//   while (remaining > 0) {
//     int binChunkLen = min((int)remaining, chunkSize);
//     int base64ChunkLen = base64_enc_len(binChunkLen);

//     totalLen += String(base64ChunkLen, HEX).length();  // hex header
//     totalLen += 2; // \r\n after header
//     totalLen += base64ChunkLen;
//     totalLen += 2; // \r\n after data

//     remaining -= binChunkLen;
//   }

//   // Add final chunk: "0\r\n\r\n"
//   totalLen += 5;

//   return totalLen;
// }


int calculateChunkedPostLength(size_t imgLen, int chunkSize) {
  const String prefix = "image=";
  int total = 0;

  // —–– 1) the “image=” chunk header + body
  int prefLen = prefix.length();
  total += String(prefLen, HEX).length(); // hex digits for length
  total += 2;                             // "\r\n"
  total += prefLen;                       // the "image=" itself
  total += 2;                             // "\r\n"

  // —–– 2) each base64‐encoded chunk
  size_t remaining = imgLen;
  while (remaining > 0) {
    // how many raw bytes we’ll base64-encode in this chunk
    size_t binChunk = min(remaining, (size_t)chunkSize);
    // how long its base64 string will be
    int encLen = base64_enc_len(binChunk);

    total += String(encLen, HEX).length(); // hex digits for this chunk
    total += 2;                            // "\r\n"
    total += encLen;                       // the base64 text
    total += 2;                            // "\r\n"

    remaining -= binChunk;
  }

  // —–– 3) the final zero‐length chunk “0\r\n\r\n”
  total += 1; // the single "0"
  total += 2; // "\r\n"
  total += 2; // "\r\n"

  return total;
}


bool sendPhotoOverGSM(uint8_t* imgData, size_t imgLen) {
  // const String proxyURL = "http://618a-2a02-587-2a10-d400-f483-83cd-5a0e-9795.ngrok-free.app/upload"; // Replace with actual URL
  const String proxyURL = "http://24c7-2a02-587-2a0c-5f00-8d3a-86f9-6206-3711.ngrok-free.app/upload"; // Replace with actual URL
    
  const int chunkSize = 3 * 1600;
  const int base64ChunkSize = base64_enc_len(chunkSize);
  int httpDataLen = calculateChunkedPostLength(imgLen, chunkSize) +73; // 2*
  char encodedChunk[base64ChunkSize + 1];

  Serial.println();
  Serial.println("-----------");
  Serial.println("📡 Starting GSM image upload (chunked)");

  // Initialize HTTP
  modemSerial.println("AT+HTTPTERM"); delay(300);
  modemSerial.println("AT+HTTPINIT"); delay(300);
  if (!checkModemResponse("OK")) return false;

  modemSerial.println("AT+HTTPPARA=\"URL\",\"" + proxyURL + "\""); delay(300);
  modemSerial.println("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\""); delay(300);
  if (!checkModemResponse("OK")) return false;

  // Rough estimate of full size
  // int totalBase64 = base64_enc_len(imgLen);
  // int estimatedLen = totalBase64 + 32; // buffer overhead
  modemSerial.println("AT+HTTPDATA=" + String(httpDataLen) + ",10000");
  // modemSerial.println("AT+HTTPDATA=" + String(2*httpDataLen) + ",10000");
  delay(300);
  if (!modemSerial.find("DOWNLOAD")) {
    Serial.println("❌ DOWNLOAD prompt not received");
    return false;
  }

  // Start manual chunked HTTP POST body
  modemSerial.println("POST /upload HTTP/1.1");
  modemSerial.println("Host: script.google.com");
  modemSerial.println("Transfer-Encoding: chunked");
  modemSerial.println();  // End of headers
  delay(100);

  // Step 1: Send "image=" chunk
  String prefix = "image=";
  modemSerial.print(prefix.length(), HEX); modemSerial.print("\r\n");
  modemSerial.print(prefix); modemSerial.print("\r\n");

  Serial.println("⬇️ --------- Chunk 0 ---------");
  Serial.println(String(prefix.length(), HEX));
  Serial.println(prefix);
  Serial.println();

  // Step 2: Send base64 chunks
  uint8_t* ptr = imgData;
  size_t remaining = imgLen;
  int chunkIndex = 1;

  while (remaining > 0) {
    int len = min((int)remaining, chunkSize);
    int encLen = base64_encode(encodedChunk, (char*)ptr, len);
    encodedChunk[encLen] = '\0';

    String hexLen = String(encLen, HEX);
    modemSerial.print(hexLen); delay(500);
    modemSerial.print("\r\n"); delay(500);
    modemSerial.print(encodedChunk); delay(500);
    modemSerial.print("\r\n"); delay(500);

    Serial.println("⬇️ --------- Chunk " + String(chunkIndex) + " ---------");
    Serial.println(hexLen);
    Serial.println(encodedChunk);
    Serial.println();

    ptr += len;
    remaining -= len;
    chunkIndex++;
    
  }

  // Final chunk
  modemSerial.print("0\r\n\r\n");
  Serial.println("0\r\n");
  Serial.println("✅ Finished sending all chunks");

  // **then** wait for the modem itself to reply “OK”:
  // if (!modemSerial.find("OK")) {
  //   Serial.println("❌ HTTPDATA never completed");
  //   return false;
  // }

  // Trigger HTTP POST
  modemSerial.println("AT+HTTPACTION=1");
  Serial.println("⏳ Waiting for +HTTPACTION response...");

  String response = "", line = "";
  unsigned long startTime = millis();
  bool received = false;

  while (millis() - startTime < 20000) {
    if (modemSerial.available()) {
      char c = modemSerial.read();
      response += c;
      Serial.write(c);
      if (c == '\n') {
        if (line.startsWith("+HTTPACTION:")) {
          received = true;
          break;
        }
        line = "";
      } else {
        line += c;
      }
    }
  }

  Serial.println();

  if (!received) {
    Serial.println("❌ No +HTTPACTION received");
    return false;
  }

  if (response.indexOf("+HTTPACTION: 1,200") != -1) {
    Serial.println("✅ Upload successful (HTTP 200)");
  } else {
    Serial.println("❌ Upload failed");
    Serial.println(response);
    return false;
  }

  // Optional: Read server response
  // modemSerial.println("AT+HTTPREAD"); delay(2000);
  // Serial.println(readModemResponse());

  modemSerial.println("AT+HTTPTERM"); delay(300);
  Serial.println(readModemResponse());

  return true;
}

void SendCapturedPhotosViaGSM() {
  Serial.println("📷 Capturing image...");
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Camera capture failed!");
    //ESP.restart();
    return;
  }

  Serial.println("✅ Image captured, size: " + String(fb->len));  Serial.println(""); 
  

  sendPhotoOverGSM(fb->buf, fb->len);
  // if (!sendPhotoOverGSM(fb->buf, fb->len)) {
  //   Serial.println("❌ Failed to send image via GSM");
  // } else {
  //   Serial.println("✅ Image sent successfully");
  // }

  esp_camera_fb_return(fb);
}


/**
 * 📷 Setup camera
 */
void setupCamera() {
    Serial.println("📷 Initializing camera...");
    camera_config_t config;
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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // Selectable camera resolution details :
    // -UXGA   = 1600 x 1200 pixels
    // -SXGA   = 1280 x 1024 pixels
    // -XGA    = 1024 x 768  pixels
    // -SVGA   = 800 x 600   pixels
    // -VGA    = 640 x 480   pixels
    // -CIF    = 352 x 288   pixels
    // -QVGA   = 320 x 240   pixels
    // -HQVGA  = 240 x 160   pixels
    // -QQVGA  = 160 x 120   pixels

    if (psramFound()) {
        Serial.println("PSRAM detected, using high-res mode.");
        config.frame_size = FRAMESIZE_VGA; // FRAMESIZE_UXGA FRAMESIZE_VGA
        config.jpeg_quality = 20; // 12 20
        config.fb_count = 2;
        //config.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        Serial.println("No PSRAM detected, using low-res mode.");
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count = 1;
    }

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("❌ Camera init failed! Restarting...");
        // ESP.restart(); // Restart if camera fails
    }

    //calibrateCamera();
    Serial.println("✅ Camera ready!");  
}

/**
 * 🌅 Optimize camera settings for better brightness
 */
void calibrateCamera() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        Serial.println("❌ Camera sensor not found!");
        return;
    }

    // 🌞 Adjust brightness (-2 to 2)
    s->set_brightness(s, 2);  // Higher values = Brighter image

    // 🎭 Adjust contrast (-2 to 2)
    s->set_contrast(s, 2);

    // 🌡️ Adjust white balance
    s->set_whitebal(s, 1);  // 0: Disable, 1: Enable

    // 🚀 Auto Exposure (AE) settings
    s->set_exposure_ctrl(s, 1);  // 0: Manual, 1: Auto
    s->set_aec2(s, 1);           // 0: Disable, 1: Enable

    // 🔦 Adjust gain for better exposure
    s->set_gain_ctrl(s, 0);      // 0: Manual, 1: Auto
    s->set_agc_gain(s, 12);      // Increase for brighter images (0-30)

    // 🌅 Adjust AEC (Auto Exposure Compensation)
    s->set_aec_value(s, 800);    // Lower = Brighter, Higher = Darker (0-1200) - Increase exposure 

    // 🔆 Set White Balance Mode        
    s->set_wb_mode(s, 1);  // 0: Auto, 1: Sunny, 2: Cloudy, 3: Office, 4: Home

    // 🚫 Disable Noise Reduction if necessary
    s->set_denoise(s, 0);  // 0: Off, 1: On

    // 🌓 Enable Low-Light Mode
    s->set_lenc(s, 1);  // Enable Lens Correction
    s->set_special_effect(s, 0);  // No special effects
    s->set_hmirror(s, 0);  // No horizontal flip
    s->set_vflip(s, 0);    // No vertical flip

    Serial.println("✅ Camera calibration done!");
}

/**
 * 📶 Connect to WiFi 🌐
 */
void setupWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) { // Avoid infinite loop
        delay(1000);
        Serial.println(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi Connected!");
    } else {
        Serial.println("\n❌ Failed to connect. Restarting...");
        //ESP.restart();  // Restart ESP if WiFi fails
    }
}

//________________________________________________________________________________ VOID SETUP()
void setup() {
  // put your setup code here, to run once:
 
  Serial.begin(115200);
  Serial.println();
  delay(1000);

  setupCamera();

  if (USE_WIFI) {
      setupWiFi();
      Test_Con();
            
    } else {
      // setupWiFi();
      // Test_Con();
      // initializeModem();
      modemSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
      if (!connectToGSM("internet","8578")){
        Serial.println("❌ GSM connection failed!");
        // ESP.restart();
      }
      
      
      delay(2000);
    }

  //---------------------------------------- 
  // 


}
//________________________________________________________________________________ 

//________________________________________________________________________________ VOID LOOP()
void loop() {
  // put your main code here, to run repeatedly:


    if (USE_WIFI) {
      SendCapturedPhotos();
    } else {
      // SendCapturedPhotos();
      SendCapturedPhotosViaGSM();
    }
  
  
  // 💤 Go-to-sleep
    Serial.println("💤 Going to sleep for 24h...");
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();  
  //---------------------------------------- 
}
//________________________________________________________________________________ 
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 