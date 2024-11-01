# ESP32-CAM Dual-Mode Face Recognition Door Lock System

This project implements a face recognition door lock system using the ESP32-CAM module. The system operates in two modes:

1. **Active Mode**: Automatically recognizes faces and unlocks the door when an authorized face is detected.
2. **Admin Mode**: Provides a web interface for enrolling new faces, managing stored faces, and monitoring the camera stream.

The system utilizes neural networks for face detection and recognition, leveraging the ESP32's capabilities. It also uses WebSockets for real-time communication with a web browser.

## Table of Contents

1. [Introduction](#introduction)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Requirements](#software-requirements)
4. [Libraries and Dependencies](#libraries-and-dependencies)
5. [Overview of the Code](#overview-of-the-code)
6. [Detailed Explanation of the Code](#detailed-explanation-of-the-code)
   - [Global Variables and Constants](#global-variables-and-constants)
   - [Function Declarations](#function-declarations)
   - [Class Definitions](#class-definitions)
     - [ActiveMode Class](#activemode-class)
     - [AdminMode Class](#adminmode-class)
   - [Main Setup and Loop Functions](#main-setup-and-loop-functions)
   - [Active Mode Implementation](#active-mode-implementation)
   - [Admin Mode Implementation](#admin-mode-implementation)
7. [Face Detection and Recognition](#face-detection-and-recognition)
   - [MTCNN Face Detection](#mtcnn-face-detection)
   - [Face Recognition Process](#face-recognition-process)
8. [Neural Network Usage](#neural-network-usage)
9. [Important Functions and Libraries](#important-functions-and-libraries)
10. [Conclusion](#conclusion)
11. [Notes](#notes)

---

## Introduction

This project is a comprehensive face recognition door lock system using the ESP32-CAM module. It enhances security by allowing only authorized individuals to unlock the door using facial recognition. The system operates in two modes:

- **Active Mode**: The system runs autonomously, detecting and recognizing faces to control the door lock.
- **Admin Mode**: An administrator can manage the system via a web interface, enrolling new users, deleting existing ones, and monitoring the camera stream.

## Hardware Requirements

- **ESP32-CAM Module**: For capturing images and processing face recognition.
- **Relay Module**: To control the door lock mechanism.
- **Power Supply**: To power the ESP32-CAM and the relay.
- **Push Button or Switch**: To toggle between Active Mode and Admin Mode (optional).

## Software Requirements

- **Arduino IDE**: For compiling and uploading the code to the ESP32-CAM.
- **ESP32 Board Support**: Install via the Arduino IDE Board Manager.

## Libraries and Dependencies

Ensure the following libraries are installed in your Arduino IDE:

- **ESP32 Camera Driver**
- **ESP32 HTTP Server**
- **ArduinoWebsockets**
- **Preferences**

These are usually included with the ESP32 Board Support Package.

## Overview of the Code

The code is structured to support two modes of operation: Active Mode and Admin Mode. It uses classes to encapsulate the functionality for each mode. The mode is toggled by storing a flag in the ESP32's non-volatile storage, allowing it to switch modes on reset.

- **Active Mode**: Runs face detection and recognition continuously to control the door lock.
- **Admin Mode**: Hosts a web server providing a web interface for administration tasks.

---

## Detailed Explanation of the Code

### Global Variables and Constants

```cpp
const char* ssid = "iPhone";       // Wi-Fi network SSID
const char* password = "12345678"; // Wi-Fi network password

#define relay_pin 2 // GPIO pin connected to the relay

int currentMode = 0; // Mode flag: 0 for active mode, 1 for admin mode

Preferences preferences; // To store the mode flag in non-volatile storage
```

- **ssid and password**: Wi-Fi credentials (replace with your network's SSID and password).
- **relay_pin**: GPIO pin used to control the relay for the door lock.
- **currentMode**: Determines the operating mode of the system.
- **Preferences**: Used to store and retrieve the mode flag across resets.

### Function Declarations

```cpp
esp_err_t index_handler(httpd_req_t *req); // Forward declaration for the HTTP index handler
```

### Class Definitions

#### ActiveMode Class

Handles the Active Mode functionality.

```cpp
class ActiveMode {
public:
  void setup(); // Initializes Active Mode components
  void loop();  // Main loop for Active Mode
private:
  // Variables for camera frame buffer, timing, and face recognition
  camera_fb_t * fb = NULL;
  unsigned long door_opened_millis = 0;
  long interval = 5000;
  mtmn_config_t mtmn_config;
  face_id_name_list st_face_list;
  dl_matrix3du_t * aligned_face = NULL;

  // Private methods
  void app_facenet_main();        // Initializes face recognition
  mtmn_config_t app_mtmn_config();// Configures MTCNN for face detection
  void open_door();               // Controls the relay to unlock the door
};
```

#### AdminMode Class

Handles the Admin Mode functionality.

```cpp
class AdminMode {
public:
  void setup(); // Initializes Admin Mode components
  void loop();  // Main loop for Admin Mode
private:
  // Variables for camera frame buffer, WebSockets server, timing, and face recognition
  camera_fb_t * fb = NULL;
  WebsocketsServer socket_server;
  long current_millis;
  long last_detected_millis = 0;
  unsigned long door_opened_millis = 0;
  long interval = 5000;
  bool face_recognised = false;

  mtmn_config_t mtmn_config;
  face_id_name_list st_face_list;
  dl_matrix3du_t *aligned_face = NULL;
  httpd_handle_t camera_httpd = NULL;

  en_fsm_state g_state = START_RECOGNITION;  // Initial state of the FSM
  httpd_resp_value st_name;                  // Holds enrollment name

  // Private methods
  void app_facenet_main();        // Initializes face recognition
  void app_httpserver_init();     // Initializes the HTTP server
  static mtmn_config_t app_mtmn_config(); // Configures MTCNN for face detection
  static esp_err_t index_handler(httpd_req_t *req); // HTTP index handler
  void handle_message(WebsocketsClient &client, WebsocketsMessage msg); // WebSocket message handler
  void open_door(WebsocketsClient &client); // Controls the relay to unlock the door
  esp_err_t send_face_list(WebsocketsClient &client); // Sends the list of faces to the client
  esp_err_t delete_all_faces(WebsocketsClient &client); // Deletes all stored faces
};
```

### Main Setup and Loop Functions

```cpp
void setup() {
  Serial.begin(115200);
  Serial.println();

  // Initialize Preferences
  preferences.begin("mode", false);
  currentMode = preferences.getInt("mode", 0); // Default to Active Mode (0)

  // Initialize the relay pin
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW); // Ensure the door is locked initially

  if (currentMode == 0) {
    activeMode.setup(); // Initialize Active Mode
  } else {
    adminMode.setup();  // Initialize Admin Mode
  }

  // Toggle mode for next reset
  int nextMode = 1 - currentMode;
  preferences.putInt("mode", nextMode);

  preferences.end();
}

void loop() {
  if (currentMode == 0) {
    activeMode.loop(); // Run Active Mode loop
  } else {
    adminMode.loop();  // Run Admin Mode loop
  }
}
```

- **Preferences**: Stores the mode flag, toggling between modes on each reset.
- **Mode Initialization**: Depending on the mode, initializes the respective components.

### Active Mode Implementation

#### Setup Function

Initializes the camera and face recognition components.

```cpp
void ActiveMode::setup() {
  Serial.println("Starting Active Mode");

  // Camera initialization
  camera_config_t config;
  // [Camera pin assignments and configurations]
  // [Camera frame size and quality settings based on PSRAM availability]

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  mtmn_config = app_mtmn_config(); // Configure MTCNN
  app_facenet_main();              // Initialize face recognition
}
```

#### Loop Function

Continuously captures frames, detects faces, recognizes them, and controls the door lock.

```cpp
void ActiveMode::loop() {
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = {0};
  out_res.image = image_matrix->item;

  fb = esp_camera_fb_get();

  if (fb) {
    fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

    out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

    if (out_res.net_boxes) {
      if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) {
        out_res.face_id = get_face_id(aligned_face);

        if (st_face_list.count > 0) {
          face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
          if (f) {
            Serial.printf("Face recognized: %s\n", f->id_name);
            open_door();
          } else {
            Serial.println("Face not recognized");
          }
        }
        dl_matrix3d_free(out_res.face_id);
      }
    }
    esp_camera_fb_return(fb);
    fb = NULL;
  }

  if (millis() - door_opened_millis > interval) {
    digitalWrite(relay_pin, LOW); // Lock the door after the interval
  }

  dl_matrix3du_free(image_matrix);
}
```

#### open_door Function

Unlocks the door by activating the relay.

```cpp
void ActiveMode::open_door() {
  if (digitalRead(relay_pin) == LOW) {
    digitalWrite(relay_pin, HIGH); // Energize the relay to unlock the door
    Serial.println("Door Unlocked");
    door_opened_millis = millis();
  }
}
```

### Admin Mode Implementation

#### Setup Function

Initializes the camera, Wi-Fi connection, HTTP server, WebSocket server, and face recognition components.

```cpp
void AdminMode::setup() {
  Serial.println("Starting Admin Mode");

  // Initialize relay pin
  digitalWrite(relay_pin, LOW);
  pinMode(relay_pin, OUTPUT);

  // Camera initialization
  camera_config_t config;
  // [Camera pin assignments and configurations]
  // [Camera frame size and quality settings based on PSRAM availability]

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  // Wi-Fi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  app_httpserver_init();    // Initialize the HTTP server
  app_facenet_main();       // Initialize face recognition
  mtmn_config = app_mtmn_config(); // Configure MTCNN
  socket_server.listen(82); // Start WebSocket server on port 82

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}
```

#### Loop Function

Handles WebSocket communication, face detection, face recognition, enrollment, and door control.

```cpp
void AdminMode::loop() {
  if (socket_server.poll()) {
    auto client = socket_server.accept();
    if (client.available()) {
      Serial.println("Client connected");

      client.onMessage([this, &client](WebsocketsMessage msg) {
        handle_message(client, msg);
      });

      dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
      http_img_process_result out_res = {0};
      out_res.image = image_matrix->item;

      send_face_list(client);
      client.send("STREAMING");

      while (client.available()) {
        client.poll();

        if (millis() - door_opened_millis > interval) {
          digitalWrite(relay_pin, LOW); // Lock the door after the interval
        }

        fb = esp_camera_fb_get();
        if (!fb) {
          Serial.println("Camera capture failed");
          continue;
        }

        // Face detection and recognition logic
        // ...

        client.sendBinary((const char *)fb->buf, fb->len);

        esp_camera_fb_return(fb);
        fb = NULL;
      }
      Serial.println("Client disconnected");

      // Clean up
      dl_matrix3du_free(image_matrix);
      client.close();
    } else {
      delay(100);
    }
  } else {
    delay(100);
  }
}
```

#### WebSocket Message Handler

Handles messages from the web interface to control the system's state.

```cpp
void AdminMode::handle_message(WebsocketsClient &client, WebsocketsMessage msg) {
  String data = msg.data();
  Serial.println("Received message: " + data);

  if (data == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }
  else if (data == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  }
  else if (data.startsWith("capture:")) {
    g_state = START_ENROLL;
    String person = data.substring(8);
    person.toCharArray(st_name.enroll_name, sizeof(st_name.enroll_name));
    client.send("CAPTURING");
  }
  // [Other command handling]
}
```

---

## Face Detection and Recognition

### MTCNN Face Detection

The system uses the **Multi-Task Cascaded Convolutional Neural Network (MTCNN)** for real-time face detection. MTCNN consists of three stages:

1. **P-Net (Proposal Network)**: Quickly generates candidate windows (face proposals).
2. **R-Net (Refine Network)**: Refines the candidate windows and rejects a large number of non-faces.
3. **O-Net (Output Network)**: Produces the final bounding boxes with facial landmarks.

The MTCNN configuration parameters in the code determine the sensitivity and performance of face detection.

### Face Recognition Process

1. **Face Alignment**: Detected faces are aligned using facial landmarks to standardize orientation and scale.
2. **Feature Extraction**: An aligned face image is passed through a neural network to extract a high-dimensional feature vector (face ID).
3. **Face Matching**: The extracted face ID is compared with stored face IDs using similarity metrics (e.g., Euclidean distance).
4. **Decision Making**: If a match is found within a threshold, the face is recognized as an authorized user.

---

## Neural Network Usage

- **Face Detection**: Utilizes MTCNN, which is composed of convolutional neural networks trained for detecting faces and facial landmarks.
- **Face Recognition**: Employs a pre-trained neural network model to extract unique features from faces for identification.

Both networks are pre-trained and optimized for the ESP32's limited computational resources.

---

## Important Functions and Libraries

### Libraries

- **esp_camera.h**: Provides camera driver functions for initialization and image capture.
- **esp_timer.h**: Offers timing functions.
- **ArduinoWebsockets.h**: Enables WebSocket communication.
- **esp_http_server.h**: Allows hosting an HTTP server on the ESP32.
- **Preferences.h**: Manages non-volatile storage for saving the mode flag.
- **fd_forward.h, fr_forward.h, fr_flash.h**: Provide face detection, recognition, and flash storage functionalities.

### Key Functions

- **esp_camera_init()**: Initializes the camera with specified configurations.
- **esp_camera_fb_get()**: Captures an image frame from the camera.
- **esp_camera_fb_return()**: Returns the frame buffer to the driver.
- **fmt2rgb888()**: Converts image data to RGB888 format.
- **face_detect()**: Performs face detection on an image.
- **align_face()**: Aligns a detected face for recognition.
- **get_face_id()**: Extracts a feature vector (face ID) from an aligned face.
- **recognize_face_with_name()**: Matches the extracted face ID with stored face IDs.
- **enroll_face_id_to_flash_with_name()**: Saves a new face ID to flash memory.
- **delete_face_id_in_flash_with_name()**: Deletes a face ID from flash memory.

---

## Conclusion

This project demonstrates a practical implementation of a face recognition door lock system using the ESP32-CAM module. By leveraging neural networks for face detection and recognition, it provides a secure method for controlling access. The dual-mode operation allows for both autonomous functioning and administrative control via a web interface.

---

## Notes

- **Security Considerations**: Ensure that the Wi-Fi network is secure, and consider implementing additional security measures if deploying in a sensitive environment.
- **Power Supply**: Use a stable power supply to prevent unexpected resets or performance issues.
- **Hardware Connections**: Carefully wire the relay module to avoid damaging the ESP32-CAM or the door lock mechanism.
- **Legal Compliance**: Be aware of privacy laws and regulations regarding the use of facial recognition technology in your region.

---

**Disclaimer**: This code is provided for educational purposes. The author is not responsible for any misuse or damages resulting from the use of this code.
