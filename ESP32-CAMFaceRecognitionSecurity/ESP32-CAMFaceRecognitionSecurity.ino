#include "esp_camera.h"         // Include the ESP32 camera library
#include "esp_timer.h"          // Include the ESP32 timer library
#include "Arduino.h"            // Include the core Arduino library
#include "fd_forward.h"         // Include face detection forward declarations
#include "fr_forward.h"         // Include face recognition forward declarations
#include "fr_flash.h"           // Include face recognition flash storage functions

#include <Preferences.h>        // Include the Preferences library for storing mode

#include <WiFi.h>               // Include Wi-Fi library
#include <ArduinoWebsockets.h>  // Include WebSockets library
#include "esp_http_server.h"    // Include the HTTP server library
#include "camera_index.h"       // Include the web interface HTML (index_ov2640_html_gz)

#define ENROLL_CONFIRM_TIMES 5  // Number of confirmations required for enrollment
#define FACE_ID_SAVE_NUMBER 100 // Maximum number of faces to save

#define CAMERA_MODEL_AI_THINKER  // Define the camera model as AI Thinker
#include "camera_pins.h"         // Include camera pin definitions

using namespace websockets;     // Use the websockets namespace

// Global constants and variables
const char* ssid = "iPhone";       // Wi-Fi SSID (replace with your network SSID)
const char* password = "12345678"; // Wi-Fi password (replace with your network password)

#define relay_pin 2 // Define the GPIO pin connected to the relay (used in both modes)

// Mode flag: 0 for active mode, 1 for admin mode
int currentMode = 0; // Variable to store the current mode

Preferences preferences; // Create a Preferences object for storing mode in non-volatile storage

// Typedefs and Enums
typedef struct
{
  uint8_t *image;              // Pointer to image data
  box_array_t *net_boxes;      // Pointer to detected face boxes
  dl_matrix3d_t *face_id;      // Pointer to face ID matrix
} http_img_process_result;

typedef enum
{
  START_STREAM,        // State to start video streaming
  START_DETECT,        // State to start face detection
  SHOW_FACES,          // State to show the list of faces
  START_RECOGNITION,   // State to start face recognition
  START_ENROLL,        // State to start face enrollment
  ENROLL_COMPLETE,     // State when enrollment is complete
  DELETE_ALL,          // State to delete all faces
} en_fsm_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN]; // Name used for enrollment
} httpd_resp_value;

// Forward declarations of global functions
esp_err_t index_handler(httpd_req_t *req); // Forward declaration of index handler

// Active Mode Class Definition
class ActiveMode {
public:
  void setup(); // Setup function for Active Mode
  void loop();  // Loop function for Active Mode
private:
  camera_fb_t * fb = NULL;        // Frame buffer pointer
  unsigned long door_opened_millis = 0; // Time when the door was unlocked
  long interval = 5000;           // Duration (in milliseconds) to keep the door unlocked
  mtmn_config_t mtmn_config;      // Configuration for face detection
  face_id_name_list st_face_list; // List of stored face IDs
  dl_matrix3du_t * aligned_face = NULL; // Pointer for aligned face data

  void app_facenet_main();        // Function to initialize face recognition
  mtmn_config_t app_mtmn_config();// Function to configure MTCNN
  void open_door();               // Function to unlock the door
};

// Admin Mode Class Definition
class AdminMode {
public:
  void setup(); // Setup function for Admin Mode
  void loop();  // Loop function for Admin Mode
private:
  camera_fb_t * fb = NULL;            // Frame buffer pointer
  WebsocketsServer socket_server;     // WebSockets server instance
  long current_millis;
  long last_detected_millis = 0;      // Time of the last face detection
  unsigned long door_opened_millis = 0; // Time when the door was unlocked
  long interval = 5000;               // Duration to keep the door unlocked
  bool face_recognised = false;       // Flag indicating if a face was recognized

  mtmn_config_t mtmn_config;          // Configuration for face detection
  face_id_name_list st_face_list;     // List of stored face IDs
  dl_matrix3du_t *aligned_face = NULL;// Pointer for aligned face data
  httpd_handle_t camera_httpd = NULL; // HTTP server handle

  en_fsm_state g_state = START_RECOGNITION;  // Initial state of the FSM
  httpd_resp_value st_name;                  // Structure to hold enrollment name

  void app_facenet_main();        // Function to initialize face recognition
  void app_httpserver_init();     // Function to initialize the HTTP server
  static mtmn_config_t app_mtmn_config(); // Static function to configure MTCNN
  static esp_err_t index_handler(httpd_req_t *req); // Static index handler
  void handle_message(WebsocketsClient &client, WebsocketsMessage msg); // Message handler
  void open_door(WebsocketsClient &client); // Function to unlock the door
  esp_err_t send_face_list(WebsocketsClient &client); // Send face list to client
  esp_err_t delete_all_faces(WebsocketsClient &client); // Delete all faces
};

// Create instances of classes
ActiveMode activeMode; // Instance of ActiveMode
AdminMode adminMode;   // Instance of AdminMode

// Main setup function
void setup() {
  Serial.begin(115200); // Initialize serial communication
  Serial.println();

  // Initialize Preferences
  preferences.begin("mode", false);                   // Open preferences in read-write mode
  currentMode = preferences.getInt("mode", 0);        // Get the current mode from preferences (default to 0)

  // Initialize the relay pin
  pinMode(relay_pin, OUTPUT);                         // Set relay pin as output
  digitalWrite(relay_pin, LOW);                       // Ensure the relay is off (door locked)

  if (currentMode == 0) {
    // Active mode
    activeMode.setup();                               // Call setup for Active Mode
  } else {
    // Admin mode
    adminMode.setup();                                // Call setup for Admin Mode
  }

  // Toggle mode for next reset
  int nextMode = 1 - currentMode;                     // Compute the next mode
  preferences.putInt("mode", nextMode);               // Store the next mode in preferences

  preferences.end();                                  // Close preferences
}

// Main loop function
void loop() {
  if (currentMode == 0) {
    activeMode.loop();                                // Call loop for Active Mode
  } else {
    adminMode.loop();                                 // Call loop for Admin Mode
  }
}

// Active Mode Implementation
void ActiveMode::setup() {
  Serial.println("Starting Active Mode");

  // Camera initialization
  camera_config_t config;                  // Create a camera configuration structure
  config.ledc_channel = LEDC_CHANNEL_0;    // Set LEDC PWM channel
  config.ledc_timer = LEDC_TIMER_0;        // Set LEDC timer
  config.pin_d0 = Y2_GPIO_NUM;             // Assign camera data pins D0-D7
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;         // Assign external clock pin
  config.pin_pclk = PCLK_GPIO_NUM;         // Assign pixel clock pin
  config.pin_vsync = VSYNC_GPIO_NUM;       // Assign vertical sync pin
  config.pin_href = HREF_GPIO_NUM;         // Assign horizontal reference pin
  config.pin_sscb_sda = SIOD_GPIO_NUM;     // Assign SCCB data pin
  config.pin_sscb_scl = SIOC_GPIO_NUM;     // Assign SCCB clock pin
  config.pin_pwdn = PWDN_GPIO_NUM;         // Assign power-down pin
  config.pin_reset = RESET_GPIO_NUM;       // Assign reset pin
  config.xclk_freq_hz = 20000000;          // Set XCLK frequency to 20 MHz
  config.pixel_format = PIXFORMAT_JPEG;    // Set pixel format to JPEG

  if (psramFound()) {                      // Check if PSRAM is available
    config.frame_size = FRAMESIZE_UXGA;    // Set frame size to UXGA (1600x1200)
    config.jpeg_quality = 10;              // Set JPEG quality (lower number = higher quality)
    config.fb_count = 2;                   // Use two frame buffers
  } else {
    config.frame_size = FRAMESIZE_SVGA;    // Set frame size to SVGA (800x600)
    config.jpeg_quality = 12;              // Set JPEG quality
    config.fb_count = 1;                   // Use one frame buffer
  }

  esp_err_t err = esp_camera_init(&config);// Initialize the camera with the configuration
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err); // Print error message if initialization fails
    return;                      // Exit the setup function
  }

  sensor_t * s = esp_camera_sensor_get();  // Get the camera sensor handle
  s->set_framesize(s, FRAMESIZE_QVGA);     // Set frame size to QVGA (320x240)

  mtmn_config = app_mtmn_config();         // Configure MTCNN for face detection
  app_facenet_main();                      // Initialize face recognition components
}

void ActiveMode::app_facenet_main() {
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES); // Initialize the face ID list
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);            // Allocate memory for aligned face data
  read_face_id_from_flash_with_name(&st_face_list);                            // Load saved face IDs from flash memory
}

mtmn_config_t ActiveMode::app_mtmn_config() {
  mtmn_config_t mtmn_config = {0};     // Initialize MTCNN configuration structure
  mtmn_config.type = FAST;             // Set detection type to FAST
  mtmn_config.min_face = 80;           // Set minimum face size to detect
  mtmn_config.pyramid = 0.707;         // Set pyramid scaling factor
  mtmn_config.pyramid_times = 4;       // Set number of pyramid levels
  mtmn_config.p_threshold.score = 0.6; // Set P-Net score threshold
  mtmn_config.p_threshold.nms = 0.7;   // Set P-Net non-maximum suppression threshold
  mtmn_config.p_threshold.candidate_number = 20; // Set P-Net candidate number
  mtmn_config.r_threshold.score = 0.7; // Set R-Net score threshold
  mtmn_config.r_threshold.nms = 0.7;   // Set R-Net non-maximum suppression threshold
  mtmn_config.r_threshold.candidate_number = 10; // Set R-Net candidate number
  mtmn_config.o_threshold.score = 0.7; // Set O-Net score threshold
  mtmn_config.o_threshold.nms = 0.7;   // Set O-Net non-maximum suppression threshold
  mtmn_config.o_threshold.candidate_number = 1;  // Set O-Net candidate number
  return mtmn_config;                   // Return the configured MTCNN settings
}

void ActiveMode::open_door() {
  if (digitalRead(relay_pin) == LOW) {  // Check if the relay is currently off
    digitalWrite(relay_pin, HIGH);      // Close relay to unlock the door
    Serial.println("Door Unlocked");
    door_opened_millis = millis();      // Record the time when the door was unlocked
  }
}

void ActiveMode::loop() {
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3); // Allocate memory for the image matrix
  http_img_process_result out_res = {0};     // Initialize image processing result structure
  out_res.image = image_matrix->item;        // Set the image pointer to the image matrix data

  fb = esp_camera_fb_get();                  // Capture a frame from the camera

  if (fb) {
    fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image); // Convert the frame buffer to RGB888 format

    out_res.net_boxes = face_detect(image_matrix, &mtmn_config); // Perform face detection on the image

    if (out_res.net_boxes) {
      if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) { // Align the detected face
        out_res.face_id = get_face_id(aligned_face); // Extract face ID from the aligned face

        if (st_face_list.count > 0) {                // Check if there are stored faces
          face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id); // Recognize the face
          if (f) {
            Serial.printf("Face recognized: %s\n", f->id_name); // Print recognized face name
            open_door();                                       // Unlock the door
          } else {
            Serial.println("Face not recognized");             // Face not recognized
          }
        }
        dl_matrix3d_free(out_res.face_id);         // Free the face ID matrix memory
      }
    }
    esp_camera_fb_return(fb); // Return the frame buffer to the driver
    fb = NULL;                // Reset the frame buffer pointer
  }

  if (millis() - door_opened_millis > interval) {
    digitalWrite(relay_pin, LOW); // Open relay to lock the door after interval
  }

  dl_matrix3du_free(image_matrix); // Free image matrix memory
}

// Admin Mode Implementation
void AdminMode::setup() {
  Serial.println("Starting Admin Mode");

  // Initialize relay pin
  digitalWrite(relay_pin, LOW);   // Ensure the relay is off (door locked)
  pinMode(relay_pin, OUTPUT);     // Set relay pin as output

  // Camera initialization
  camera_config_t config;                  // Create a camera configuration structure
  config.ledc_channel = LEDC_CHANNEL_0;    // Set LEDC PWM channel
  config.ledc_timer = LEDC_TIMER_0;        // Set LEDC timer
  config.pin_d0 = Y2_GPIO_NUM;             // Assign camera data pins D0-D7
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;         // Assign external clock pin
  config.pin_pclk = PCLK_GPIO_NUM;         // Assign pixel clock pin
  config.pin_vsync = VSYNC_GPIO_NUM;       // Assign vertical sync pin
  config.pin_href = HREF_GPIO_NUM;         // Assign horizontal reference pin
  config.pin_sscb_sda = SIOD_GPIO_NUM;     // Assign SCCB data pin
  config.pin_sscb_scl = SIOC_GPIO_NUM;     // Assign SCCB clock pin
  config.pin_pwdn = PWDN_GPIO_NUM;         // Assign power-down pin
  config.pin_reset = RESET_GPIO_NUM;       // Assign reset pin
  config.xclk_freq_hz = 20000000;          // Set XCLK frequency to 20 MHz
  config.pixel_format = PIXFORMAT_JPEG;    // Set pixel format to JPEG
  // Initialize with high specs to pre-allocate larger buffers
  if (psramFound()) {                      // Check if PSRAM is available
    config.frame_size = FRAMESIZE_UXGA;    // Set frame size to UXGA (1600x1200)
    config.jpeg_quality = 10;              // Set JPEG quality (lower number = higher quality)
    config.fb_count = 2;                   // Use two frame buffers
  } else {
    config.frame_size = FRAMESIZE_SVGA;    // Set frame size to SVGA (800x600)
    config.jpeg_quality = 12;              // Set JPEG quality
    config.fb_count = 1;                   // Use one frame buffer
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);// Initialize the camera with the configuration
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err); // Print error message if initialization fails
    return;                      // Exit the setup function
  }

  sensor_t * s = esp_camera_sensor_get();  // Get the camera sensor handle
  s->set_framesize(s, FRAMESIZE_QVGA);     // Set frame size to QVGA (320x240)

  WiFi.begin(ssid, password);             // Begin Wi-Fi connection
  while (WiFi.status() != WL_CONNECTED) { // Wait until connected
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  app_httpserver_init();                  // Initialize the HTTP server
  app_facenet_main();                     // Initialize face recognition components
  mtmn_config = app_mtmn_config();        // Configure MTCNN for face detection
  socket_server.listen(82);               // Start WebSocket server on port 82

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());           // Print the IP address
  Serial.println("' to connect");
}

void AdminMode::app_httpserver_init() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();    // Use default HTTP server configuration
  if (httpd_start(&camera_httpd, &config) == ESP_OK) // Start the HTTP server
  {
    Serial.println("HTTP Server started");
    httpd_uri_t index_uri = {
      .uri       = "/",             // Match the root URI "/"
      .method    = HTTP_GET,        // Handle GET requests
      .handler   = index_handler,   // Function to handle the request
      .user_ctx  = NULL             // No user context
    };
    httpd_register_uri_handler(camera_httpd, &index_uri); // Register the root URI handler
  }
}

esp_err_t AdminMode::index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");          // Set response content type to HTML
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip"); // Set header for gzip encoding
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len); // Send the webpage
}

void AdminMode::app_facenet_main() {
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES); // Initialize the face ID list
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);            // Allocate memory for aligned face data
  read_face_id_from_flash_with_name(&st_face_list);                            // Load saved face IDs from flash memory
}

mtmn_config_t AdminMode::app_mtmn_config() {
  mtmn_config_t mtmn_config = {0};     // Initialize MTCNN configuration structure
  mtmn_config.type = FAST;             // Set detection type to FAST
  mtmn_config.min_face = 80;           // Set minimum face size to detect
  mtmn_config.pyramid = 0.707;         // Set pyramid scaling factor
  mtmn_config.pyramid_times = 4;       // Set number of pyramid levels
  mtmn_config.p_threshold.score = 0.6; // Set P-Net score threshold
  mtmn_config.p_threshold.nms = 0.7;   // Set P-Net non-maximum suppression threshold
  mtmn_config.p_threshold.candidate_number = 20; // Set P-Net candidate number
  mtmn_config.r_threshold.score = 0.7; // Set R-Net score threshold
  mtmn_config.r_threshold.nms = 0.7;   // Set R-Net non-maximum suppression threshold
  mtmn_config.r_threshold.candidate_number = 10; // Set R-Net candidate number
  mtmn_config.o_threshold.score = 0.7; // Set O-Net score threshold
  mtmn_config.o_threshold.nms = 0.7;   // Set O-Net non-maximum suppression threshold
  mtmn_config.o_threshold.candidate_number = 1;  // Set O-Net candidate number
  return mtmn_config;                   // Return the configured MTCNN settings
}

void AdminMode::handle_message(WebsocketsClient &client, WebsocketsMessage msg) {
  String data = msg.data();             // Get the message data
  Serial.println("Received message: " + data);

  if (data == "stream") {
    g_state = START_STREAM;             // Set state to start streaming
    client.send("STREAMING");           // Inform client
  }
  else if (data == "detect") {
    g_state = START_DETECT;             // Set state to start detection
    client.send("DETECTING");           // Inform client
  }
  else if (data.startsWith("capture:")) {
    g_state = START_ENROLL;             // Set state to start enrollment
    String person = data.substring(8);  // Extract person name
    person.toCharArray(st_name.enroll_name, sizeof(st_name.enroll_name)); // Copy name for enrollment
    client.send("CAPTURING");           // Inform client
  }
  else if (data == "recognise") {
    g_state = START_RECOGNITION;        // Set state to start recognition
    client.send("RECOGNISING");         // Inform client
  }
  else if (data.startsWith("remove:")) {
    String person = data.substring(7);  // Extract person name
    char person_name[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    person.toCharArray(person_name, sizeof(person_name));
    delete_face_id_in_flash_with_name(&st_face_list, person_name); // Delete face ID
    send_face_list(client);             // Reset faces in the browser
  }
  else if (data == "delete_all") {
    delete_all_faces(client);           // Delete all faces
  }
}

esp_err_t AdminMode::send_face_list(WebsocketsClient &client) {
  client.send("delete_faces"); // Tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) { // Loop through current faces
    sprintf(add_face, "listface:%s", head->id_name); // Format the face name
    client.send(add_face); // Send face to browser
    head = head->next;
  }
  return ESP_OK;
}

esp_err_t AdminMode::delete_all_faces(WebsocketsClient &client) {
  delete_face_all_in_flash_with_name(&st_face_list); // Delete all faces from flash
  client.send("delete_faces"); // Inform the client to delete faces
  return ESP_OK;
}

void AdminMode::open_door(WebsocketsClient &client) {
  if (digitalRead(relay_pin) == LOW) {
    digitalWrite(relay_pin, HIGH);      // Close (energize) relay so door unlocks
    Serial.println("Door Unlocked");
    client.send("door_open");           // Inform client
    door_opened_millis = millis();      // Record the time when the door was unlocked
  }
}

void AdminMode::loop() {
  if (socket_server.poll()) {           // Check for new clients
    auto client = socket_server.accept(); // Accept a new WebSocket client
    if (client.available()) {
      Serial.println("Client connected");

      // Capture 'this' and 'client' in the lambda
      client.onMessage([this, &client](WebsocketsMessage msg) {
        handle_message(client, msg);    // Set the message handler
      });

      dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3); // Allocate memory for the image matrix
      http_img_process_result out_res = {0};     // Initialize image processing result structure
      out_res.image = image_matrix->item;        // Set the image pointer to the image matrix data

      send_face_list(client);                // Send the list of faces to the client
      client.send("STREAMING");              // Inform the client that streaming has started

      while (client.available()) {           // While client is connected
        client.poll();                       // Process incoming messages

        if (millis() - door_opened_millis > interval) { // Close door after interval
          digitalWrite(relay_pin, LOW); // Open relay to lock the door
        }

        fb = esp_camera_fb_get();            // Capture a frame from the camera
        if (!fb) {
          Serial.println("Camera capture failed");
          continue;
        }

        if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION) {
          out_res.net_boxes = NULL;          // Reset detection results
          out_res.face_id = NULL;

          fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image); // Convert the frame buffer to RGB888 format

          out_res.net_boxes = face_detect(image_matrix, &mtmn_config); // Perform face detection on the image

          if (out_res.net_boxes) {
            if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) { // Align the detected face
              out_res.face_id = get_face_id(aligned_face); // Extract face ID from the aligned face
              last_detected_millis = millis();             // Update the last detected time
              if (g_state == START_DETECT) {
                client.send("FACE DETECTED");              // Inform client
              }

              if (g_state == START_ENROLL) {
                int left_sample_face = enroll_face_id_to_flash_with_name(&st_face_list, out_res.face_id, st_name.enroll_name); // Enroll the face ID
                char enrolling_message[64];
                sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
                client.send(enrolling_message);           // Inform client of enrollment progress
                if (left_sample_face == 0) {
                  Serial.printf("Enrolled Face ID: %s\n", st_face_list.tail->id_name);
                  g_state = START_STREAM;                // Reset state to streaming
                  char captured_message[64];
                  sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
                  client.send(captured_message);         // Inform client that enrollment is complete
                  send_face_list(client);                // Update face list on client side
                }
              }

              if (g_state == START_RECOGNITION  && (st_face_list.count > 0)) {
                face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id); // Recognize the face
                if (f) {
                  char recognised_message[64];
                  sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
                  open_door(client);                     // Unlock the door
                  client.send(recognised_message);       // Inform client
                } else {
                  client.send("FACE NOT RECOGNISED");    // Inform client that face was not recognized
                }
              }
              dl_matrix3d_free(out_res.face_id);         // Free the face ID matrix memory
            }
          } else {
            if (g_state != START_DETECT) {
              client.send("NO FACE DETECTED");           // Inform client that no face was detected
            }
          }

          if (g_state == START_DETECT && millis() - last_detected_millis > 500) {
            client.send("DETECTING");                   // Inform client
          }
        }

        client.sendBinary((const char *)fb->buf, fb->len); // Send the frame buffer to the client

        esp_camera_fb_return(fb);                   // Return the frame buffer to the driver
        fb = NULL;                                  // Reset the frame buffer pointer
      }
      Serial.println("Client disconnected");

      // Clean up
      dl_matrix3du_free(image_matrix);             // Free image matrix memory
      client.close();                              // Close the client connection
    } else {
      // No client connected
      delay(100);                                  // Delay before next poll
    }
  } else {
    // No new client, delay before next poll
    delay(100);                                    // Delay before next poll
  }
}