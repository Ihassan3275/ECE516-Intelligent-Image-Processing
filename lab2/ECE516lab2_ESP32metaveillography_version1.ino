/*****ECE516-LAB2*****/

//#include "Arduino.h"
//#include "FS.h"                // SD Card ESP32
//#include "SD_MMC.h"            // SD Card ESP32

#include "esp_camera.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include "dsps_dotprod.h"
#include <EEPROM.h> 


// Pin definition for CAMERA_MODEL_AI_THINKER
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

#define led_pin 4

camera_fb_t * fb = NULL;              // Cycle / flush multi-framebuffer for camera
volatile uint16_t mtx[160][120] = {0};// Input image matrix, initialize to zeros at compile time
uint32_t exposure;
volatile uint32_t framenumber = 0;

float p = 3.1415926;
volatile uint32_t capturefail = 0;
volatile uint64_t pixsum = 0;
uint64_t ledout = 0;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);

  camera_config_t config;
    //config.ledc_channel = LEDC_CHANNEL_0;
    //config.ledc_timer = LEDC_TIMER_0;
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
    config.xclk_freq_hz = 10000000;             // 10mhz less fussy; can go to 20mhz maybe less reliable; varies board-to-board
    config.pixel_format = PIXFORMAT_RGB565;     // YUV422,GRAYSCALE,RGB565,JPEG    //jpeg might be fastest, then decompress
    //config.grab_mode = CAMERA_GRAB_WHEN_EMPTY // CAMERA_GRAB_LATEST. Sets when buffers should be filled
    //config.fb_location = CAMERA_FB_IN_DRAM;
  
  if(psramFound())
  {
    config.frame_size = FRAMESIZE_QQVGA;  // FRAMESIZE_ + QQVGA ,QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA   Do not use sizes above QVGA when not JPEG
    config.jpeg_quality = 10;             // 0-63 lower number means higher quality
    config.fb_count = 2;
  } 
  else 
  {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) 
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  Serial.print("\n");
  Serial.print("Welcome to ECE516 Lab 2.\n");
  
  fb = esp_camera_fb_get();  //PUT IMAGE IN MAIN CAM FB SO IT INHERITS THE CAMERA SETTINGS
  if(!fb) 
  {
    Serial.println("Camera capture failed");
    return;
  }
  esp_camera_fb_return(fb); //release framebuffer memory

  sensor_t * s = esp_camera_sensor_get();
  
  s->set_brightness(s, 0);                // -2 to 2  **2
  s->set_contrast(s, 0);                  // -2 to 2
  s->set_saturation(s, 0);                // -2 to 2
  s->set_special_effect(s, 2);            // 0 to 6 (0 No Effect, 1 Negative, 2 Grayscale, 3 Red Tint, 4 Green Tint, 5 Blue Tint, 6 Sepia)
  s->set_whitebal(s, 1);                  // 0 = disable , 1 = enable
  s->set_awb_gain(s, 0);                  // 0 = disable , 1 = enable
  s->set_wb_mode(s, 1);                   // 0 to 4 if awb_gain enabled (0 Auto, 1 Sunny, 2 Cloudy, 3 Office, 4 Home)
  s->set_exposure_ctrl(s, 0);             // 0 = disable , 1 = enable
  s->set_aec2(s, 1);                      // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);                  // -2 to 2
  s->set_aec_value(s, 300);               // 0 to 1200 **800   150
  s->set_gain_ctrl(s, 0);                 // 0 = disable , 1 = enable
  s->set_agc_gain(s, 3);                  // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);// 0 to 6
  s->set_bpc(s, 0);                       // 0 = disable , 1 = enable
  s->set_wpc(s, 0);                       // 0 = disable , 1 = enable
  s->set_raw_gma(s, 0);                   // 0 = disable , 1 = enable
  s->set_lenc(s, 1);                      // 0 = disable , 1 = enable   lense correction
  s->set_hmirror(s, 0);                   // 0 = disable , 1 = enable
  s->set_vflip(s, 0);                     // 0 = disable , 1 = enable
  s->set_dcw(s, 1);                       // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);                  // 0 = disable , 1 = enable

}

void sum_uint8_with_dotprod(uint8_t* u8_data, int len) {
    // 1. Prepare buffers (ESP-DSP works best with float or int16)
    float* float_data = (float*)malloc(len * sizeof(float));
    float* unit_vector = (float*)malloc(len * sizeof(float));
    float result = 0;

    // 2. Initialize: Convert uint8 to float and create unit vector
    for (int i = 0; i < len; i++) {
        float_data[i] = (float)u8_data[i];
        unit_vector[i] = 1.0f;
    }

    // 3. Execute Vectorized Dot Product
    // dsps_dotprod_f32 uses assembly optimizations for Xtensa LX6/LX7
    dsps_dotprod_f32(float_data, unit_vector, &result, len);

    pixsum = result;
    printf("Total Sum: %f\n", result);

    free(float_data);
    free(unit_vector);
}


void loop() 
{
  uint16_t bd, mred, mgreen, mblue, ired , igreen, iblue;

//  Serial.print("#Please enter an integer, not more than 500, to control the exposure:\n");
//  
//  while(!Serial.available()){}
//  exposure=Serial.parseInt();
//  
//  Serial.println("P3");
//  Serial.print("#Exposure=");
//  Serial.println(exposure);
//  Serial.println("160");
//  Serial.println("120");
//  Serial.println("255");
  //exposure = 500;
  
  sensor_t * s = esp_camera_sensor_get();
  //s->set_aec_value(s, exposure);  // 0 to 1200 **800   //VIA SOMETHING RELATED TO EXPOSURE
  //s->set_agc_gain();            // 0 to 30           //VIA ELEMENT ANALOG GAIN

  fb = esp_camera_fb_get();  //attempt to capture latest image from camera to PSRAM
  if(!fb) 
  {
    Serial.println("CAP FAIL");
    Serial.println("COUNTING IT");

    capturefail++;
    if(capturefail > 10)
    {
      Serial.println("Camera capture failed too many times");
      delay(200);
      Serial.println("ATTEMPT REBOOT");
      delay(20);
      Serial.print(".");
      delay(20);
      Serial.print(".");
      delay(20);
      Serial.print(".");
      ESP.restart();
      return;
    }
  }
  
  sum_uint8_with_dotprod(fb->buf, (120 * 160));
//  for (int i = 0; i < 120 * 160; i++)
//  {
//      pixsum = pixsum + fb->buf[i];      //make simplified-access copy of the framebuffer in LOCAL ram
//  }
  esp_camera_fb_return(fb); //release PSRAM framebuffer immediately

  ledout = map(pixsum, 1000000, 4896000, 0, 255);
  analogWrite(led_pin, ledout);
  Serial.println(pixsum);
  pixsum = 0;

//  for (int iy=0; iy<(120); iy=iy+1)    //process and write to outbound buffer
//  {
//    for (int ix=0; ix<(160); ix=ix+1) 
//    {
//      ired = ((mtx[ix][iy+0]>>0) & 0b00011111) <<3 ;
//      igreen  = ((mtx[ix][iy+0]>>5) & 0b00111111) << 2;   
//      iblue = ((mtx[ix][iy+0]>>11) & 0b00011111) << 3;
//
//      mred = ired;
//      mblue = iblue;
//      mgreen = igreen;
//
//      if(!(framenumber%60))
//      {
//        Serial.print(mred);
//        Serial.print(" ");
//        Serial.print(mgreen);
//        Serial.print(" ");
//        Serial.print(mblue);
//        Serial.print(" ");
//      }
//
//      bd = ( (((mred  >>3) <<11) & 0b1111100000000000 ) + 
//             (((mgreen>>2) << 5) & 0b0000011111100000 ) + 
//             (((mblue >>3) << 0) & 0b0000000000011111 ) );   
//
//    }
//    if (!(framenumber%60)) Serial.print("\n");
//  }
//  if (!(framenumber%60)) Serial.print("\n\n");
//  framenumber++; 
}