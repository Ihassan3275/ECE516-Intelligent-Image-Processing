/***ECE516-LAB2***/
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "dsps_dotprod.h"

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

#define LED_PIN 12
#define FRAME_W 96
#define FRAME_H 96

camera_fb_t * fb = NULL;
float pixSum = 0;
uint8_t ledOut = 0;
uint64_t currtime = 0;
uint64_t pastime = 0;
uint64_t dt = 0;

esp_err_t camera_init(camera_config_t* config)
{
  esp_err_t err;
  
  // Pin and clock config
  config->ledc_channel = LEDC_CHANNEL_0;
  config->ledc_timer = LEDC_TIMER_0;
  config->pin_d0 = Y2_GPIO_NUM;
  config->pin_d1 = Y3_GPIO_NUM;
  config->pin_d2 = Y4_GPIO_NUM;
  config->pin_d3 = Y5_GPIO_NUM;
  config->pin_d4 = Y6_GPIO_NUM;
  config->pin_d5 = Y7_GPIO_NUM;
  config->pin_d6 = Y8_GPIO_NUM;
  config->pin_d7 = Y9_GPIO_NUM;
  config->pin_xclk = XCLK_GPIO_NUM;
  config->pin_pclk = PCLK_GPIO_NUM;
  config->pin_vsync = VSYNC_GPIO_NUM;
  config->pin_href = HREF_GPIO_NUM;
  config->pin_sscb_sda = SIOD_GPIO_NUM;
  config->pin_sscb_scl = SIOC_GPIO_NUM;
  config->pin_pwdn = PWDN_GPIO_NUM;
  config->pin_reset = RESET_GPIO_NUM;
  
  // High FPS config
  config->xclk_freq_hz = 20000000;         // 20MHz is often more stable for the divider override
  config->pixel_format = PIXFORMAT_GRAYSCALE; 
  config->frame_size = FRAMESIZE_96X96;
  config->fb_count = 2;                    // MUST be 2 for high speed
  config->grab_mode = CAMERA_GRAB_LATEST;
  config->fb_location = CAMERA_FB_IN_DRAM; // CRITICAL: Internal RAM is faster than PSRAM

  err = esp_camera_init(config);
  return err;
}

void sensor_config(sensor_t* s)
{
  esp_err_t err;

  // The OV2640 camera has two banks of registers for its settings 
  // bank 0 and bank 1, we switch to bank 1 registers first
  s->set_reg(s, 0xff, 0xff, 0x01);
  
  // Now that we are in bank 1 we modify the internal clock prescaler 
  // of ov2640 which indicates the factor that XCLK clock input gets divided by
  // in our case we are setting the prescaler to 1 to get the max pixel output
  // which results in occasional frame drops which doesn't matter for our use case
  s->set_reg(s, 0x11, 0xff, 0x01);

  // Manual exposure settings
  s->set_exposure_ctrl(s, 0); 
  s->set_aec_value(s, 150);   // 0 to 1200 **800   150
  s->set_aec2(s, 0);          
  // To compensate for low exposure time we boost the gain 
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 15);     // 0 to 30

  s->set_bpc(s, 0);
  s->set_wpc(s, 0);
  s->set_lenc(s, 0);
}

void optimized_sum(uint8_t *camBuf, size_t len)
{
  float* toFloat = (float*)malloc(len * sizeof(float));
  float* mockArr = (float*)malloc(len * sizeof(float));

  for(size_t i = 0; i < len; i++)
  {
    toFloat[i] = (float)camBuf[i];
    mockArr[i] = 1.0f;
  }

  dsps_dotprod_f32(toFloat, mockArr, &pixSum, len);

  free(toFloat);
  free(mockArr);
}

void setup() 
{
  esp_err_t err;
  camera_config_t config;  
  sensor_t* s;
    
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout
  Serial.begin(2000000);
  pinMode(LED_PIN, OUTPUT);

  err = camera_init(&config);
  if(err != ESP_OK)
  {
    Serial.println("***Camera Init Failed");
  }
  else
  {
    Serial.println("***Camera Initialized");
  }
  
  s = esp_camera_sensor_get();
  sensor_config(s);

  Serial.println("***High FPS Camera Settings");
}

void loop() 
{
  currtime = micros();
  dt = currtime - pastime;
  pastime = currtime;
  
  fb = esp_camera_fb_get();

  if(fb)
  {
    optimized_sum(fb->buf, (FRAME_W * FRAME_H));
    ledOut = map(pixSum, -2000, 2350080, 0, 255);
    analogWrite(LED_PIN, ledOut);
    Serial.printf("SUM %f FPS %f\n", pixSum, (1000000.0 / dt));
  }
  else
  {
    Serial.println("***Cam Capture Failed");  
  }

  esp_camera_fb_return(fb);
    
}