/***ECE516-LAB2***/
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "dsps_dotprod.h"
#include "math.h"
#include "driver/ledc.h"
#include "img_converters.h"

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

#define LED_PIN       12
#define LEDC_MODE     LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_12_BIT
#define LEDC_CHANNEL  LEDC_CHANNEL_5
#define LEDC_TIMER    LEDC_TIMER_1
#define LEDC_FREQ     19531
#define LED_RES       12

#define FRAME_W 96
#define FRAME_H 96
#define BUFFER_SIZE (FRAME_W * FRAME_H)

camera_config_t config;  
sensor_t* s;
ledc_timer_config_t ledc_timer;
ledc_channel_config_t ledc_channel;
camera_fb_t * fb = NULL;

static float toFloat[BUFFER_SIZE]__attribute__((aligned(16)));
static float mockArr[BUFFER_SIZE]__attribute__((aligned(16)));
uint8_t * rgb_buf = NULL;
bool mockInit = false;
float pixSum = 0;
uint16_t ledOut = 0;
float pixGamma = 1.8;

uint64_t currtime = 0;
uint64_t pastime = 0;
uint64_t dt = 0;

esp_err_t ledc_init(ledc_timer_config_t *ledc_timer, ledc_channel_config_t *ledc_channel)
{
  esp_err_t err;
  
  ledc_timer->timer_num = LEDC_TIMER;
  ledc_timer->speed_mode = LEDC_MODE;
  ledc_timer->freq_hz = LEDC_FREQ;
  ledc_timer->duty_resolution = LEDC_DUTY_RES;

  err = ledc_timer_config(ledc_timer);
  if(err != ESP_OK)
  {
    Serial.println("***Ledc Timer Config Failed");
    return err;
  }

  ledc_channel->channel = LEDC_CHANNEL;
  ledc_channel->duty = 0;
  ledc_channel->gpio_num = LED_PIN;
  ledc_channel->speed_mode = LEDC_MODE;
  ledc_channel->timer_sel = LEDC_TIMER;

  err = ledc_channel_config(ledc_channel);
  if(err != ESP_OK)
  {
    Serial.println("***Ledc Channel Config Failed");
    return err;
  }
  
  return err;
}

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
  config->xclk_freq_hz = 40000000;         // 20MHz is often more stable for the divider override
  config->pixel_format = PIXFORMAT_JPEG; 
  config->frame_size = FRAMESIZE_QQVGA;
  config->jpeg_quality = 60;   
  config->fb_count = 2;                    // MUST be 2 for high speed
  config->grab_mode = CAMERA_GRAB_LATEST;
  config->fb_location = CAMERA_FB_IN_DRAM; // CRITICAL: Internal RAM is faster than PSRAM

  err = esp_camera_init(config);
  return err;
}

void sensor_config(sensor_t* s)
{
  esp_err_t err;

  // This line modifies the REG32_CIF preset register which is responsible 
  // for setting the pixel clock divider, by writing 0x09 to the register
  // we basically get rid of the waiting period between each pixel
  s->set_reg(s, 0x111, 0x09, 0xFF);  // REG32_CIF = 0x09

  // Here we disable the CLKRC (Clock Rate Control) prescaler by writing 0x00
  // and enable clock doubler by writing 0x80 to register 0x11, this make the 
  // camera run on double the input frequency of XCLK input which in our case
  // is 40 MHz * 2 = 80 MHz = max clock speed of OV2640
  s->set_reg(s, 0x11, 0x00 | 0x80 , 0xFF);  // CLKRC, 0x00 | CLKRC_2X
  
  // The OV2640 camera has two banks of registers for its settings 
  // bank 0 and bank 1, we switch to bank 1 register
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

void jpeg_decomp(camera_fb_t *fb, float *gfb)
{
  size_t outLen = fb->width * fb->height * 2;
  
  bool success = jpg2rgb565(fb->buf, fb->len, rgb_buf, JPG_SCALE_NONE);
  if(success)
  {
    for(uint64_t i = 0; i < 9216; i ++)
    {
      uint16_t pixel = rgb_buf[i * 2 + 1] << 8 | rgb_buf[i * 2];

      uint8_t r = (pixel >> 11) & 0x1F;
      uint8_t g = (pixel >> 5) & 0x3F;
      uint8_t b = pixel & 0x1F;

      //gfb[i] = (r << 3) * 0.299 + (g << 2) * 0.587 + (b << 3) * 0.114;
      gfb[i] = (g << 2) * 0.587f;
    }
  }
}

esp_err_t optimized_sum()
{
  dsps_dotprod_f32(toFloat, mockArr, &pixSum, BUFFER_SIZE);

  return ESP_OK;
}

void mockArr_init(float *mockArr)
{
  for(uint64_t i = 0; i < BUFFER_SIZE; i++)
  {
    mockArr[i] = 1.0f;
  }

  rgb_buf = (uint8_t *)heap_caps_malloc((120 * 160 * 2), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  mockInit = true;
}

void setup() 
{
  esp_err_t err;
    
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout
  Serial.begin(2000000);
  
  err = ledc_init(&ledc_timer, &ledc_channel);
  if(err != ESP_OK)
  {
    Serial.println("***Ledc Init Failed");
  }

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

  mockArr_init(mockArr);
  
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
    jpeg_decomp(fb, toFloat);
    optimized_sum();

    pixSum = pixSum / (2350080 / 10);
    if(pixSum > 1.0f){pixSum = 1.0f;}
    if(pixSum < 0.0f){pixSum = 0.0f;}
    ledOut = (pow(pixSum, pixGamma) * 4095.0f) + 5;
    if(ledOut > 4095){ledOut = 4095.0f;}
    if(ledOut < 0){ledOut = 0;}
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (uint16_t)(ledOut));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    Serial.printf("SUM %f FPS %f\n", pixSum, (1000000.0 / dt));
  }
  else
  {
    Serial.println("***Cam Capture Failed");  
  }

  esp_camera_fb_return(fb);
    
}