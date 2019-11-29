/* The main program of 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include "driver/i2s.h"
#include <FastLED.h>
#include "includes/FFT.h"
#include "includes/VisualEffect.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino

/* Some variables in programs
*
* BUFFER_SIZE: How many samples to be received each time. This number must be the power of 2.
* const uint16_t BUFFER_SIZE = 1024;
*
* N_ROLLING_HISTORY: How many buffers to be processed each time. This number must be the power of 2: 1,2,4,8,...
* const uint8_t N_ROLLING_HISTORY = 2;
*
* SAMPLE_RATE: The sample rate of the audio every second.
* const uint16_t SAMPLE_RATE = 44100;
*
* N_PIXELS: The number of the LEDS on the led strip, it must be even.
* const uint16_t N_PIXELS = 60;
*
* N_MEL_BIN: The number of the channels of the mel frequency.
* const uint16_t N_MEL_BIN = 18;
*
* MIN_FREQUENCY, MAX_FREQUENCY: The audio's min/max frequency to be processed. The max frequency always less than SAMPLE_RATE/2
* const float MIN_FREQUENCY = 200;
* const float MAX_FREQUENCY = 12000;
*
* MIN_VOLUME_THRESHOLD: If the audio's volume is less than this number, the signal will not be processed.
* const float MIN_VOLUME_THRESHOLD = 0.0003;
*
* PDM_WS_IO_PIN, PDM_DATA_IN_PIN: Microphone(type of PDM)'s WS Pin and DATA_IN Pin, connecting to GPIO
* const int PDM_WS_IO_PIN = 19;
* const int PDM_DATA_IN_PIN = 22;
*
* LED_STRIP_DATA_PIN, LED_STRIP_CLOCK_PIN: Led-strip's data pin and clock pin, connecting to GPIO
* If you use a led-strip with clock pin, you should modify the FastLED.addLeds calling in programs.
* const int LED_STRIP_DATA_PIN = 21;
* const int LED_STRIP_CLOCK_PIN = 17;
*
* TOUCH_PAD_PIN: TOUCH PAD's number. TOUCH_PAD_NUM9 is GPIO32. https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/include/driver/driver/touch_pad.h
* const touch_pad_t TOUCH_PAD_PIN = TOUCH_PAD_NUM9;
* 
*  NOTE: ADC2 cannot be used when wifi is enabled. ADC2 pins are GPIO 0, 2, 4, 12-15, 25-27 = A0,A1,A5,A6,A8,A10,A11 on the feather
*  GPIO #12 / A11 has a pull-down built in, so use only as output
*  ADC1: GPIO 13, 32 - 39 = A2 (34), A3 (39), A4 (36), A7 (32), A9 (33), A12 (13)
*  GPIO 13 / A12 is connected to the led; A13 (GPIO 35) is connected to the battery and thus not exposed on the feather.
*  GPIO 34, 36, 39 are NOT output capable
*
*/

const i2s_port_t I2S_PORT = I2S_NUM_0;

const uint16_t BUFFER_SIZE = 1024;
const uint8_t N_ROLLING_HISTORY = 2;
const uint16_t SAMPLE_RATE = 36000;
const uint16_t N_PIXELS = 144;
const uint16_t N_MEL_BIN = 18;
const float MIN_FREQUENCY = 200;
const float MAX_FREQUENCY = 12000;
// The max value is 2^(SAMPLE_BITS-1) - 1
const float MAX_SAMPLE_VALUE = 2147483647.0;

const int BCKL_PIN = 14;
const int LRCL_PIN = 15;
const int DOUT_PIN = 32;

const int LED_STRIP_DATA_PIN = 23;
const int LED_STRIP_CLOCK_PIN = 22;
const touch_pad_t TOUCH_PAD_PIN = TOUCH_PAD_NUM8; 
/* Touch pad channel 0 is GPIO4, 1 is GPIO0, 2 is GPIO2, 3 is GPIO15, 4 is GPIO13, 
 * Touch pad channel 5 is GPIO12, 6 is GPIO14, 7 is GPIO27, 8 is GPIO33,  9 is GPIO32
 */

float y_data[BUFFER_SIZE * N_ROLLING_HISTORY];
class FFT fft(BUFFER_SIZE*N_ROLLING_HISTORY, N_MEL_BIN, MIN_FREQUENCY, MAX_FREQUENCY, SAMPLE_RATE);
class VisualEffect effect(N_MEL_BIN, N_PIXELS);
CRGB physic_leds[N_PIXELS];

int g_mode = 2;
float g_min_vol = 0.0003;
int g_bright = 127;
float g_scale = 2147483648.0;

BluetoothSerial ESP_BT; //Object for Bluetooth

void setup() {
  FastLED.addLeds<NEOPIXEL, LED_STRIP_DATA_PIN>(physic_leds, N_PIXELS);
  //FastLED.addLeds<APA102, LED_STRIP_DATA_PIN, LED_STRIP_CLOCK_PIN, GRB>(physic_leds, N_PIXELS);
  //FastLED.addLeds<DOTSTAR, LED_STRIP_DATA_PIN, LED_STRIP_CLOCK_PIN, RGB>(physic_leds, N_PIXELS);
  Serial.begin(115200);
  Serial.println("Configuring I2S...");
  esp_err_t err;

  // The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // although the SEL config should be left, it seems to transmit on right
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 32,                           // number of buffers
      .dma_buf_len = 32                              // samples per buffer (minimum = 8)
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = BCKL_PIN,
      .ws_io_num = LRCL_PIN,
      .data_out_num = -1, // not used (only for speakers)
      .data_in_num = DOUT_PIN
  };

  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S driver installed.");
  
  ESP_BT.begin("ESP32_LightOrgan"); //Name of your Bluetooth Signal
  Serial.println("Bluetooth Device is Ready to Pair");
  
//  touch_pad_init();
//  touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
//  touch_pad_config(TOUCH_PAD_PIN, 0);
}

void loop() {
  static float mel_data[N_MEL_BIN];

  for (int i = 0; i < N_ROLLING_HISTORY - 1; i++)
    memcpy(y_data + i * BUFFER_SIZE, y_data + (i + 1)*BUFFER_SIZE, sizeof(float)*BUFFER_SIZE);

  int32_t l[BUFFER_SIZE];

  unsigned int read_num;
  i2s_read(I2S_PORT, l, BUFFER_SIZE * 2, &read_num, portMAX_DELAY);

  for (int i = 0; i < BUFFER_SIZE; i++) {
    y_data[BUFFER_SIZE * (N_ROLLING_HISTORY - 1) + i] = constrain(l[i] / g_scale, -1, 1);

    /*
    * This should output the current time(in millisececonds) every second.
    * The output frequency larger than one second greatly, means the CPU is overload.
    */
//    static uint32_t ii = 0;
//    ii++;
//    if (ii % SAMPLE_RATE == 0) {
//      //Serial.printf("%d\n", millis());
//      for (int i = 0; i < BUFFER_SIZE; i++) {
//        Serial.printf("%d, ", l[i]);
//      }
//      Serial.printf("mode = %d\n", CurrentMode);
//    }
  }

  fft.t2mel(y_data, mel_data, g_min_vol);

  switch (g_mode) {
    case 0:
      fill_solid(physic_leds, N_PIXELS, CRGB::Black);
      break;
    case 1:
      fill_solid(physic_leds, N_PIXELS, CRGB(g_bright, g_bright, g_bright));
      break;
    case 2:
      effect.visualize_scroll(mel_data, physic_leds, g_bright);
      break;
    case 3:
      effect.visualize_energy(mel_data, physic_leds, g_bright);
      break;
    case 4:
      effect.visualize_spectrum(mel_data, physic_leds, g_bright);
      break;
    case 5:
      static uint8_t gHue = 0;
      fill_rainbow(physic_leds, N_PIXELS, gHue, 7);
      EVERY_N_MILLISECONDS( 20 ) {
        gHue++;
      }
      break;
  }
  FastLED.show();

//  static uint32_t oldtime = 0;
//  uint16_t touch_value;
//  touch_pad_read(TOUCH_PAD_NUM9, &touch_value);
//  if ((touch_value < 1000) && (millis() - oldtime > 1000)) {
//    oldtime = millis();
//    CurrentMode = PLAYMODE((CurrentMode + 1) % MODE_MAX);
//  }
//  else if (touch_value > 1000)
//    oldtime = 0;
  static int command;
  static int value;
  if (ESP_BT.available()) {
    command = ESP_BT.read();
    Serial.println(command);
    if (ESP_BT.available()) {
      value = ESP_BT.read();
      Serial.println(value);
      //Serial.printf("%s:%d\n", command, value);
      value -= '0';
      if(command=='m') {
        g_mode = value;
        ESP_BT.print("mode = ");  ESP_BT.println(g_mode);
      } else if (command=='v') {
        g_min_vol = value * 0.0001;
        ESP_BT.print("minimum volume = "); ESP_BT.println(g_min_vol);
      } else if (command=='b') {
        g_bright = (int)((value+1)*25.5);
        ESP_BT.print("maximum RGB value = "); ESP_BT.println(g_bright);
      } else if (command=='g') {
        // Set the gain
        g_scale = MAX_SAMPLE_VALUE / (value*15+1);
        ESP_BT.print("gain = "); ESP_BT.println(value*2+1);
      } else {
        Serial.print("ERROR: unknown command: "); Serial.println(command);
        ESP_BT.print("ERROR: unknown command: "); ESP_BT.println(command);
      }
    } else {
      Serial.println("ERROR: no value.");
      ESP_BT.print("ERROR: no value.");
      ESP_BT.flush();
    }
  }
  yield();
}
