/**
 * The example demonstrates how to port LVGL.
 *
 * ## How to Use
 *
 * To use this example, please firstly install `ESP32_Display_Panel` (including its dependent libraries) and
 * `lvgl` (v8.3.x) libraries, then follow the steps to configure them:
 *
 * 1. [Configure ESP32_Display_Panel](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-esp32_display_panel)
 * 2. [Configure LVGL](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-lvgl)
 * 3. [Configure Board](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-board)
 *
 * ## Example Output
 *
 * ```bash
 * ...
 * Hello LVGL! V8.3.8
 * I am ESP32_Display_Panel
 * Starting LVGL task
 * Setup done
 * Loop
 * Loop
 * Loop
 * Loop
 * ...
 * ```
 */

#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include "lv_conf.h"
// #include <demos/lv_demos.h>
#include <Arduino.h>
#include <esp_dmx.h>
#include <FastLED.h>

CRGB leds[1];

/* First, lets define the hardware pins that we are using with our ESP32. We
  need to define which pin is transmitting data and which pin is receiving data.
  DMX circuits also often need to be told when we are transmitting and when we
  are receiving data. We can do this by defining an enable pin. */
int transmitPin1 = 16;
int receivePin1 = 15;
int enablePin1 = -1;

int transmitPin2 = 6;
int receivePin2 = 19;
int enablePin2 = -1;

/* Make sure to double-check that these pins are compatible with your ESP32!
  Some ESP32s, such as the ESP32-WROVER series, do not allow you to read or
  write data on pins 16 or 17, so it's always good to read the manuals. */

/* Next, lets decide which DMX port to use. The ESP32 has either 2 or 3 ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 1! */
dmx_port_t dmxPort1 = 1;
dmx_port_t dmxPort2 = 2;

/* Now we want somewhere to store our DMX data. Since a single packet of DMX
  data can be up to 513 bytes long, we want our array to be at least that long.
  This library knows that the max DMX packet size is 513, so we can fill in the
  array size with `DMX_PACKET_SIZE`. */
byte data[DMX_PACKET_SIZE];

/* The last two variables will allow us to know if DMX has been connected and
  also to update our packet and print to the Serial Monitor at regular
  intervals. */
bool dmxIsConnected = false;
unsigned long lastUpdate = millis();

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
// #include <demos/lv_demos.h>
// #include <examples/lv_examples.h>

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;                  // LVGL mutex

/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}

/* Read the touchpad */
void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data) {
    panel->getLcdTouch()->readData();

    bool touched = panel->getLcdTouch()->getTouchState();
    if(!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();

        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = point.x;
        data->point.y = point.y;

        Serial.printf("Touch point: x %d, y %d\n", point.x, point.y);
    }
}

void lvgl_port_lock(int timeout_ms) {
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
}

void lvgl_port_unlock(void) {
    xSemaphoreGiveRecursive(lvgl_mux);
}

void lvgl_port_task(void *arg) {
    Serial.println("Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        lvgl_port_lock(-1);
        task_delay_ms = lv_timer_handler();
        // Release the mutex
        lvgl_port_unlock();
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void setup() {

  Serial.begin(115200); /* prepare for possible serial debug */
  delay(100);
  
  String LVGL_Arduino = "Hello LVGL! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  Serial.println(LVGL_Arduino);
  Serial.println("I am ESP32_Display_Panel");

  panel = new ESP_Panel();

  /* Initialize LVGL core */
  lv_init();

  /* Initialize LVGL buffers */
  static lv_disp_draw_buf_t draw_buf;
  /* Using double buffers is more faster than single buffer */
  /* Using internal SRAM is more fast than PSRAM (Note: Memory allocated using `malloc` may be located in PSRAM.) */
  uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
  assert(buf);
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

  /* Initialize the display device */
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = ESP_PANEL_LCD_H_RES;
  disp_drv.ver_res = ESP_PANEL_LCD_V_RES;
  disp_drv.flush_cb = lvgl_port_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the input device */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lvgl_port_tp_read;
  lv_indev_drv_register(&indev_drv);

  /* Initialize bus and device of panel */
  panel->init();

  /**
   * These development boards require the use of an IO expander to configure the screen,
   * so it needs to be initialized in advance and registered with the panel for use.
   *
   */
  Serial.println("Initialize IO expander");
  /* Initialize IO expander */
  // ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
  ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
  expander->init();
  expander->begin();
  expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
  expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);

  // Turn off backlight
  // expander->digitalWrite(USB_SEL, LOW);
  expander->digitalWrite(USB_SEL, LOW);
  /* Add into panel */
  panel->addIOExpander(expander);

  /* Start panel */
  panel->begin();

  // /* Create a task to run the LVGL task periodically */
  // lvgl_mux = xSemaphoreCreateRecursiveMutex();
  // xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

  // /* Lock the mutex due to the LVGL APIs are not thread-safe */
  // lvgl_port_lock(-1);

  // /* Create simple label */
  // lv_obj_t *label = lv_label_create(lv_scr_act());
  // lv_label_set_text(label, LVGL_Arduino.c_str());
  // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  // /**
  //  * Try an example. Don't forget to uncomment header.
  //  * See all the examples online: https://docs.lvgl.io/master/examples.html
  //  * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples
  //  */
  // //  lv_example_btn_1();

  // /**
  //  * Or try out a demo.
  //  * Don't forget to uncomment header and enable the demos in `lv_conf.h`. E.g. `LV_USE_DEMOS_WIDGETS`
  //  */
  // // lv_demo_widgets();
  // // lv_demo_benchmark();
  // // lv_demo_music();
  // // lv_demo_stress();

  // /* Release the mutex */
  // lvgl_port_unlock();

  Serial.printf("*** SOC has %d UARTs\n",SOC_UART_NUM);
  
  dmx_config_t config = DMX_CONFIG_DEFAULT;

  dmx_driver_install(dmxPort1, &config, DMX_INTR_FLAGS_DEFAULT);
  dmx_driver_install(dmxPort2, &config, DMX_INTR_FLAGS_DEFAULT);

  dmx_set_pin(dmxPort1, transmitPin1, receivePin1, enablePin1);
  dmx_set_pin(dmxPort2, transmitPin2, receivePin2, enablePin2);

  Serial.println("Setup done");

}

void loop() {

  dmx_packet_t packet;

  if (dmx_receive(dmxPort1, &packet, DMX_TIMEOUT_TICK)) {

    unsigned long now = millis();

    if (!packet.err) {
      /* If this is the first DMX data we've received, lets log it! */
      if (!dmxIsConnected) {
        Serial.println("DMX is connected!");
        dmxIsConnected = true;
      }

      dmx_read(dmxPort1, data, packet.size);
      
      //  1 = pan
      //  2 = tilt
      //  3 = speed
      //  4 = main beam bright
      //  5 = strobe slow to fast, 0 = constant
      //  6 = color wheel main beam
      //  7 = gobo main beam
      //  8 = ring rotation, bottom half manual, top half auto
      //  9 = ring bright
      // 10 = ring strobe, slow to fast
      // 11 = ring red
      // 12 = ring green
      // 13 = ring blue
      // 14 = ring white
      // 15 = 0 manual, low half is auto, high half is sound, 255 = reset

      // data[1] = beatsin8(20);
      
      if (data[1] != 0 && data[1] != 255) data[1] = beatsin8(map8(data[8],1,100),0,data[1]); // map8(data[1],0,255); // stop over-rotation

      // data[8] = beatsin8(20,0,63);
      fill_rainbow(&(leds[0]),1,beat8(map8(data[7],1,30)),0);

      data[2] = leds[0][0];
      data[3] = leds[0][1];
      data[4] = leds[0][2];

      // data[11] = beatsin8(60,0,255,0,0);
      // data[12] = beatsin8(60,0,255,0,85);
      // data[13] = beatsin8(60,0,255,0,170);

      // data[11] = beatsin8(60,0,255,0,0);
      // data[12] = beatsin8(60,0,255,0,85);
      // data[13] = beatsin8(60,0,255,0,170);

      for (uint16_t i = 1; i<=16; i++) {
        data[i+16] = data[i];
      }

      data[1+16] = map(data[1+16],67,255,255,67);
      // data[8+16] = map(data[8],0,43,63,0);

      dmx_write(dmxPort2, data, packet.size);
      dmx_send(dmxPort2, packet.size);
      dmx_wait_sent(dmxPort2, DMX_TIMEOUT_TICK);

      if (now - lastUpdate > 200) {
        // lv_label_set_text(label, data[1]);
        for (int i=1; i<=32; i++) {
          Serial.printf("%03u ",data[i]);
        }
        Serial.println();
        lastUpdate = now;
      }
    } else {
      Serial.println("A DMX error occurred.");
    }
  } else if (dmxIsConnected) {
    // TODO: Reconnect without resetting board.
    Serial.println("DMX was disconnected.");
    dmx_driver_delete(dmxPort1);
    /* Stop the program. */
    while (true) yield();
  }
}


// /*

//   DMX Read

//   This sketch allows you to read DMX from a DMX controller using a standard DMX
//   shield, such SparkFun ESP32 Thing Plus DMX to LED Shield. This sketch was
//   made for the Arduino framework!

//   Created 9 September 2021
//   By Mitch Weisbrod

//   https://github.com/someweisguy/esp_dmx

// */


// void setup() {
//   /* Start the serial connection back to the computer so that we can log
//     messages to the Serial Monitor. Lets set the baud rate to 115200. */
//   Serial.begin(115200);
//   delay(100);
//   // Serial.printf("*** SOC has %d UARTs\n",SOC_UART_NUM);
//   // delay(100);

//   /* Now we will install the DMX driver! We'll tell it which DMX port to use,
//     what device configuration to use, and what DMX personalities it should have.
//     If you aren't sure which configuration to use, you can use the macros
//     `DMX_CONFIG_DEFAULT` to set the configuration to its default settings.
//     This device is being setup as a DMX responder so it is likely that it should
//     respond to DMX commands. It will need at least one DMX personality. Since
//     this is an example, we will use a default personality which only uses 1 DMX
//     slot in its footprint. */
//   // dmx_config_t config = DMX_CONFIG_DEFAULT;
//   // dmx_personality_t personalities[] = {
//   //   {1, "Default Personality"}
//   // };
//   // int personality_count = 1;
//   // dmx_driver_install(dmxPort1, &config, DMX_INTR_FLAGS_DEFAULT);
//   // dmx_driver_install(dmxPort2, &config, DMX_INTR_FLAGS_DEFAULT);

//   dmx_config_t config = DMX_CONFIG_DEFAULT;

//   dmx_driver_install(dmxPort1, &config, DMX_INTR_FLAGS_DEFAULT);
//   dmx_driver_install(dmxPort2, &config, DMX_INTR_FLAGS_DEFAULT);

//   /* Now set the DMX hardware pins to the pins that we want to use and setup
//     will be complete! */
//   dmx_set_pin(dmxPort1, transmitPin1, receivePin1, enablePin1);
//   dmx_set_pin(dmxPort2, transmitPin2, receivePin2, enablePin2);

// }

// void loop() {
//   /* We need a place to store information about the DMX packets we receive. We
//     will use a dmx_packet_t to store that packet information.  */
//   dmx_packet_t packet;

//   /* And now we wait! The DMX standard defines the amount of time until DMX
//     officially times out. That amount of time is converted into ESP32 clock
//     ticks using the constant `DMX_TIMEOUT_TICK`. If it takes longer than that
//     amount of time to receive data, this if statement will evaluate to false. */
//   if (dmx_receive(dmxPort1, &packet, DMX_TIMEOUT_TICK)) {
//     /* If this code gets called, it means we've received DMX data! */

//     /* Get the current time since boot in milliseconds so that we can find out
//       how long it has been since we last updated data and printed to the Serial
//       Monitor. */
//     unsigned long now = millis();

//     /* We should check to make sure that there weren't any DMX errors. */
//     if (!packet.err) {
//       /* If this is the first DMX data we've received, lets log it! */
//       if (!dmxIsConnected) {
//         Serial.println("DMX is connected!");
//         dmxIsConnected = true;
//       }

//       /* Don't forget we need to actually read the DMX data into our buffer so
//         that we can print it out. */
//       dmx_read(dmxPort1, data, packet.size);
      
//       //  1 = pan
//       //  2 = tilt
//       //  3 = speed
//       //  4 = main beam bright
//       //  5 = strobe slow to fast, 0 = constant
//       //  6 = color wheel main beam
//       //  7 = gobo main beam
//       //  8 = ring rotation, bottom half manual, top half auto
//       //  9 = ring bright
//       // 10 = ring strobe, slow to fast
//       // 11 = ring red
//       // 12 = ring green
//       // 13 = ring blue
//       // 14 = ring white
//       // 15 = 0 manual, low half is auto, high half is sound, 255 = reset

//       // data[1] = beatsin8(20);
      
//       if (data[1] != 0 && data[1] != 255) data[1] = beatsin8(map8(data[8],1,100),0,data[1]); // map8(data[1],0,255); // stop over-rotation

//       // data[8] = beatsin8(20,0,63);
//       fill_rainbow(&(leds[0]),1,beat8(map8(data[7],1,30)),0);

//       data[2] = leds[0][0];
//       data[3] = leds[0][1];
//       data[4] = leds[0][2];

//       // data[11] = beatsin8(60,0,255,0,0);
//       // data[12] = beatsin8(60,0,255,0,85);
//       // data[13] = beatsin8(60,0,255,0,170);

//       // data[11] = beatsin8(60,0,255,0,0);
//       // data[12] = beatsin8(60,0,255,0,85);
//       // data[13] = beatsin8(60,0,255,0,170);

//       for (uint16_t i = 1; i<=16; i++) {
//         data[i+16] = data[i];
//       }

//       data[1+16] = map(data[1+16],67,255,255,67);
//       // data[8+16] = map(data[8],0,43,63,0);

//       dmx_write(dmxPort2, data, packet.size);
//       dmx_send(dmxPort2, packet.size);
//       dmx_wait_sent(dmxPort2, DMX_TIMEOUT_TICK);

//       if (now - lastUpdate > 200) {
//         for (int i=1; i<=32; i++) {
//           Serial.printf("%03u ",data[i]);
//         }
//         /* Print the received start code - it's usually 0. */
//         // Serial.printf("Start code is %03u and slot 1 is %03u\n", data[0], data[1]);
//         Serial.println();
//         lastUpdate = now;
//       }
//     } else {
//       /* Oops! A DMX error occurred! Don't worry, this can happen when you first
//         connect or disconnect your DMX devices. If you are consistently getting
//         DMX errors, then something may have gone wrong with your code or
//         something is seriously wrong with your DMX transmitter. */
//       Serial.println("A DMX error occurred.");
//     }
//   } else if (dmxIsConnected) {
//     /* If DMX times out after having been connected, it likely means that the
//       DMX cable was unplugged. When that happens in this example sketch, we'll
//       uninstall the DMX driver. */
//     Serial.println("DMX was disconnected.");
//     dmx_driver_delete(dmxPort1);

//     /* Stop the program. */
//     while (true) yield();
//   }
// }


// // #include <HardwareSerial.h>

// // #define dmxMaxChannel  513
// // #define defaultMax 32

// // #define DMXSPEED       250000
// // #define DMXFORMAT      SERIAL_8N2

// // int enablePin = -1;		//dafault on ESP32
// // int rxPin = 15;
// // int txPin = 16;

// // HardwareSerial DMXSerial(2);

// // void setup() {

// //     Serial.begin(115200);
// //     // Serial2.begin(250000, SERIAL_8N2, RXD2, TXD2);

// //     DMXSerial.begin(DMXSPEED, DMXFORMAT, rxPin, txPin);
    
// //     delay(1000);

// //     Serial.println("ESP32 Serial Txd is on pin: "+String(TX));
// //     Serial.println("ESP32 Serial Rxd is on pin: "+String(RX));
// //     Serial.println("DMXSerial Txd is on pin: "+String(txPin));
// //     Serial.println("DMXSerial Rxd is on pin: "+String(rxPin));

// //     delay(1000);

// // }

// // void loop() {
    
// //     int count = 0;

// //     while (DMXSerial.available()) {

// //         char dmxvalue = DMXSerial.read();

// //         if (count == 16) {

// //             Serial.println();
// //             count = 0;
            
// //         }

// //         Serial.printf("%03u ",dmxvalue);
// //         count++;

// //     }

// // }
