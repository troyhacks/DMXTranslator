#include <Arduino.h>
#include <esp_dmx.h>
#include <FastLED.h>
#include <Arduino.h>
#include <Arduino_GFX_Library.h>

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    5 /* DE */, 3 /* VSYNC */, 46 /* HSYNC */, 7 /* PCLK */,
    1 /* R0 */, 2 /* R1 */, 42 /* R2 */, 41 /* R3 */, 40 /* R4 */,
    39 /* G0 */, 0 /* G1 */, 45 /* G2 */, 48 /* G3 */, 47 /* G4 */, 21 /* G5 */,
    14 /* B0 */, 38 /* B1 */, 18 /* B2 */, 17 /* B3 */, 10 /* B4 */,
    0 /* hsync_polarity */, 210 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    0 /* vsync_polarity */, 22 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel);

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

uint16_t w, h;

void setup() {

  Serial.begin(115200); /* prepare for possible serial debug */
  delay(100);

  Serial.printf("*** SOC has %d UARTs\n",SOC_UART_NUM);
  
  dmx_config_t config = DMX_CONFIG_DEFAULT;

  dmx_driver_install(dmxPort1, &config, DMX_INTR_FLAGS_DEFAULT);
  dmx_driver_install(dmxPort2, &config, DMX_INTR_FLAGS_DEFAULT);

  dmx_set_pin(dmxPort1, transmitPin1, receivePin1, enablePin1);
  dmx_set_pin(dmxPort2, transmitPin2, receivePin2, enablePin2);

  gfx->begin();

  w = gfx->width();
  h = gfx->height();

  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE,BLACK);
  gfx->setTextSize(7);

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
        gfx->setCursor(0, 10);
        gfx->println(" TroyHacks DMX Toy");
        gfx->setCursor(0, gfx->getCursorY()+15);

        // gfx->fillScreen(BLACK);
        for (int i=1; i<=35; i++) {
          if (i%5 == 0) {
            gfx->printf("%03u",data[i]);
          } else {
            gfx->printf("%03u ",data[i]);
          }
          Serial.printf("%03u ",data[i]);
        }
        Serial.println();
        // gfx->flush();
        lastUpdate = now;
      }
    } else {
      gfx->fillScreen(BLACK);
      gfx->setCursor(0, 10);
      gfx->println(" TroyHacks DMX Toy");
      gfx->setCursor(0, gfx->getCursorY()+15);
      gfx->println("DMX Error!");
      Serial.println("A DMX error occurred.");
    }
  } else if (dmxIsConnected) {
    
    Serial.println("DMX was disconnected.");
    
    gfx->fillScreen(BLACK);
    gfx->setCursor(0, 10);
    gfx->println(" TroyHacks DMX Toy");
    gfx->setCursor(0, gfx->getCursorY()+15);
    gfx->println("DMX Disconnected.");

    dmxIsConnected = false;

  }
}
