#include <Arduino.h>
#include <esp_dmx.h>
#include <FastLED.h>
#include <Arduino.h>
#include <Arduino_GFX_Library.h>

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    5 /* DE */, 3 /* VSYNC */, 46 /* HSYNC */, 7 /* PCLK */,
    1 /* R0 */, 2 /* R1 */, 42 /* R2 */, 41 /* R3 */, 40 /* R4 */,                // if not R0, start at lowest R
    39 /* G0 */, 0 /* G1 */, 45 /* G2 */, 48 /* G3 */, 47 /* G4 */, 21 /* G5 */,  // if not G0, start at lowest G
    14 /* B0 */, 38 /* B1 */, 18 /* B2 */, 17 /* B3 */, 10 /* B4 */,              // if not B0, start at lowest B
    0 /* hsync_polarity */, 210 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    0 /* vsync_polarity */, 22 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, rgbpanel);

CRGB leds[1];

int transmitPin1 = 16;
int receivePin1 = 15;
int enablePin1 = -1;

int transmitPin2 = 6;
int receivePin2 = -1;
int enablePin2 = -1;

dmx_port_t dmxPort1 = 1;
dmx_port_t dmxPort2 = 2;

byte data[DMX_PACKET_SIZE];

bool dmxIsConnected = false;
unsigned long lastUpdate = millis();

uint16_t w, h;

void setup() {

  Serial.begin(115200); /* prepare for possible serial debug */
  delay(100);
  
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
