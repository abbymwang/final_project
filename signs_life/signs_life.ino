// =============================================================================
// signs_life.ino
// Code for DESINV 23 Final Project
// Author: Abby Wang (integrated with Claude)
//
// Combines:
//   1. MQ-135 Flying Fish sensor  → CO2 delta tints LEDs white → blue
//   2. NewPing ultrasonic sensor  → proximity selects which panel shows
//   3. FastLED 8x8 WS2812B grid   → displays proximity pixel art panels
//
// Proximity bands:
//   out of range / >150cm → panel 0  (150cm)
//   126–150cm             → panel 1  (125cm)
//   101–125cm             → panel 2  (100cm)
//   76–100cm              → panel 3  (75cm)
//   51–75cm               → panel 4  (50cm)
//   26–50cm               → panel 5  (25cm)
//   0–25cm                → panel 6  (0cm / closest)
//
// CO2 tint:
//   delta 0             → white
//   delta CO2_DELTA_MAX → full blue
// =============================================================================

#include <FastLED.h>
#include <NewPing.h>

// ── Pin assignments ───────────────────────────────────────────────────────────
#define DATA_PIN      6    // FastLED data
#define TRIGGER_PIN   11   // NewPing trigger
#define ECHO_PIN      12   // NewPing echo
#define MQ135_PIN     A0   // MQ-135 analog out

// ── FastLED matrix settings ───────────────────────────────────────────────────
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB
#define W             8
#define H             8
#define NUM_LEDS      (W * H)
#define BRIGHTNESS    7    // keep low for power safety

CRGB leds[NUM_LEDS];

// Adjust to match your physical wiring
bool SERPENTINE      = true;
bool ORIGIN_TOP_LEFT = true;
int  ROTATE          = 0;

// ── NewPing settings ──────────────────────────────────────────────────────────
#define MAX_DISTANCE  200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ── MQ-135 / CO2 settings ─────────────────────────────────────────────────────
#define MQ135_WARMUP_SEC   10   // warm-up time in seconds
#define MQ135_BASELINE_N   10    // samples for baseline average
#define CO2_DELTA_MAX      1.00f  // delta that maps to 100% blue
float co2Baseline = 0;
float co2Delta    = 0;

// ── Proximity panel data (from pixel_grid_-_light__1_.csv) ───────────────────
// Index matches band: 0 = farthest, 6 = closest
#define NUM_BANDS 7

const uint32_t proximityPanels[NUM_BANDS][8][8] PROGMEM = {

  // Panel 0 — farthest
  {
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
  },
  
  // Panel 1 — nearer
  {
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
  },

  // Panel 2 — closer
  {
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF},
    {0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
  },

  // Panel 3 — closest?
  {
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0xFFFFFF, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF},
    {0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
  },

  // Unused panels
  {
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0xFFFFFF, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000},
    {0xFFFFFF, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF},
    {0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
  },

  // Unused panels
  {
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
    {0xFFFFFF, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0xFFFFFF},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF},
    {0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
  },

  // Unused panels
  {
    {0xFFFFFF, 0x000000, 0x000000, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000},
    {0x000000, 0xFFFFFF, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000, 0xFFFFFF},
    {0x000000, 0x000000, 0xFFFFFF, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
  },

};

// =============================================================================
// XY mapping — matches your physical serpentine wiring
// =============================================================================
int XY(uint8_t x, uint8_t y) {
  if (x >= W || y >= H) return -1;
  uint8_t yy = ORIGIN_TOP_LEFT ? y : (H - 1 - y);
  uint8_t xx = x;
  uint8_t rx, ry;
  switch (ROTATE) {
    case 0:   rx = xx;            ry = yy;            break;
    case 90:  rx = (W - 1 - yy); ry = xx;            break;
    case 180: rx = (W - 1 - xx); ry = (H - 1 - yy); break;
    case 270: rx = yy;            ry = (H - 1 - xx); break;
    default:  rx = xx;            ry = yy;            break;
  }
  if (!SERPENTINE) return ry * W + rx;
  if ((ry & 1) == 0) return ry * W + rx;
  return ry * W + (W - 1 - rx);
}

// =============================================================================
// Display a proximity panel with white → blue CO2 tint
// =============================================================================
void displayProximityPanel(int bandIndex, float delta) {
  float t = constrain(delta / CO2_DELTA_MAX, 0.0f, 1.0f);
  t = pow(t, 0.3f);

  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      uint32_t color = pgm_read_dword(&proximityPanels[bandIndex][row][col]);

      CRGB out;
      if (color == 0x000000) {
        out = CRGB::Black;                      // background stays off
      } else {
        // White (t=0) → Blue (t=1)
        out.r = (uint8_t)(255 * (1.0f - t));
        out.g = (uint8_t)(255 * (1.0f - t));
        out.b = 255;
      }

      int idx = XY(col, row);
      if (idx >= 0) leds[idx] = out;
    }
  }
  FastLED.show();
}

// =============================================================================
// Proximity → band index (0 = farthest, 6 = closest)
// =============================================================================
int distanceToBand(unsigned int cm) {
  if (cm == 0 || cm > 150) return 0;
  if (cm > 50)            return 1;
  if (cm > 25)            return 2;
  return 3;
}

// =============================================================================
// MQ-135 averaged read
// =============================================================================
float readMQ135avg(int n = 5) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogRead(MQ135_PIN);
    delay(10);
  }
  return (float)sum / n;
}

// =============================================================================
// setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Signs of Life: CO2 + Proximity Display ==="));

  // FastLED init
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // MQ-135 warm-up — cycles through panels as a visual progress indicator
  Serial.print(F("MQ-135 warming up"));
  for (int s = 0; s < MQ135_WARMUP_SEC; s++) {
    delay(1000);
    if (s % 10 == 9) {
      Serial.print('.');
      displayProximityPanel((s / 10) % NUM_BANDS, 0);
    }
  }
  Serial.println(F(" done!"));

  // Baseline calibration
  Serial.print(F("Calibrating CO2 baseline"));
  float sum = 0;
  for (int i = 0; i < MQ135_BASELINE_N; i++) {
    sum += readMQ135avg();
    delay(300);
    if (i % 5 == 4) Serial.print('.');
  }
  co2Baseline = sum / MQ135_BASELINE_N;
  Serial.println(F(" done!"));
  Serial.print(F("Baseline ADC: "));
  Serial.println(co2Baseline, 1);
  Serial.println(F("Ready — proximity controls panel, CO2 controls tint (white → blue)."));
  Serial.println(F("dist(cm) | band | CO2-delta | tint"));
  Serial.println(F("---------|------|-----------|-----"));
}

// =============================================================================
// loop
// =============================================================================
void loop() {
  // ── 1. Ultrasonic distance ────────────────────────────────────────────────────
  delay(100);
  unsigned int dist = sonar.ping_cm();

  // ── 2. MQ-135 CO2 delta (exponentially smoothed) ─────────────────────────────
  float rawReading = readMQ135avg(3);
  float rawDelta   = max(rawReading - co2Baseline, 0.0f);
  co2Delta = 0.7f * co2Delta + 0.3f * rawDelta;
  float tintDelta = constrain(co2Delta, 0.0f, CO2_DELTA_MAX);

  // Slowly drift baseline during clean air
  if (co2Delta < 1.0f) {
    co2Baseline = co2Baseline * 0.999f + rawReading * 0.001f;
  }

  // ── 3. Display ────────────────────────────────────────────────────────────────
  int band = distanceToBand(dist);
  displayProximityPanel(band, tintDelta);

  // ── 4. Serial debug ───────────────────────────────────────────────────────────
  Serial.print(dist);
  Serial.print(F(" cm\t| band "));
  Serial.print(band);
  Serial.print(F("\t| CO2 Δ "));
  Serial.print(tintDelta, 2);

  if      (tintDelta < 0.01f)          Serial.println(F("\t| white (clean air)"));
  else if (tintDelta < 0.2f)          Serial.println(F("\t| slight blue"));
  else if (tintDelta < 0.5f)          Serial.println(F("\t| medium blue"));
  else if (tintDelta < CO2_DELTA_MAX) Serial.println(F("\t| strong blue"));
  else                                 Serial.println(F("\t| MAX BLUE (breath!)"));
}
