/*
 * room_status -- window/room mission readout for a parallel TFT LCD shield on
 * an Arduino Uno (the MCUFRIEND-style 2.4"/2.8" shield: ILI9341, ILI9325,
 * HX8347, SPFD5408 and friends, auto-detected at runtime).
 *
 * FLASH THIS INSTEAD OF tft_status.ino / tft_doll_counter.ino for the
 * window-room mission. It is a merge of the two: the big colour-coded STATE
 * banner from tft_status, and the two big numbers from tft_doll_counter, plus
 * the mission's own progress strip.
 *
 * The screen shows, top to bottom:
 *
 *   STATE      the big banner: SEARCHING / WINDOW / ENTERING / IN ROOM /
 *              ROOM RUN / WINDOW 2 / EXITING / OUTSIDE / LANDING, colour
 *              coded 0 grey idle, 1 green ok, 2 amber busy, 3 red alarm
 *   TOTAL      cumulative dolls counted this flight. This is the mission
 *              number and it is the biggest thing on the screen.
 *   NOW        dolls visible in the current frame
 *   3 rows     free text from the Jetson: stage detail, altitude, flow
 *
 * The Arduino is deliberately dumb. It stores numbers and strings and draws
 * them; every decision about what the state is called and what colour it
 * should be is made in the ROS 2 node (room_display.py), so the layout and
 * the wording can change without reflashing the board.
 *
 * REQUIRED LIBRARIES (Arduino IDE -> Library Manager):
 *   - MCUFRIEND_kbv  by David Prentice
 *   - Adafruit GFX Library  by Adafruit
 *
 * The shield occupies D2-D9 and A0-A4. D0/D1 stay free, so the USB serial
 * link to the Jetson works alongside it -- that link is the only wiring this
 * needs.
 *
 * PROTOCOL (newline terminated, 115200 baud):
 *
 *   S:<0-3>      banner colour. Send BEFORE T: for the colour to apply.
 *   T:<text>     banner text, i.e. the mission state
 *   D:<v>,<t>    doll counts: v = visible right now, t = cumulative total
 *   1:<text>     body row 1  (stage detail)
 *   2:<text>     body row 2
 *   3:<text>     body row 3
 *   C            clear the whole screen and forget everything
 *
 * Both "C" and "C:" are accepted so the sender can use one key:value form
 * for everything.
 *
 * Only the parts of the screen whose content actually changed are redrawn; a
 * full repaint over an 8-bit parallel bus is slow enough to be visibly laggy,
 * and the counts update far more often than anything else.
 *
 * If nothing valid arrives for LINK_TIMEOUT_MS the screen goes red with
 * NO LINK, so an unplugged cable or a dead Jetson is obvious rather than
 * leaving a stale count on screen that reads like a result.
 */

#include <MCUFRIEND_kbv.h>
#include <Adafruit_GFX.h>

MCUFRIEND_kbv tft;

// ---- layout ---------------------------------------------------------------
// Portrait, 240x320, same as tft_status.ino -- if your shield is mounted the
// other way up change setRotation(0) to setRotation(2), not these numbers.
const int16_t SCREEN_W = 240;
const int16_t SCREEN_H = 320;

const int16_t BANNER_H = 46;
const uint8_t BANNER_TEXT_SIZE = 3;   // 18x24 px per char -> 13 cols

// The counts block. TOTAL is the mission result, so it gets size 6 (36x48)
// and NOW gets size 3 next to its own label.
const int16_t TOTAL_LABEL_Y = BANNER_H + 8;
const int16_t TOTAL_Y = BANNER_H + 26;
const int16_t TOTAL_H = 50;
const int16_t NOW_LABEL_Y = TOTAL_Y + TOTAL_H + 6;
const int16_t NOW_Y = NOW_LABEL_Y + 18;
const int16_t NOW_H = 26;

const int16_t BODY_TOP = NOW_Y + NOW_H + 10;
const int16_t BODY_ROW_H = 24;
const int16_t BODY_LEFT = 6;
const uint8_t BODY_TEXT_SIZE = 2;     // 12x16 px per char -> 20 cols

const uint8_t NUM_ROWS = 3;
const uint8_t MAX_TEXT = 24;

// ---- colours --------------------------------------------------------------
const uint16_t C_BLACK = 0x0000;
const uint16_t C_WHITE = 0xFFFF;
const uint16_t C_GREY = 0x8410;
const uint16_t C_GREEN = 0x07E0;
const uint16_t C_AMBER = 0xFD20;
const uint16_t C_RED = 0xF800;
const uint16_t C_CYAN = 0x07FF;
const uint16_t C_YELLOW = 0xFFE0;
const uint16_t C_BG = 0x0000;

const unsigned long LINK_TIMEOUT_MS = 3000;

// ---- state ----------------------------------------------------------------
char rowText[NUM_ROWS][MAX_TEXT + 1];
char bannerText[MAX_TEXT + 1];
uint8_t bannerSeverity = 0;

int visibleCount = 0;
int totalCount = 0;
int lastVisible = -1;
int lastTotal = -1;

char buffer[48];
uint8_t bufferLen = 0;

unsigned long lastMessageMs = 0;
bool linkLost = false;

uint16_t severityColour(uint8_t s) {
  switch (s) {
    case 1: return C_GREEN;
    case 2: return C_AMBER;
    case 3: return C_RED;
    default: return C_GREY;
  }
}

void drawBanner() {
  uint16_t colour = severityColour(bannerSeverity);
  tft.fillRect(0, 0, SCREEN_W, BANNER_H, colour);
  // Dark text on the light amber/green fills, white on the grey and red ones.
  tft.setTextColor((bannerSeverity == 1 || bannerSeverity == 2) ? C_BLACK : C_WHITE);
  tft.setTextSize(BANNER_TEXT_SIZE);
  tft.setCursor(6, (BANNER_H - 8 * BANNER_TEXT_SIZE) / 2);
  tft.print(bannerText);
}

void drawLabels() {
  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(BODY_LEFT, TOTAL_LABEL_Y);
  tft.print("DOLLS COUNTED (TOTAL)");
  tft.setCursor(BODY_LEFT, NOW_LABEL_Y);
  tft.print("SEEN NOW");
}

void drawCounts() {
  tft.fillRect(0, TOTAL_Y, SCREEN_W, TOTAL_H, C_BG);
  tft.setTextColor(C_GREEN);
  tft.setTextSize(6);
  tft.setCursor(BODY_LEFT, TOTAL_Y);
  tft.print(totalCount);

  tft.fillRect(0, NOW_Y, SCREEN_W, NOW_H, C_BG);
  tft.setTextColor(C_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(BODY_LEFT, NOW_Y);
  tft.print(visibleCount);
}

void drawRow(uint8_t row) {
  if (row >= NUM_ROWS) return;
  int16_t y = BODY_TOP + row * BODY_ROW_H;
  // Blank first: without this a shorter string leaves the tail of the
  // previous one on screen.
  tft.fillRect(0, y, SCREEN_W, BODY_ROW_H, C_BG);
  tft.setTextColor(row == 0 ? C_CYAN : C_WHITE);
  tft.setTextSize(BODY_TEXT_SIZE);
  tft.setCursor(BODY_LEFT, y);
  tft.print(rowText[row]);
}

void repaintAll() {
  tft.fillScreen(C_BG);
  drawBanner();
  drawLabels();
  drawCounts();
  for (uint8_t i = 0; i < NUM_ROWS; i++) drawRow(i);
}

void clearAll() {
  for (uint8_t i = 0; i < NUM_ROWS; i++) rowText[i][0] = '\0';
  bannerText[0] = '\0';
  bannerSeverity = 0;
  // NOT the counts. A clear is a screen operation, and silently zeroing a
  // cumulative mission count from a display command is how a real result
  // disappears.
  repaintAll();
}

void copyTruncated(char *dest, const char *src) {
  uint8_t i = 0;
  while (i < MAX_TEXT && src[i] != '\0') {
    dest[i] = src[i];
    i++;
  }
  dest[i] = '\0';
}

void handleCounts(const char *arg) {
  // "<visible>,<total>"
  const char *comma = strchr(arg, ',');
  if (comma == NULL) return;
  int v = atoi(arg);
  int t = atoi(comma + 1);
  if (v < 0 || t < 0) return;
  visibleCount = v;
  totalCount = t;
  if (visibleCount != lastVisible || totalCount != lastTotal) {
    drawCounts();
    lastVisible = visibleCount;
    lastTotal = totalCount;
  }
}

void handleLine(char *line) {
  lastMessageMs = millis();

  if (linkLost) {
    // Coming back from the NO LINK screen: repaint everything we still know.
    linkLost = false;
    repaintAll();
  }

  if (line[0] == 'C' && (line[1] == '\0' || (line[1] == ':' && line[2] == '\0'))) {
    clearAll();
    return;
  }

  if (line[1] != ':') return;

  switch (line[0]) {
    case 'S': {
      uint8_t s = line[2] - '0';
      if (s <= 3 && s != bannerSeverity) {
        bannerSeverity = s;
        drawBanner();
      }
      return;
    }
    case 'T':
      if (strncmp(bannerText, line + 2, MAX_TEXT) != 0) {
        copyTruncated(bannerText, line + 2);
        drawBanner();
      }
      return;
    case 'D':
      handleCounts(line + 2);
      return;
    default:
      if (line[0] >= '1' && line[0] <= '0' + NUM_ROWS) {
        uint8_t row = line[0] - '1';
        if (strncmp(rowText[row], line + 2, MAX_TEXT) != 0) {
          copyTruncated(rowText[row], line + 2);
          drawRow(row);
        }
      }
      return;
  }
}

void showNoLink() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, SCREEN_W, BANNER_H, C_RED);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(BANNER_TEXT_SIZE);
  tft.setCursor(6, (BANNER_H - 8 * BANNER_TEXT_SIZE) / 2);
  tft.print("NO LINK");
  tft.setTextSize(BODY_TEXT_SIZE);
  tft.setCursor(BODY_LEFT, BODY_TOP);
  tft.print("jetson silent");
  tft.setCursor(BODY_LEFT, BODY_TOP + BODY_ROW_H);
  tft.print("check usb / node");
  // The last known total, small, underneath: the count is the one thing worth
  // still being able to read off a screen whose link has died.
  tft.setCursor(BODY_LEFT, BODY_TOP + 2 * BODY_ROW_H);
  tft.print("last total: ");
  tft.print(totalCount);
}

void setup() {
  Serial.begin(115200);

  uint16_t id = tft.readID();
  // Some shields report 0x0000 or 0xD3D3; 0x9341 is the usual fallback that
  // drives them correctly anyway.
  if (id == 0x0000 || id == 0xD3D3 || id == 0xFFFF) id = 0x9341;
  tft.begin(id);
  tft.setRotation(0);            // portrait, 240x320
  tft.fillScreen(C_BG);

  for (uint8_t i = 0; i < NUM_ROWS; i++) rowText[i][0] = '\0';

  copyTruncated(bannerText, "BOOT");
  bannerSeverity = 0;
  copyTruncated(rowText[0], "window room mission");
  copyTruncated(rowText[1], "waiting for ROS");
  repaintAll();

  lastMessageMs = millis();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      buffer[bufferLen] = '\0';
      if (bufferLen > 0) handleLine(buffer);
      bufferLen = 0;
      continue;
    }

    // Overlong lines are dropped rather than wrapped, so a garbled byte
    // stream cannot desync the parser permanently.
    if (bufferLen < sizeof(buffer) - 1) {
      buffer[bufferLen++] = c;
    } else {
      bufferLen = 0;
    }
  }

  if (!linkLost && (millis() - lastMessageMs) > LINK_TIMEOUT_MS) {
    linkLost = true;
    showNoLink();
  }
}
