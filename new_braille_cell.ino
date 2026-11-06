/*
  Braille cell v1.2 — two cam drums, two 28BYJ-48 steppers, two ULN2003 boards, Arduino UNO R4.

  WHAT'S NEW IN v1.2
  - Backlash fix: every move finishes turning FORWARD, so the gear slack is always
    taken up the same way and the drum stops in the same place every time.
  - Precise homing with "edge": finds the exact centre of home from the pins, instead
    of guessing by eye.
  - "zero L" / "zero R": set home for one drum without touching the other.
  - Saves both drum positions after every move (survives unplugging), so the cell goes
    straight from letter to letter and remembers where it is after a restart.
  - "letter" prints which pins SHOULD be up, so you can compare at a glance.
  - "blank": all six pins down.

  FIRST-TIME SETUP (and any time the letters look wrong)
    Serial Monitor: 115200 baud, line ending "Newline".
    For each drum (L, then R):
      1. stop L 7            pins should read: far down, middle UP, near down
      2. step L 10           repeat until the NEAR pin is fully UP, then type:  edge L
      3. step L 10           repeat until the MIDDLE pin just starts to DROP, then type:  edge L
         -> the drum moves to the exact centre of home and saves it.
      (If in step 2 the FAR pin rises instead, that drum turns backwards:
       flip LEFT_DIR or RIGHT_DIR below, upload, and start again.)

  EVERYDAY COMMANDS
    letter h            show one letter (prints the expected pins)
    type hello          show text one letter at a time
    abc                 a to z
    blank               all pins down
    where               where the code thinks each drum is
*/

// ===================== SETTINGS =====================

// ULN2003 inputs IN1, IN2, IN3, IN4 for each drum
const int LEFT_PINS[4]  = {8, 9, 10, 11};
const int RIGHT_PINS[4] = {4, 5, 6, 7};

// Steps for one full drum turn. 2048 for most 28BYJ-48s; some are ~2038.
const float STEPS_PER_REV = 2048.0;

// Microseconds between steps. Bigger = slower, stronger, less pin bounce.
const unsigned int STEP_DELAY_US = 4000;

// +1 or -1. Flip if a drum turns the opposite way (see FIRST-TIME SETUP).
const int LEFT_DIR  = 1;
const int RIGHT_DIR = 1;

// Gear slack compensation. Any move that would end going backward overshoots by this
// many steps and comes back forward. 32 steps is about 5.6 degrees.
const int BACKLASH_STEPS = 32;

// How long each letter is shown by "type" and "abc" (ms)
const unsigned long LETTER_HOLD_MS = 3000;

// Pause at each stop during "cal" (ms)
const unsigned long CAL_PAUSE_MS = 1500;

// ===================== DRUM TABLE =====================
// Pattern bits: top<<2 | middle<<1 | bottom   (1 = dot up). top = far pin, bottom = near pin.
// Stop k is k*45 degrees from home. From the drum CAD:
//   stop:     0    1    2    3    4    5    6    7
//   pattern: 011  001  000  100  101  111  110  010
const byte STOP_PATTERN[8] = {0b011, 0b001, 0b000, 0b100, 0b101, 0b111, 0b110, 0b010};
byte PATTERN_TO_STOP[8];   // filled in setup()

// ===================== BRAILLE TABLE =====================
// Bit 0 = dot 1 ... bit 5 = dot 6.
// Dots 1,2,3 = left column far->near. Dots 4,5,6 = right column far->near.
#define D(a) (1 << ((a) - 1))
const byte LETTERS[26] = {
  D(1),                              // a
  D(1) | D(2),                       // b
  D(1) | D(4),                       // c
  D(1) | D(4) | D(5),                // d
  D(1) | D(5),                       // e
  D(1) | D(2) | D(4),                // f
  D(1) | D(2) | D(4) | D(5),         // g
  D(1) | D(2) | D(5),                // h
  D(2) | D(4),                       // i
  D(2) | D(4) | D(5),                // j
  D(1) | D(3),                       // k
  D(1) | D(2) | D(3),                // l
  D(1) | D(3) | D(4),                // m
  D(1) | D(3) | D(4) | D(5),         // n
  D(1) | D(3) | D(5),                // o
  D(1) | D(2) | D(3) | D(4),         // p
  D(1) | D(2) | D(3) | D(4) | D(5),  // q
  D(1) | D(2) | D(3) | D(5),         // r
  D(2) | D(3) | D(4),                // s
  D(2) | D(3) | D(4) | D(5),         // t
  D(1) | D(3) | D(6),                // u
  D(1) | D(2) | D(3) | D(6),         // v
  D(2) | D(4) | D(5) | D(6),         // w
  D(1) | D(3) | D(4) | D(6),         // x
  D(1) | D(3) | D(4) | D(5) | D(6),  // y
  D(1) | D(3) | D(5) | D(6),         // z
};
const byte NUMBER_SIGN = D(3) | D(4) | D(5) | D(6);
const byte BLANK = 0;

// ===================== DRUM STATE =====================
struct Drum {
  const char* name;
  const int* pins;
  int dir;
  long pos;        // steps from home (signed, never wrapped)
  byte phase;      // 0..3, current coil phase
  bool haveEdge;   // "edge" command: first edge recorded?
  long edgePos;    // "edge" command: position of first edge
};

Drum drums[2] = {
  {"left",  LEFT_PINS,  LEFT_DIR,  0, 0, false, 0},
  {"right", RIGHT_PINS, RIGHT_DIR, 0, 0, false, 0},
};

// Full-step, two coils on: strongest torque. Order is IN1..IN4.
const byte PHASES[4][4] = {
  {1, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 1},
  {1, 0, 0, 1},
};

// ===================== SAVED POSITION (EEPROM) =====================
// Uses only EEPROM.read / EEPROM.write so it works on both the UNO R3 and R4.
#include <EEPROM.h>
const byte SAVE_MARKER = 0xC2;   // v1.2 layout
const int  ADDR_MARKER = 0;
const int  ADDR_DATA   = 1;
const int  SAVE_SIZE   = 10;     // 2 x long (4 bytes) + 2 x phase (1 byte)

void writeByteIfChanged(int addr, byte v) {
  if (EEPROM.read(addr) != v) EEPROM.write(addr, v);
}

void savePosition() {
  byte buf[SAVE_SIZE];
  for (int i = 0; i < 2; i++) {
    uint32_t u = (uint32_t)(int32_t)drums[i].pos;
    for (int b = 0; b < 4; b++) buf[i * 4 + b] = (u >> (8 * b)) & 0xFF;
    buf[8 + i] = drums[i].phase;
  }
  for (int k = 0; k < SAVE_SIZE; k++) writeByteIfChanged(ADDR_DATA + k, buf[k]);
  writeByteIfChanged(ADDR_MARKER, SAVE_MARKER);
}

bool loadPosition() {
  if (EEPROM.read(ADDR_MARKER) != SAVE_MARKER) return false;
  for (int i = 0; i < 2; i++) {
    uint32_t u = 0;
    for (int b = 0; b < 4; b++) u |= ((uint32_t)EEPROM.read(ADDR_DATA + i * 4 + b)) << (8 * b);
    drums[i].pos = (long)(int32_t)u;
    drums[i].phase = EEPROM.read(ADDR_DATA + 8 + i) % 4;
  }
  return true;
}

// ===================== MOTOR =====================
void writePhase(Drum& d) {
  for (int i = 0; i < 4; i++) digitalWrite(d.pins[i], PHASES[d.phase][i] ? HIGH : LOW);
}

// Coils off and save. Every move ends here.
void release(Drum& d) {
  for (int i = 0; i < 4; i++) digitalWrite(d.pins[i], LOW);
  savePosition();
}

// Raw steps (positive = forward). Leaves coils powered.
void stepRaw(Drum& d, long n) {
  int s = (n >= 0) ? 1 : -1;
  long count = (n >= 0) ? n : -n;
  for (long i = 0; i < count; i++) {
    d.phase = (d.phase + 4 + s * d.dir) % 4;
    writePhase(d);
    d.pos += s;
    delayMicroseconds(STEP_DELAY_US);
  }
}

// Move by n steps, always FINISHING with a forward approach (takes up gear slack).
void moveBy(Drum& d, long n) {
  if (n < 0 && BACKLASH_STEPS > 0) {
    stepRaw(d, n - BACKLASH_STEPS);
    stepRaw(d, BACKLASH_STEPS);
  } else {
    stepRaw(d, n);
  }
  release(d);
}

// Where the drum is within one turn: 0 .. STEPS_PER_REV
float angleSteps(const Drum& d) {
  float a = fmod((float)d.pos, STEPS_PER_REV);
  if (a < 0) a += STEPS_PER_REV;
  return a;
}

// Steps to a stop by the shorter way around (never more than half a turn)
long stepsToStop(const Drum& d, int stop) {
  float target = stop * STEPS_PER_REV / 8.0;
  float delta = target - angleSteps(d);
  if (delta >  STEPS_PER_REV / 2) delta -= STEPS_PER_REV;
  if (delta < -STEPS_PER_REV / 2) delta += STEPS_PER_REV;
  return lround(delta);
}

int currentStop(const Drum& d) {
  return ((int)lround(angleSteps(d) / (STEPS_PER_REV / 8.0))) % 8;
}

// Straight from wherever the drum is to the stop — no return to home in between.
void moveToStop(Drum& d, int stop) {
  moveBy(d, stepsToStop(d, stop));
}

// ===================== CELL =====================
byte columnPattern(byte dots, int firstDot) {
  // firstDot = 1 for left column, 4 for right column
  byte top = (dots >> (firstDot - 1)) & 1;
  byte mid = (dots >> firstDot) & 1;
  byte bot = (dots >> (firstDot + 1)) & 1;
  return (top << 2) | (mid << 1) | bot;
}

void printPattern(byte p) {
  Serial.print("far ");    Serial.print((p >> 2) & 1 ? "UP" : "down");
  Serial.print(", mid ");  Serial.print((p >> 1) & 1 ? "UP" : "down");
  Serial.print(", near "); Serial.print(p & 1 ? "UP" : "down");
}

void showCell(byte dots) {
  byte lp = columnPattern(dots, 1);
  byte rp = columnPattern(dots, 4);
  int ls = PATTERN_TO_STOP[lp];
  int rs = PATTERN_TO_STOP[rp];
  // one motor at a time keeps current draw low on USB power
  moveToStop(drums[0], ls);
  moveToStop(drums[1], rs);
  Serial.print("  dots ");
  for (int i = 1; i <= 6; i++) if (dots & D(i)) Serial.print(i);
  if (!dots) Serial.print("none");
  Serial.println();
  Serial.print("    LEFT  (stop "); Serial.print(ls); Serial.print("): "); printPattern(lp); Serial.println();
  Serial.print("    RIGHT (stop "); Serial.print(rs); Serial.print("): "); printPattern(rp); Serial.println();
}

bool showChar(char c, bool& inNumber) {
  if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';
  if (c >= 'a' && c <= 'z') {
    inNumber = false;
    Serial.print("'"); Serial.print(c); Serial.print("'");
    showCell(LETTERS[c - 'a']);
    return true;
  }
  if (c >= '0' && c <= '9') {
    if (!inNumber) {
      Serial.print("number sign");
      showCell(NUMBER_SIGN);
      delay(LETTER_HOLD_MS);
      inNumber = true;
    }
    int idx = (c == '0') ? 9 : (c - '1');   // 1..9 -> a..i, 0 -> j
    Serial.print("'"); Serial.print(c); Serial.print("'");
    showCell(LETTERS[idx]);
    return true;
  }
  if (c == ' ') {
    inNumber = false;
    Serial.print("space");
    showCell(BLANK);
    return true;
  }
  return false;   // anything else is skipped
}

void showText(const String& s) {
  bool inNumber = false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (showChar(s[i], inNumber)) delay(LETTER_HOLD_MS);
  }
  Serial.println("done");
}

// ===================== COMMANDS =====================
Drum* pickDrum(const String& w, bool quiet = false) {
  if (w == "l" || w == "left")  return &drums[0];
  if (w == "r" || w == "right") return &drums[1];
  if (!quiet) Serial.println("  which drum? use L or R");
  return nullptr;
}

void printWhere() {
  for (int i = 0; i < 2; i++) {
    Serial.print("  "); Serial.print(drums[i].name);
    Serial.print(": step "); Serial.print(drums[i].pos);
    Serial.print(", nearest stop "); Serial.println(currentStop(drums[i]));
  }
}

void printHelp() {
  Serial.println(F("Commands (L = left drum, R = right drum):"));
  Serial.println(F("  letter <c>        show one character (prints the expected pins)"));
  Serial.println(F("  type <text>       show text one character at a time"));
  Serial.println(F("  abc               show a to z"));
  Serial.println(F("  blank             all pins down"));
  Serial.println(F("  stop L|R <0-7>    move one drum to a stop"));
  Serial.println(F("  step L|R <n>      move n steps (negative = backward)"));
  Serial.println(F("  edge L|R          precise homing: type at each of the two edges"));
  Serial.println(F("  zero              both drums: here is home"));
  Serial.println(F("  zero L|R          one drum: here is home"));
  Serial.println(F("  home              both drums to stop 0"));
  Serial.println(F("  spin L|R          one full turn forward, then back"));
  Serial.println(F("  cal L|R           visit all 8 stops, end at home"));
  Serial.println(F("  where             print each drum's position"));
  Serial.println(F("  release           turn off all coils"));
}

void handle(String line) {
  line.trim();
  String lower = line;
  lower.toLowerCase();
  int sp = lower.indexOf(' ');
  String cmd  = (sp < 0) ? lower : lower.substring(0, sp);
  String rest = (sp < 0) ? "" : line.substring(sp + 1);
  rest.trim();
  String restLower = rest;
  restLower.toLowerCase();
  int sp2 = restLower.indexOf(' ');
  String arg1 = (sp2 < 0) ? restLower : restLower.substring(0, sp2);
  String arg2 = (sp2 < 0) ? "" : restLower.substring(sp2 + 1);
  arg2.trim();

  if (cmd == "help" || cmd == "?") { printHelp(); return; }

  if (cmd == "spin") {
    Drum* d = pickDrum(arg1); if (!d) return;
    Serial.println("  forward one turn...");
    stepRaw(*d, lround(STEPS_PER_REV));
    delay(500);
    Serial.println("  back one turn...");
    moveBy(*d, -lround(STEPS_PER_REV));
    Serial.println("  done");
    return;
  }

  if (cmd == "step") {
    Drum* d = pickDrum(arg1); if (!d) return;
    moveBy(*d, arg2.toInt());
    Serial.println("  done");
    return;
  }

  if (cmd == "stop") {
    Drum* d = pickDrum(arg1); if (!d) return;
    int s = arg2.toInt();
    if (arg2.length() == 0 || s < 0 || s > 7) { Serial.println("  stop must be 0-7"); return; }
    moveToStop(*d, s);
    Serial.print("  "); Serial.print(d->name); Serial.print(" at stop "); Serial.print(s);
    Serial.print(": "); printPattern(STOP_PATTERN[s]); Serial.println();
    return;
  }

  if (cmd == "edge") {
    Drum* d = pickDrum(arg1); if (!d) return;
    if (!d->haveEdge) {
      d->haveEdge = true;
      d->edgePos = d->pos;
      Serial.println(F("  first edge saved. Now keep typing \"step <L|R> 10\" until the MIDDLE pin"));
      Serial.println(F("  just starts to drop, then type \"edge <L|R>\" again."));
    } else {
      d->haveEdge = false;
      long span = d->pos - d->edgePos;
      if (span <= 0) {
        Serial.println(F("  second edge must be FORWARD of the first. Start again from \"stop L 7\"."));
        return;
      }
      long mid = d->edgePos + span / 2;
      moveBy(*d, mid - d->pos);   // goes back past the middle, then forward onto it
      d->pos = 0;
      savePosition();
      Serial.print("  "); Serial.print(d->name);
      Serial.print(" home set at the centre (the two edges were ");
      Serial.print(span); Serial.println(" steps apart).");
      Serial.println(F("  Pins should read: far down, mid UP, near UP"));
    }
    return;
  }

  if (cmd == "zero") {
    Drum* d = pickDrum(arg1, true);
    if (d) {
      d->pos = 0;
      Serial.print("  "); Serial.print(d->name); Serial.println(" drum set to stop 0 and saved");
    } else {
      drums[0].pos = drums[1].pos = 0;
      Serial.println("  both drums set to stop 0 and saved");
    }
    savePosition();
    return;
  }

  if (cmd == "home") {
    moveToStop(drums[0], 0);
    moveToStop(drums[1], 0);
    Serial.println("  both drums at stop 0: far down, mid UP, near UP");
    return;
  }

  if (cmd == "blank") {
    showCell(BLANK);
    return;
  }

  if (cmd == "cal") {
    Drum* d = pickDrum(arg1); if (!d) return;
    moveToStop(*d, 0);
    Serial.print("  stop 0: "); printPattern(STOP_PATTERN[0]); Serial.println();
    delay(CAL_PAUSE_MS);
    for (int s = 1; s <= 8; s++) {
      stepRaw(*d, lround(s * STEPS_PER_REV / 8.0) - lround((s - 1) * STEPS_PER_REV / 8.0));
      release(*d);
      Serial.print("  stop "); Serial.print(s % 8); Serial.print(": ");
      printPattern(STOP_PATTERN[s % 8]); Serial.println();
      delay(CAL_PAUSE_MS);
    }
    Serial.println("  back at stop 0");
    return;
  }

  if (cmd == "letter") {
    bool inNumber = false;
    if (rest.length() == 0 || !showChar(rest[0], inNumber)) Serial.println("  can't show that character");
    return;
  }

  if (cmd == "type") { showText(rest); return; }

  if (cmd == "abc") { showText("abcdefghijklmnopqrstuvwxyz"); return; }

  if (cmd == "where") { printWhere(); return; }

  if (cmd == "release") {
    for (int i = 0; i < 2; i++) release(drums[i]);
    Serial.println("  coils off");
    return;
  }

  if (cmd.length()) { Serial.print("  unknown command: "); Serial.println(cmd); }
}

// ===================== MAIN =====================
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }   // UNO R4: wait briefly for the Serial Monitor

  for (int s = 0; s < 8; s++) PATTERN_TO_STOP[STOP_PATTERN[s]] = s;
  bool restored = loadPosition();
  for (int i = 0; i < 2; i++) {
    for (int p = 0; p < 4; p++) pinMode(drums[i].pins[p], OUTPUT);
    for (int p = 0; p < 4; p++) digitalWrite(drums[i].pins[p], LOW);
  }

  if (restored) {
    Serial.println(F("Braille cell v1.2 ready. Restored saved drum positions:"));
    printWhere();
  } else {
    Serial.println(F("Braille cell v1.2 ready. No saved position yet: do FIRST-TIME SETUP (edge L, edge R)."));
  }
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handle(line);
  }
}
