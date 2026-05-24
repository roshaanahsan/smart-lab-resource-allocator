// ============================================================
// Smart Resource Allocation & Deadlock Detection System
// ESP32 firmware for lab equipment management
// Author: Roshaan Ahsan | github.com/roshaanahsan
// ============================================================

#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// --------------------- Pin Definitions -----------------------
#define RC522_SDA_PIN   5
#define RC522_RST_PIN   4
#define LCD_SDA_PIN     21
#define LCD_SCL_PIN     22
#define LED_GREEN       15
#define LED_YELLOW      2
#define LED_RED         16
#define BUZZER_PIN      17
#define EXPORT_BTN_PIN  34

// --------------------- System Limits -------------------------
#define MAX_STUDENTS    10
#define MAX_EQUIPMENT   15
#define MAX_NODES       (MAX_STUDENTS + MAX_EQUIPMENT)
#define QUEUE_SIZE      MAX_STUDENTS
#define LCD_COLS        16
#define LCD_ROWS        2
#define SERIAL_DIGITS   6
#define BUZZER_BEEP_MS  150
#define BUZZER_ALARM_MS 800
#define IDLE_TIMEOUT_MS 15000
#define DEBOUNCE_MS     300

// --------------------- WiFi Configuration --------------------
bool USE_AP_MODE = true;
const char* WIFI_SSID     = "YourWiFiName";
const char* WIFI_PASSWORD = "YourWiFiPassword";
const char* AP_SSID = "LabSystem";
const char* AP_PASS = "lab12345";

// --------------------- Equipment Type Table -------------------
struct EquipType {
  int typeCode;
  const char* name;       // full name (for serial / web)
  const char* shortName;  // exactly 8 chars for LCD display
};

const EquipType EQUIP_TYPES[] = {
  {0,   "Oscilloscope",      "Oscillo."},
  {1,   "Multimeter",        "Multim."},
  {2,   "Function Gen",      "FuncGen."},
  {3,   "Power Supply",      "PwrSupp."},
  {4,   "Logic Analyzer",    "LogicA."},
  {5,   "Signal Generator",  "SigGen."},
  {6,   "Spectrum Analyzer", "SpecAn."},
  {7,   "Soldering Iron",    "Solder."},
  {8,   "Oscilloscope Pro",  "OscPro."},
  {9,   "DC Supply",         "DCSupp."},
  {999, "Unknown Device",    "Unknow."},
};
const int NUM_EQUIP_TYPES = sizeof(EQUIP_TYPES) / sizeof(EquipType);

// --------------------- Data Structures ------------------------
struct Student {
  bool active;
  char uid[20];          // RFID card UID
  char name[20];         // first name (fits LCD)
  char studentId[12];    // short ID e.g. "05"
};

struct EquipQueue {
  int data[QUEUE_SIZE];  // waiting student indices
  int front, rear, count;
};

struct Equipment {
  bool active;
  int serial;            // full 6-digit integer
  char serialStr[8];     // zero-padded string
  int typeCode;          // first 3 digits
  int unitNum;           // last 3 digits
  bool available;        // true = on shelf
  int heldBy;            // student index (-1 if free)
  EquipQueue waitQueue;
  unsigned long borrowTime;
};

// Resource allocation graph: adjacency matrix
int graph[MAX_NODES][MAX_NODES];
Student   students[MAX_STUDENTS];
Equipment equipment[MAX_EQUIPMENT];
int numStudents  = 0;
int numEquipment = 0;

// --------------------- UI State Machine -----------------------
enum UIState {
  STATE_IDLE,
  STATE_STUDENT_MENU,
  STATE_BORROW_ENTER_SERIAL,
  STATE_RETURN_ENTER_SERIAL,
  STATE_VIEW_ITEMS,
  STATE_ADMIN_MENU,
  STATE_ADMIN_REGISTER_SCAN,
  STATE_ADMIN_REGISTER_NAME,
  STATE_DEADLOCK_ALERT,
  STATE_MSG_SUCCESS,
  STATE_MSG_ERROR,
  STATE_MSG_QUEUED
};

UIState  uiState = STATE_IDLE;
int      activeStudent = -1;
char     inputBuffer[16] = {0};
int      inputLen = 0;
String   tempName, tempStudentId, tempUID;
int      viewItemIndex = 0, viewItemCount = 0;
int      viewItems[MAX_EQUIPMENT];
unsigned long lastActivityMs = 0, msgShowUntilMs = 0;
UIState  returnState = STATE_IDLE;
String   deadlockPath;

// --------------------- Hardware Objects -----------------------
MFRC522 rfid(RC522_SDA_PIN, RC522_RST_PIN);
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);
Preferences prefs;
AsyncWebServer server(80);

const byte KEYPAD_ROWS = 4, KEYPAD_COLS = 4;
char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[KEYPAD_ROWS] = {13, 12, 14, 27};
byte colPins[KEYPAD_COLS] = {26, 25, 33, 32};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);
unsigned long lastKeyMs = 0;

// --------------------- Forward Declarations -------------------
void lcdLine(int row, const char* text);
void lcdPrint(const char* line1, const char* line2);
void beepSuccess(), beepQueued(), beepError(), beepDeadlockAlarm();
void ledSuccess(), ledQueued(), ledDeadlock(), allLedsOff();
int  findStudentByUID(const char* uid);
int  findEquipmentBySerial(int serial);
void queueInit(EquipQueue* q);
bool queueEmpty(EquipQueue* q), queueFull(EquipQueue* q);
void queuePush(EquipQueue* q, int idx);
int  queuePop(EquipQueue* q);
void graphAddEdge(int from, int to), graphRemoveEdge(int from, int to);
void graphClear();
bool dfsCycleDetect(int v, bool* visited, bool* recStack, int* path, int* pathLen);
bool checkDeadlock(String& cycleInfo);
void handleDeadlockDetected(String& cycleInfo);
void borrowEquipment(int studentIdx, int equipIdx);
bool returnEquipment(int studentIdx, int equipIdx, String& msg);
void processBorrowRequest(int serial);
void processReturnRequest(int serial);
void buildViewItemsList();
void saveStudentsToFlash(), loadStudentsFromFlash();
void saveEquipmentToFlash(), loadEquipmentFromFlash();
void handleRFID(), handleKeypad(char key);
void updateUI(), renderIdleScreen(), renderStudentMenu();
void renderEnterSerial(const char* prompt), renderViewItems(), renderAdminMenu();
void showWiFiInfo();
String uidToString(byte* uid, byte size);
String equipShortName(int equipIdx);
const char* getEquipTypeName(int typeCode);
const char* getEquipTypeShortName(int typeCode);
void setupWebServer();
String buildHTMLPage(), buildCSV(), buildJSONData(), buildGraphText();
void dumpSystemState();
void initDefaultEquipment(), initDefaultStudents();
void registerNewStudent(const char* uid, const char* name, const char* sid);

// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== SMART LAB SYSTEM v2.0 (Realistic logic) ==="));

  // GPIO
  pinMode(LED_GREEN, OUTPUT); pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);   pinMode(BUZZER_PIN, OUTPUT);
  pinMode(EXPORT_BTN_PIN, INPUT_PULLDOWN);
  allLedsOff(); digitalWrite(BUZZER_PIN, LOW);

  // LCD
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.begin(); lcd.backlight(); lcd.clear();
  lcdPrint("  Lab Mgmt Sys  ", "  Initializing..");

  // RFID
  SPI.begin();
  rfid.PCD_Init();
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.printf("[RFID] Version: 0x%02X\n", v);

  // Flash storage
  prefs.begin("labsys", false);
  loadStudentsFromFlash();
  loadEquipmentFromFlash();
  if (numStudents == 0) initDefaultStudents();
  if (numEquipment == 0) initDefaultEquipment();
  graphClear();

  // WiFi
  if (USE_AP_MODE) {
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WIFI] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      USE_AP_MODE = true;
      WiFi.softAP(AP_SSID, AP_PASS);
    }
    Serial.printf("[WIFI] IP: %s\n", WiFi.localIP().toString().c_str());
  }

  // Re-init SPI and RFID after WiFi (prevents SPI conflict)
  SPI.begin(18, 19, 23, 5);
  rfid.PCD_Init();
  rfid.PCD_SetAntennaGain(rfid.RxGain_max);
  Serial.printf("[RFID] Post-WiFi v: 0x%02X\n", rfid.PCD_ReadRegister(MFRC522::VersionReg));

  // Web server
  setupWebServer();
  server.begin();

  dumpSystemState();
  beepSuccess();
  lcdPrint("  System Ready! ", " Scan your card ");
  lastActivityMs = millis();
  uiState = STATE_IDLE;
}

void loop() {
  handleRFID();

  char key = keypad.getKey();
  if (key) {
    unsigned long now = millis();
    if (now - lastKeyMs > DEBOUNCE_MS) {
      lastKeyMs = now;
      lastActivityMs = now;
      Serial.printf("[KEY] '%c' State:%d\n", key, uiState);
      handleKeypad(key);
    }
  }

  // Export button (active low)
  static unsigned long lastBtnMs = 0;
  if (digitalRead(EXPORT_BTN_PIN) == LOW && millis() - lastBtnMs > 2000) {
    lastBtnMs = millis();
    dumpSystemState();
    lcdPrint(" Data exported! ", "  Check Serial  ");
    beepSuccess(); delay(2000); updateUI();
  }

  // Idle timeout
  if (uiState != STATE_IDLE && uiState != STATE_DEADLOCK_ALERT) {
    if (millis() - lastActivityMs > IDLE_TIMEOUT_MS) {
      activeStudent = -1; inputLen = 0;
      memset(inputBuffer, 0, sizeof(inputBuffer));
      uiState = STATE_IDLE; allLedsOff(); updateUI();
    }
  }

  // Message timeout (success/error/queued)
  if (uiState == STATE_MSG_SUCCESS || uiState == STATE_MSG_ERROR ||
      uiState == STATE_MSG_QUEUED) {
    if (millis() > msgShowUntilMs) {
      uiState = returnState; updateUI();
    }
  }

  // Keep idle screen updated for IP address alternation
  static unsigned long lastIdleRender = 0;
  if (uiState == STATE_IDLE && millis() - lastIdleRender > 500) {
    lastIdleRender = millis();
    renderIdleScreen();
  }
}

// ============================================================
// RFID Handler
// ============================================================
void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  delay(50);
  if (!rfid.PICC_ReadCardSerial()) return;

  String uid = uidToString(rfid.uid.uidByte, rfid.uid.size);
  Serial.printf("[RFID] Card: %s\n", uid.c_str());
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  lastActivityMs = millis();
  int idx = findStudentByUID(uid.c_str());

  if (uiState == STATE_ADMIN_REGISTER_SCAN) {
    tempUID = uid;
    if (idx >= 0) {
      lcdPrint("Already Reg'd!  ", students[idx].name);
      beepError(); delay(2000);
    } else {
      lcdPrint("Card scanned OK ", "Enter ID->keys  ");
      beepSuccess();
      tempName = ""; inputLen = 0;
      memset(inputBuffer, 0, sizeof(inputBuffer));
      uiState = STATE_ADMIN_REGISTER_NAME;
    }
    return;
  }

  if (idx < 0) {
    lcdPrint("Unknown Card!   ", "See admin 'D'   ");
    beepError(); ledDeadlock(); delay(1500); allLedsOff();
    if (uiState == STATE_IDLE) renderIdleScreen();
    return;
  }

  activeStudent = idx;
  Serial.printf("[AUTH] %s (%s)\n", students[idx].name, students[idx].studentId);
  uiState = STATE_STUDENT_MENU;
  updateUI();
  beepSuccess(); ledSuccess(); delay(300); allLedsOff();
}

// ============================================================
// Keypad Handler (UI State Machine)
// ============================================================
void handleKeypad(char key) {
  if (key == '*') {   // Cancel / Back
    inputLen = 0; memset(inputBuffer, 0, sizeof(inputBuffer));
    if (uiState == STATE_IDLE) return;
    if (uiState == STATE_STUDENT_MENU || uiState == STATE_ADMIN_MENU) {
      activeStudent = -1; uiState = STATE_IDLE;
    } else if (uiState == STATE_BORROW_ENTER_SERIAL ||
               uiState == STATE_RETURN_ENTER_SERIAL ||
               uiState == STATE_VIEW_ITEMS) {
      uiState = STATE_STUDENT_MENU;
    } else if (uiState == STATE_ADMIN_REGISTER_SCAN ||
               uiState == STATE_ADMIN_REGISTER_NAME) {
      uiState = STATE_ADMIN_MENU;
    } else if (uiState == STATE_DEADLOCK_ALERT) {
      uiState = STATE_STUDENT_MENU;
    } else {
      uiState = STATE_IDLE;
    }
    updateUI(); return;
  }

  switch (uiState) {
    case STATE_IDLE:
      if (key == 'D') { uiState = STATE_ADMIN_MENU; updateUI(); }
      break;

    case STATE_STUDENT_MENU:
      if (key == 'A') { inputLen=0; memset(inputBuffer,0,sizeof(inputBuffer)); uiState=STATE_BORROW_ENTER_SERIAL; updateUI(); }
      else if (key == 'B') { inputLen=0; memset(inputBuffer,0,sizeof(inputBuffer)); uiState=STATE_RETURN_ENTER_SERIAL; updateUI(); }
      else if (key == 'C') { buildViewItemsList(); viewItemIndex=0; uiState=STATE_VIEW_ITEMS; updateUI(); }
      else if (key == 'D') { uiState=STATE_ADMIN_MENU; updateUI(); }
      break;

    case STATE_BORROW_ENTER_SERIAL:
      if (key >= '0' && key <= '9') {
        if (inputLen < SERIAL_DIGITS) {
          inputBuffer[inputLen++] = key; inputBuffer[inputLen] = '\0';
          renderEnterSerial("Borrow Serial:");
        }
        if (inputLen == SERIAL_DIGITS) processBorrowRequest(atoi(inputBuffer));
      } else if (key == '#') {
        if (inputLen == SERIAL_DIGITS) processBorrowRequest(atoi(inputBuffer));
        else { lcdPrint("Need 6 digits!", "Keep typing..."); delay(1000); renderEnterSerial("Borrow Serial:"); }
      }
      break;

    case STATE_RETURN_ENTER_SERIAL:
      if (key >= '0' && key <= '9') {
        if (inputLen < SERIAL_DIGITS) {
          inputBuffer[inputLen++] = key; inputBuffer[inputLen] = '\0';
          renderEnterSerial("Return Serial:");
        }
        if (inputLen == SERIAL_DIGITS) processReturnRequest(atoi(inputBuffer));
      } else if (key == '#') {
        if (inputLen == SERIAL_DIGITS) processReturnRequest(atoi(inputBuffer));
        else { lcdPrint("Need 6 digits!", "Keep typing..."); delay(1000); renderEnterSerial("Return Serial:"); }
      }
      break;

    case STATE_VIEW_ITEMS:
      if (key == '#' || key == 'C') {
        viewItemIndex = (viewItemIndex + 1) % (viewItemCount > 0 ? viewItemCount : 1);
        updateUI();
      } else if (key == 'A') { uiState = STATE_STUDENT_MENU; updateUI(); }
      break;

    case STATE_ADMIN_MENU:
      if (key == 'A') { lcdPrint("Scan new card","now..."); uiState=STATE_ADMIN_REGISTER_SCAN; }
      else if (key == 'B') { dumpSystemState(); lcdPrint("Data dumped","Serial Monitor"); beepSuccess(); delay(2000); updateUI(); }
      else if (key == 'D') showWiFiInfo();
      break;

    case STATE_ADMIN_REGISTER_NAME:
      if (key >= '0' && key <= '9') {
        if (tempName.length() < 15) {
          tempName += key;
          char buf[17]; snprintf(buf,17,"ID: %-12s", tempName.c_str());
          lcdPrint("Enter Stud ID:", buf);
        }
      } else if (key == '#') {
        if (tempName.length() > 0) {
          tempStudentId = tempName;
          char autoName[20]; snprintf(autoName,20,"Student %d", numStudents+1);
          registerNewStudent(tempUID.c_str(), autoName, tempStudentId.c_str());
          tempName=""; tempStudentId=""; tempUID="";
          uiState=STATE_ADMIN_MENU; updateUI();
        } else { lcdPrint("Enter student","ID then press #"); }
      }
      break;

    case STATE_DEADLOCK_ALERT:
      allLedsOff(); digitalWrite(BUZZER_PIN, LOW);
      uiState = STATE_STUDENT_MENU; updateUI();
      break;
  }
}

// ============================================================
// Borrow Logic – respects queue; only the first waiting student
// can borrow an available item. Others are rejected.
// ============================================================
void processBorrowRequest(int serial) {
  Serial.printf("[BORROW] Student %s serial %06d\n", students[activeStudent].name, serial);
  int eq = findEquipmentBySerial(serial);
  if (eq < 0) {
    lcdPrint("Not in system!", "Check serial");
    beepError(); delay(2000);
    inputLen=0; memset(inputBuffer,0,sizeof(inputBuffer));
    renderEnterSerial("Borrow Serial:");
    return;
  }
  if (equipment[eq].heldBy == activeStudent) {
    lcdPrint("You already have", "this item!");
    beepError(); delay(2000);
    uiState = STATE_STUDENT_MENU; updateUI();
    return;
  }

  // Item is available – check queue
  if (equipment[eq].available) {
    EquipQueue* q = &equipment[eq].waitQueue;

    // If there's a queue, only the front student can proceed
    if (!queueEmpty(q)) {
      int firstStudent = q->data[q->front];
      if (activeStudent != firstStudent) {
        char buf[17];
        snprintf(buf,17,"Reserved for %s", students[firstStudent].name);
        lcdPrint("Not your turn!  ", buf);
        beepError(); delay(2000);
        uiState = STATE_STUDENT_MENU; updateUI();
        return;
      } else {
        // Remove from queue, delete request edge
        queuePop(q);
        graphRemoveEdge(activeStudent, MAX_STUDENTS+eq);
      }
    }

    // Assign the item
    borrowEquipment(activeStudent, eq);
    char buf[17]; snprintf(buf,17,"%-16s", equipShortName(eq).c_str());
    lcdPrint(buf, "Assigned to you!");
    String ci; if(checkDeadlock(ci)){ handleDeadlockDetected(ci); return; }
    beepSuccess(); ledSuccess(); delay(300); allLedsOff();
    msgShowUntilMs = millis()+2500; returnState=STATE_STUDENT_MENU; uiState=STATE_MSG_SUCCESS;
  } else {
    // Item is busy – enqueue student
    EquipQueue* q = &equipment[eq].waitQueue;

    // Check for duplicate queue entry
    for (int i=0; i<q->count; i++) {
      if (q->data[(q->front+i)%QUEUE_SIZE] == activeStudent) {
        lcdPrint("Already queued!", "Please wait...");
        beepError(); delay(2000);
        uiState = STATE_STUDENT_MENU; updateUI();
        return;
      }
    }

    if (queueFull(q)) {
      lcdPrint("Queue full!", "Try later...");
      beepError(); delay(2000);
      uiState=STATE_STUDENT_MENU; updateUI();
      return;
    }
    queuePush(q, activeStudent);
    // Add request edge: student -> equipment
    graphAddEdge(activeStudent, MAX_STUDENTS+eq);

    char buf[17]; snprintf(buf,17,"%-16s", equipShortName(eq).c_str());
    char buf2[17]; snprintf(buf2,17,"Queued pos #%d", q->count);
    lcdPrint(buf, buf2);
    String ci; if(checkDeadlock(ci)){ handleDeadlockDetected(ci); return; }
    beepQueued(); ledQueued(); delay(300); allLedsOff();
    msgShowUntilMs = millis()+2500; returnState=STATE_STUDENT_MENU; uiState=STATE_MSG_QUEUED;
  }
  inputLen=0; memset(inputBuffer,0,sizeof(inputBuffer));
}

// ============================================================
// Return Logic – frees the item but does NOT auto-assign.
// Displays who is next in line.
// ============================================================
void processReturnRequest(int serial) {
  Serial.printf("[RETURN] Student %s serial %06d\n", students[activeStudent].name, serial);
  int eq = findEquipmentBySerial(serial);
  if (eq < 0) {
    lcdPrint("Item not found!","Check serial #");
    beepError(); delay(2000);
    inputLen=0; memset(inputBuffer,0,sizeof(inputBuffer));
    renderEnterSerial("Return Serial:");
    return;
  }
  if (equipment[eq].heldBy != activeStudent) {
    lcdPrint("Not yours!", "You don't hold");
    beepError(); delay(2000);
    uiState = STATE_STUDENT_MENU; updateUI();
    return;
  }

  // Remove assignment edge
  graphRemoveEdge(MAX_STUDENTS+eq, activeStudent);
  equipment[eq].available = true;
  equipment[eq].heldBy = -1;
  unsigned long held = (millis()-equipment[eq].borrowTime)/1000;
  Serial.printf("[RETURN] %s returned %s (held %lus)\n", students[activeStudent].name, equipment[eq].serialStr, held);

  char buf[17]; snprintf(buf,17,"%-16s", equipShortName(eq).c_str());
  char buf2[17];

  if (!queueEmpty(&equipment[eq].waitQueue)) {
    int next = equipment[eq].waitQueue.data[equipment[eq].waitQueue.front];
    snprintf(buf2,17,"Next: %s        ", students[next].name);
  } else {
    snprintf(buf2,17,"Returned. Thanks");
  }
  lcdPrint(buf, buf2);

  String ci; if(checkDeadlock(ci)){ handleDeadlockDetected(ci); return; }
  beepSuccess(); ledSuccess(); delay(300); allLedsOff();
  msgShowUntilMs = millis()+2500; returnState=STATE_STUDENT_MENU; uiState=STATE_MSG_SUCCESS;
  inputLen=0; memset(inputBuffer,0,sizeof(inputBuffer));
}

// ============================================================
// Graph & Deadlock Detection (DSA Core)
// ============================================================
void borrowEquipment(int s, int e) {
  equipment[e].available = false;
  equipment[e].heldBy = s;
  equipment[e].borrowTime = millis();
  // Assignment edge: equipment -> student
  graphAddEdge(MAX_STUDENTS+e, s);
  // Remove any request edge from this student
  graphRemoveEdge(s, MAX_STUDENTS+e);
}

bool dfsCycleDetect(int v, bool* visited, bool* recStack,
                    int* path, int* pathLen) {
  visited[v]   = true;
  recStack[v]  = true;
  path[(*pathLen)++] = v;

  for (int u = 0; u < MAX_NODES; u++) {
    if (graph[v][u] == 0) continue;
    if (!visited[u]) {
      if (dfsCycleDetect(u, visited, recStack, path, pathLen)) return true;
    } else if (recStack[u]) {
      // Back-edge found -> cycle (deadlock)
      path[(*pathLen)++] = u;
      return true;
    }
  }

  recStack[v] = false;
  (*pathLen)--;
  return false;
}

bool checkDeadlock(String& cycleInfo) {
  bool visited[MAX_NODES]   = {false};
  bool recStack[MAX_NODES]  = {false};
  int  path[MAX_NODES * 2]  = {0};
  int  pathLen = 0;

  for (int i = 0; i < MAX_NODES; i++) {
    if (!visited[i]) {
      pathLen = 0;
      if (dfsCycleDetect(i, visited, recStack, path, &pathLen)) {
        cycleInfo = "";
        int start = path[pathLen - 1];
        int begin = 0;
        for (int k = 0; k < pathLen - 1; k++)
          if (path[k] == start) { begin = k; break; }

        // Build cycle string (short IDs + serials)
        for (int k = begin; k < pathLen; k++) {
          int node = path[k];
          if (node < MAX_STUDENTS) {
            cycleInfo += students[node].studentId;
          } else {
            cycleInfo += equipment[node - MAX_STUDENTS].serialStr;
          }
          if (k < pathLen - 1) cycleInfo += "->";
        }
        return true;
      }
    }
  }
  return false;
}

void handleDeadlockDetected(String& cycleInfo) {
  deadlockPath = cycleInfo;
  Serial.println(F("[DEADLOCK] *** DETECTED ***"));
  Serial.println(cycleInfo);

  ledDeadlock();
  for (int i = 0; i < 4; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_ALARM_MS);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }

  lcdPrint("!! DEADLOCK !!", "Press any key");
  delay(1000);

  // Scroll the path on the LCD
  String sc = cycleInfo;
  for (int s = 0; s <= (int)sc.length() - 16; s++) {
    char buf[17];
    snprintf(buf, 17, "%-16s", sc.substring(s, s + 16).c_str());
    lcdLine(1, buf);
    delay(400);
  }

  uiState = STATE_DEADLOCK_ALERT;
  updateUI();
}

// Graph helpers
void graphClear() {
  memset(graph, 0, sizeof(graph));
}

void graphAddEdge(int from, int to) {
  if (from >= 0 && from < MAX_NODES && to >= 0 && to < MAX_NODES)
    graph[from][to] = 1;
}

void graphRemoveEdge(int from, int to) {
  if (from >= 0 && from < MAX_NODES && to >= 0 && to < MAX_NODES)
    graph[from][to] = 0;
}

// Queue helpers (circular buffer)
void queueInit(EquipQueue* q) {
  q->front = q->rear = q->count = 0;
  memset(q->data, -1, sizeof(q->data));
}

bool queueEmpty(EquipQueue* q) { return q->count == 0; }
bool queueFull(EquipQueue* q)  { return q->count >= QUEUE_SIZE; }

void queuePush(EquipQueue* q, int idx) {
  if (queueFull(q)) return;
  q->data[q->rear] = idx;
  q->rear = (q->rear + 1) % QUEUE_SIZE;
  q->count++;
}

int queuePop(EquipQueue* q) {
  if (queueEmpty(q)) return -1;
  int val = q->data[q->front];
  q->front = (q->front + 1) % QUEUE_SIZE;
  q->count--;
  return val;
}

// ============================================================
// LCD Helpers (16x2)
// ============================================================
void lcdLine(int row, const char* text) {
  lcd.setCursor(0, row);
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);
  lcd.print(buf);
}

void lcdPrint(const char* line1, const char* line2) {
  lcdLine(0, line1);
  lcdLine(1, line2);
}

// ============================================================
// UI Rendering (all formatted for 16x2)
// ============================================================
void updateUI() {
  switch (uiState) {
    case STATE_IDLE:               renderIdleScreen(); break;
    case STATE_STUDENT_MENU:       renderStudentMenu(); break;
    case STATE_BORROW_ENTER_SERIAL: renderEnterSerial("Borrow Serial:"); break;
    case STATE_RETURN_ENTER_SERIAL: renderEnterSerial("Return Serial:"); break;
    case STATE_VIEW_ITEMS:         renderViewItems(); break;
    case STATE_ADMIN_MENU:         renderAdminMenu(); break;
    case STATE_ADMIN_REGISTER_SCAN: lcdPrint("Scan new card","or * to cancel"); break;
    case STATE_ADMIN_REGISTER_NAME: lcdPrint("Enter Stud ID:","then press #"); break;
    case STATE_DEADLOCK_ALERT: {
      char buf[17];
      snprintf(buf, 17, "%-16s", deadlockPath.substring(0, 16).c_str());
      lcdPrint("!! DEADLOCK !!", buf);
      break;
    }
    default: break;
  }
}

void renderIdleScreen() {
  static unsigned long lastToggle = 0;
  static bool showIP = false;
  if (millis() - lastToggle > 3000) {
    showIP = !showIP;
    lastToggle = millis();
  }
  if (showIP) {
    String ip = USE_AP_MODE ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    char buf[17];
    snprintf(buf, sizeof(buf), "IP:%-13s", ip.c_str());
    lcdPrint(" Scan Your Card ", buf);
  } else {
    lcdPrint(" Scan Your Card ", " to get started ");
  }
}

void renderStudentMenu() {
  if (activeStudent < 0) {
    lcdPrint("Error: no auth", "Scan card first");
    return;
  }
  char n[9], id[8];
  strncpy(n, students[activeStudent].name, 8); n[8] = 0;
  strncpy(id, students[activeStudent].studentId, 7); id[7] = 0;
  char l1[17];
  snprintf(l1, sizeof(l1), "%-8s %-7s", n, id);
  lcdPrint(l1, "A:Get B:Ret C:My");
}

void renderEnterSerial(const char* prompt) {
  char l1[17];
  snprintf(l1, sizeof(l1), "%-16s", prompt);
  char l2[17] = "      ";
  for (int i = 0; i < SERIAL_DIGITS; i++) {
    if (i < inputLen) l2[i] = inputBuffer[i];
    else if (i == inputLen) l2[i] = '_';
    else l2[i] = '-';
  }
  l2[6] = ' '; l2[7] = ' ';
  snprintf(l2 + 8, 9, "*=Back  ");
  lcdPrint(l1, l2);
}

void buildViewItemsList() {
  viewItemCount = 0;
  for (int i = 0; i < numEquipment; i++) {
    if (equipment[i].active && equipment[i].heldBy == activeStudent)
      viewItems[viewItemCount++] = i;
  }
}

void renderViewItems() {
  if (viewItemCount == 0) {
    lcdPrint("No items held", "A:Menu #:Refresh");
    return;
  }
  int idx = viewItems[viewItemIndex % viewItemCount];
  char l1[17], l2[17];
  snprintf(l1, sizeof(l1), "%d/%d %-10s", viewItemIndex + 1, viewItemCount,
           equipShortName(idx).c_str());
  unsigned long min = (millis() - equipment[idx].borrowTime) / 60000;
  snprintf(l2, sizeof(l2), "%s %lum ago", equipment[idx].serialStr, min);
  lcdPrint(l1, l2);
}

void renderAdminMenu() {
  lcdPrint("ADMIN:", "A:Reg B:Dmp D:IP");
}

void showWiFiInfo() {
  String ip = USE_AP_MODE ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", ip.c_str());
  lcdPrint("WiFi IP:", buf);
  delay(4000);
  updateUI();
}

// ============================================================
// LED & Buzzer Control
// ============================================================
void allLedsOff() {
  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED,    LOW);
}

void ledSuccess()  { allLedsOff(); digitalWrite(LED_GREEN, HIGH); }
void ledQueued()   { allLedsOff(); digitalWrite(LED_YELLOW, HIGH); }
void ledDeadlock() { allLedsOff(); digitalWrite(LED_RED, HIGH); }

void beepSuccess() {
  digitalWrite(BUZZER_PIN, HIGH); delay(BUZZER_BEEP_MS);
  digitalWrite(BUZZER_PIN, LOW);  delay(50);
  digitalWrite(BUZZER_PIN, HIGH); delay(BUZZER_BEEP_MS);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepQueued() {
  digitalWrite(BUZZER_PIN, HIGH); delay(BUZZER_BEEP_MS);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepError() {
  digitalWrite(BUZZER_PIN, HIGH); delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepDeadlockAlarm() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(BUZZER_ALARM_MS);
    digitalWrite(BUZZER_PIN, LOW);  delay(150);
  }
}

// ============================================================
// Flash Storage (NVS)
// ============================================================
void saveStudentsToFlash() {
  prefs.putInt("numStudents", numStudents);
  for (int i = 0; i < numStudents; i++) {
    char k[20];
    snprintf(k, sizeof(k), "s%d_uid", i);  prefs.putString(k, students[i].uid);
    snprintf(k, sizeof(k), "s%d_name", i); prefs.putString(k, students[i].name);
    snprintf(k, sizeof(k), "s%d_sid", i);  prefs.putString(k, students[i].studentId);
  }
}

void loadStudentsFromFlash() {
  numStudents = prefs.getInt("numStudents", 0);
  for (int i = 0; i < numStudents && i < MAX_STUDENTS; i++) {
    students[i].active = true;
    char k[20];
    snprintf(k, sizeof(k), "s%d_uid", i);
    strncpy(students[i].uid, prefs.getString(k, "").c_str(), sizeof(students[i].uid) - 1);
    snprintf(k, sizeof(k), "s%d_name", i);
    strncpy(students[i].name, prefs.getString(k, "").c_str(), sizeof(students[i].name) - 1);
    snprintf(k, sizeof(k), "s%d_sid", i);
    strncpy(students[i].studentId, prefs.getString(k, "").c_str(), sizeof(students[i].studentId) - 1);
  }
}

void saveEquipmentToFlash() {
  prefs.putInt("numEquip", numEquipment);
  for (int i = 0; i < numEquipment; i++) {
    char k[20];
    snprintf(k, sizeof(k), "e%d_serial", i);
    prefs.putInt(k, equipment[i].serial);
  }
}

void loadEquipmentFromFlash() {
  numEquipment = prefs.getInt("numEquip", 0);
  for (int i = 0; i < numEquipment && i < MAX_EQUIPMENT; i++) {
    char k[20];
    snprintf(k, sizeof(k), "e%d_serial", i);
    int s = prefs.getInt(k, -1);
    if (s >= 0) {
      equipment[i].active    = true;
      equipment[i].serial    = s;
      equipment[i].typeCode  = s / 1000;
      equipment[i].unitNum   = s % 1000;
      equipment[i].available = true;
      equipment[i].heldBy    = -1;
      snprintf(equipment[i].serialStr, sizeof(equipment[i].serialStr), "%06d", s);
      queueInit(&equipment[i].waitQueue);
    }
  }
}

// ============================================================
// Default Data (first boot)
// ============================================================
void initDefaultStudents() {
  // Replace UIDs with your own card IDs after scanning
  struct { const char* uid; const char* name; const char* sid; } def[] = {
    {"AA:BB:CC:DD", "Roshaan", "05"},
    {"EE:FF:00:11", "Azaz",    "18"},
    {"22:33:44:55", "Shafique","03"}
  };
  int n = sizeof(def) / sizeof(def[0]);
  for (int i = 0; i < n && i < MAX_STUDENTS; i++) {
    students[i].active = true;
    strncpy(students[i].uid,       def[i].uid,  sizeof(students[i].uid) - 1);
    strncpy(students[i].name,      def[i].name, sizeof(students[i].name) - 1);
    strncpy(students[i].studentId, def[i].sid,  sizeof(students[i].studentId) - 1);
    numStudents++;
  }
  saveStudentsToFlash();
}

void initDefaultEquipment() {
  // Serial numbers: first 3 digits = type, last 3 = unit
  int ser[] = {0,1,2,3,4, 1000,1001,1002, 2000,2001, 3000,3001, 4000, 5000, 6000};
  int n = sizeof(ser) / sizeof(int);
  for (int i = 0; i < n && i < MAX_EQUIPMENT; i++) {
    equipment[i].active    = true;
    equipment[i].serial    = ser[i];
    equipment[i].typeCode  = ser[i] / 1000;
    equipment[i].unitNum   = ser[i] % 1000;
    equipment[i].available = true;
    equipment[i].heldBy    = -1;
    snprintf(equipment[i].serialStr, sizeof(equipment[i].serialStr), "%06d", ser[i]);
    queueInit(&equipment[i].waitQueue);
    numEquipment++;
  }
  saveEquipmentToFlash();
}

// ============================================================
// Admin: Register New Student
// ============================================================
void registerNewStudent(const char* uid, const char* name, const char* sid) {
  if (numStudents >= MAX_STUDENTS) {
    lcdPrint("Max students!", "Cannot add more");
    beepError(); delay(2000);
    return;
  }
  int i = numStudents++;
  students[i].active = true;
  strncpy(students[i].uid,       uid,  sizeof(students[i].uid) - 1);
  strncpy(students[i].name,      name, sizeof(students[i].name) - 1);
  strncpy(students[i].studentId, sid,  sizeof(students[i].studentId) - 1);
  saveStudentsToFlash();

  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", name);
  lcdPrint("Registered!", buf);
  beepSuccess(); ledSuccess(); delay(300); allLedsOff(); delay(2000);
}

// ============================================================
// Lookup Helpers
// ============================================================
int findStudentByUID(const char* uid) {
  for (int i = 0; i < numStudents; i++)
    if (students[i].active && strcmp(students[i].uid, uid) == 0) return i;
  return -1;
}

int findEquipmentBySerial(int serial) {
  for (int i = 0; i < numEquipment; i++)
    if (equipment[i].active && equipment[i].serial == serial) return i;
  return -1;
}

String uidToString(byte* uid, byte size) {
  String s = "";
  for (byte i = 0; i < size; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
    if (i < size - 1) s += ":";
  }
  s.toUpperCase();
  return s;
}

// Returns e.g. "Oscillo.#001" (exactly 12 chars)
String equipShortName(int eq) {
  const char* sn = getEquipTypeShortName(equipment[eq].typeCode);
  char buf[13];
  snprintf(buf, sizeof(buf), "%-7s.#%03d", sn, equipment[eq].unitNum);
  return String(buf);
}

const char* getEquipTypeName(int tc) {
  for (int i = 0; i < NUM_EQUIP_TYPES - 1; i++)
    if (EQUIP_TYPES[i].typeCode == tc) return EQUIP_TYPES[i].name;
  return EQUIP_TYPES[NUM_EQUIP_TYPES - 1].name;
}

const char* getEquipTypeShortName(int tc) {
  for (int i = 0; i < NUM_EQUIP_TYPES - 1; i++)
    if (EQUIP_TYPES[i].typeCode == tc) return EQUIP_TYPES[i].shortName;
  return EQUIP_TYPES[NUM_EQUIP_TYPES - 1].shortName;
}

// ============================================================
// Serial Debug Dump
// ============================================================
void dumpSystemState() {
  Serial.println(F("\n======== SYSTEM STATE ========"));
  Serial.printf("Students: %d/%d\n", numStudents, MAX_STUDENTS);
  Serial.printf("Equipment: %d/%d\n", numEquipment, MAX_EQUIPMENT);
  for (int i = 0; i < numStudents; i++)
    Serial.printf("[S%d] %s (%s) UID=%s\n", i, students[i].name,
                  students[i].studentId, students[i].uid);
  for (int i = 0; i < numEquipment; i++) {
    Serial.printf("[E%d] %s %-15s ", i, equipment[i].serialStr,
                  getEquipTypeName(equipment[i].typeCode));
    if (equipment[i].available) Serial.print("FREE");
    else Serial.printf("HELD by %s", students[equipment[i].heldBy].name);
    if (equipment[i].waitQueue.count > 0) {
      Serial.printf(" Q:%d ", equipment[i].waitQueue.count);
      for (int j = 0; j < equipment[i].waitQueue.count; j++)
        Serial.printf("%s ", students[equipment[i].waitQueue.data[(equipment[i].waitQueue.front + j) % QUEUE_SIZE]].name);
    }
    Serial.println();
  }
  String ci;
  if (checkDeadlock(ci)) Serial.println("\n*** DEADLOCK: " + ci);
  else Serial.println("\nNo deadlock");
  Serial.println("==============================\n");
}

// ============================================================
// Web Server Endpoints
// ============================================================
void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", buildHTMLPage());
  });
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildJSONData());
  });
  server.on("/export.csv", HTTP_GET, [](AsyncWebServerRequest* req) {
    AsyncWebServerResponse* resp = req->beginResponse(200, "text/csv", buildCSV());
    resp->addHeader("Content-Disposition", "attachment; filename=lab_export.csv");
    req->send(resp);
  });
  server.on("/graph", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/plain", buildGraphText());
  });
}

String buildJSONData() {
  String j = "{\"equipment\":[";
  for (int i = 0; i < numEquipment; i++) {
    if (i > 0) j += ",";
    j += "{\"serial\":\"" + String(equipment[i].serialStr) + "\"";
    j += ",\"type\":\""   + String(getEquipTypeName(equipment[i].typeCode)) + "\"";
    j += ",\"available\":" + String(equipment[i].available ? "true" : "false");
    if (!equipment[i].available && equipment[i].heldBy >= 0) {
      j += ",\"heldBy\":\""  + String(students[equipment[i].heldBy].name) + "\"";
      j += ",\"heldById\":\"" + String(students[equipment[i].heldBy].studentId) + "\"";
    }
    j += ",\"queueCount\":" + String(equipment[i].waitQueue.count) + "}";
  }
  j += "],\"students\":[";
  for (int i = 0; i < numStudents; i++) {
    if (i > 0) j += ",";
    j += "{\"name\":\"" + String(students[i].name) + "\",\"id\":\"" +
         String(students[i].studentId) + "\"}";
  }
  j += "]}";
  return j;
}

String buildCSV() {
  String csv = "Serial,Type,Status,HeldBy,StudentID,QueueLength,QueuedStudents\r\n";
  for (int i = 0; i < numEquipment; i++) {
    csv += String(equipment[i].serialStr) + ",";
    csv += String(getEquipTypeName(equipment[i].typeCode)) + ",";
    csv += String(equipment[i].available ? "Available" : "Borrowed") + ",";
    if (!equipment[i].available && equipment[i].heldBy >= 0) {
      csv += String(students[equipment[i].heldBy].name) + ",";
      csv += String(students[equipment[i].heldBy].studentId) + ",";
    } else {
      csv += ",,";
    }
    csv += String(equipment[i].waitQueue.count) + ",";
    for (int j = 0; j < equipment[i].waitQueue.count; j++) {
      if (j > 0) csv += ";";
      csv += students[equipment[i].waitQueue.data[(equipment[i].waitQueue.front + j) % QUEUE_SIZE]].name;
    }
    csv += "\r\n";
  }
  return csv;
}

String buildGraphText() {
  String out = "RESOURCE ALLOCATION GRAPH\n\n";
  for (int i = 0; i < MAX_NODES; i++) {
    for (int j = 0; j < MAX_NODES; j++) {
      if (graph[i][j]) {
        out += (i < MAX_STUDENTS
                  ? "Student:" + String(students[i].name)
                  : "Equip:"   + String(equipment[i - MAX_STUDENTS].serialStr));
        out += " --> ";
        out += (j < MAX_STUDENTS
                  ? "Student:" + String(students[j].name)
                  : "Equip:"   + String(equipment[j - MAX_STUDENTS].serialStr));
        out += "\n";
      }
    }
  }
  String ci;
  if (checkDeadlock(ci)) out += "\nDEADLOCK: " + ci + "\n";
  else out += "\nNo deadlock.\n";
  return out;
}

String buildHTMLPage() {
  String html = R"(<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lab Resources</title>
<style>
body{font-family:Arial;background:#1a1a2e;color:#eee;padding:16px}
h1{color:#00d4ff;text-align:center}
.card{background:#16213e;border-radius:8px;padding:14px;margin-bottom:10px;border-left:4px solid #333}
.free{border-left-color:#0f0} .held{border-left-color:#f60} .queue{border-left-color:#fd0}
.serial{font-size:22px;font-weight:bold;color:#0cf} .type{font-size:12px;color:#aaa}
.holder{font-size:13px} .queue-info{font-size:12px;color:#fd0}
.deadlock{background:red;color:#fff;padding:12px;text-align:center;font-weight:bold;margin-bottom:16px}
.links a{color:#0cf;margin:0 10px}
</style></head><body>
<h1>&#128300; Lab Resource System</h1>
<div class="links" style="text-align:center">
<a href="/export.csv">CSV</a> | <a href="/graph">Graph</a>
</div>)";

  String ci;
  if (checkDeadlock(ci))
    html += "<div class='deadlock'>&#9888; DEADLOCK: " + ci + "</div>";

  for (int i = 0; i < numEquipment; i++) {
    String cls = equipment[i].available ? "free" : "held";
    if (equipment[i].waitQueue.count > 0) cls = "queue";
    html += "<div class='card " + cls + "'>";
    html += "<div class='serial'>" + String(equipment[i].serialStr) + "</div>";
    html += "<div class='type'>" + String(getEquipTypeName(equipment[i].typeCode)) +
            " #" + String(equipment[i].unitNum) + "</div>";
    if (equipment[i].available) {
      html += "<div>&#10003; Available</div>";
    } else {
      html += "<div>&#10007; Borrowed</div>";
      if (equipment[i].heldBy >= 0)
        html += "<div class='holder'>By: " + String(students[equipment[i].heldBy].name) +
                " (" + students[equipment[i].heldBy].studentId + ")</div>";
    }
    if (equipment[i].waitQueue.count > 0) {
      html += "<div class='queue-info'>Queue: ";
      for (int j = 0; j < equipment[i].waitQueue.count; j++) {
        if (j > 0) html += " &#8594; ";
        html += students[equipment[i].waitQueue.data[(equipment[i].waitQueue.front + j) % QUEUE_SIZE]].name;
      }
      html += "</div>";
    }
    html += "</div>";
  }
  html += "<p style='text-align:center;color:#555'>DSA Project &mdash; ESP32</p>";
  html += "</body></html>";
  return html;
}
