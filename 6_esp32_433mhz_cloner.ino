#include <RCSwitch.h>
#include <RadioHead.h>
#include <BluetoothSerial.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

// Pin definitions
#define RF_RX_PIN 21  // RF receiver data pin
#define RF_TX_PIN 22  // RF transmitter data pin
#define ANALOG_PIN 36 // For signal strength measurement
#define LED_PIN 2     // Onboard LED

// Constants
#define MAX_SIGNALS 100
#define MAX_NAME_LENGTH 32
#define MAX_RAW_TIMINGS 512
#define SIGNAL_TIMEOUT 1000000  // 1 second in microseconds
#define EEPROM_SIZE 4096
#define MAX_SIGNAL_LIBRARY 50
#define JSON_BUFFER_SIZE 2048

// Frequency bands (in MHz)
const float FREQ_BANDS[] = {300.0, 315.0, 433.92, 868.0, 915.0};
const int NUM_FREQ_BANDS = 5;

// Protocol definitions
struct Protocol {
  const char* name;
  uint16_t pulseLength;
  uint8_t syncFactor;
  uint8_t tolerance;
  bool invertedSignal;
};

const Protocol PROTOCOLS[] = {
  {"PT2262", 350, 31, 60, false},
  {"EV1527", 500, 24, 60, false},
  {"HT6P20B", 400, 32, 60, false},
  {"SC2262", 360, 31, 60, false},
  {"HT12E", 460, 36, 60, false},
  {"SMC5326", 330, 31, 60, true},
  {"RT1527", 450, 24, 60, false},
  {"Nice-Flor-S", 380, 31, 60, true}
};

const int NUM_PROTOCOLS = sizeof(PROTOCOLS) / sizeof(Protocol);

// Signal structure
struct Signal {
  char name[MAX_NAME_LENGTH];
  unsigned long code;
  uint16_t protocol;
  uint16_t bitLength;
  uint16_t pulseLength;
  float frequency;
  uint32_t timestamp;
  bool isRaw;
  uint16_t rawTimings[MAX_RAW_TIMINGS];
  uint16_t rawTimingsLength;
};

// Global variables
RCSwitch mySwitch = RCSwitch();
BluetoothSerial SerialBT;
Signal signalLibrary[MAX_SIGNAL_LIBRARY];
int signalCount = 0;
float currentFrequency = 433.92;
bool isCapturing = false;
bool isAnalyzing = false;

// Function declarations
void setupDevice();
void handleCommand(char cmd);
void captureSignal();
void replaySignal(Signal& signal);
void analyzeProtocol(Signal& signal);
void saveSignal(Signal& signal);
void listSignals();
void deleteSignal(int index);
void craftCustomSignal();
float measureSignalStrength();
void printSignalInfo(Signal& signal);
void saveToEEPROM();
void loadFromEEPROM();
void exportSignal(int index);

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give serial time to start
  
  // Initialize pins with debug output
  Serial.println("\nInitializing ESP32 RF Cloner...");
  
  pinMode(RF_RX_PIN, INPUT);
  Serial.printf("RF RX Pin %d set to INPUT\n", RF_RX_PIN);
  
  pinMode(RF_TX_PIN, OUTPUT);
  Serial.printf("RF TX Pin %d set to OUTPUT\n", RF_TX_PIN);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Test LED at startup
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.printf("LED Pin %d initialized and tested\n", LED_PIN);
  
  pinMode(ANALOG_PIN, INPUT);
  Serial.printf("Analog Pin %d set to INPUT\n", ANALOG_PIN);
  
  // Initialize RF receiver with debug output
  mySwitch.enableReceive(RF_RX_PIN);
  mySwitch.enableTransmit(RF_TX_PIN);
  
  // Set default protocol with debug output
  mySwitch.setProtocol(1);
  mySwitch.setPulseLength(350);
  Serial.println("RF Switch initialized with Protocol 1");
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("EEPROM initialized");
  
  loadFromEEPROM();
  Serial.printf("Loaded %d signals from EEPROM\n", signalCount);
  
  // Initialize Bluetooth
  SerialBT.begin("ESP32_RF_Flipper");
  Serial.println("Bluetooth initialized");
  
  Serial.println("\n=== ESP32 RF Cloner Ready ===");
  printMenu();
}

void loop() {
  if (Serial.available()) {
    handleCommand(Serial.read());
  }

  if (SerialBT.available()) {
    handleCommand(SerialBT.read());
  }
  
  // Add continuous signal monitoring debug
  if (isCapturing) {
    static unsigned long lastDebugTime = 0;
    unsigned long currentTime = millis();
    
    // Print debug info every second
    if (currentTime - lastDebugTime > 1000) {
      Serial.printf("Monitoring for signals... Signal strength: %.2f\n", measureSignalStrength());
      lastDebugTime = currentTime;
    }
    
  if (mySwitch.available()) {
      Serial.println("Signal detected!");
      captureSignal();
    }
  }
  
  if (isAnalyzing) {
    analyzeSignals();
  }
}

void handleCommand(char cmd) {
  int choice;  // Moved variable declaration outside switch
  
  switch(cmd) {
    case 'C': // Start Capture
      isCapturing = true;
      Serial.println("Signal capture started...");
      break;
      
    case 'S': // Stop Capture
      isCapturing = false;
      Serial.println("Signal capture stopped");
      break;
      
    case 'L': // List saved signals
      listSignals();
      break;
      
    case 'R': // Replay signal
      if (Serial.available()) {
        int index = Serial.parseInt();
        if (index >= 0 && index < signalCount) {
          replaySignal(signalLibrary[index]);
        }
      }
      break;
      
    case 'D': // Delete signal
      if (Serial.available()) {
        int index = Serial.parseInt();
        if (index >= 0 && index < signalCount) {
          deleteSignal(index);
        }
      }
      break;
      
    case 'X': // Clear all signals
      signalCount = 0;
      saveToEEPROM();
      Serial.println("All signals cleared");
      break;
      
    case 'T': // Test LED
      Serial.println("Testing LED...");
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
      Serial.println("LED test complete");
      break;
      
    case 'A': // Analyze protocols
      isAnalyzing = !isAnalyzing;
      Serial.println(isAnalyzing ? "Protocol analysis started" : "Protocol analysis stopped");
      break;
      
    case 'F': // Change frequency
      Serial.println("\nAvailable Frequencies:");
      for (int i = 0; i < NUM_FREQ_BANDS; i++) {
        Serial.printf("%d: %.2f MHz\n", i + 1, FREQ_BANDS[i]);
      }
      while (!Serial.available()) delay(10);
      choice = Serial.read() - '0';
      if (choice > 0 && choice <= NUM_FREQ_BANDS) {
        currentFrequency = FREQ_BANDS[choice - 1];
        Serial.printf("Frequency set to: %.2f MHz\n", currentFrequency);
      }
      break;

    case 'E': // Export signal
      if (Serial.available()) {
        int index = Serial.parseInt();
        exportSignal(index);
      }
      break;

    case 'K': // Craft custom signal
      craftCustomSignal();
      // Clear any remaining input
      while (Serial.available()) {
        Serial.read();
      }
      break;

    case 'H': // Help menu
      printMenu();
      break;
  }
}

void captureSignal() {
  unsigned long code = mySwitch.getReceivedValue();
  if (code == 0) {
    Serial.println("Received signal but couldn't decode it");
      return;
    }
    
  // Debug output for received signal
  Serial.println("\nSignal Details:");
  Serial.printf("Raw Code: 0x%lX\n", code);
  Serial.printf("Protocol: %d\n", mySwitch.getReceivedProtocol());
  Serial.printf("Bit Length: %d\n", mySwitch.getReceivedBitlength());
  Serial.printf("Pulse Length: %d\n", mySwitch.getReceivedDelay());
  Serial.printf("Signal Strength: %.2f\n", measureSignalStrength());
  
  // Create JSON document for the signal
  StaticJsonDocument<JSON_BUFFER_SIZE> signalDoc;
  
  // Basic signal info
  signalDoc["name"] = String("Signal_") + String(signalCount + 1);
  signalDoc["code"] = code;
  signalDoc["protocol"] = mySwitch.getReceivedProtocol();
  signalDoc["bitLength"] = mySwitch.getReceivedBitlength();
  signalDoc["pulseLength"] = mySwitch.getReceivedDelay();
  signalDoc["frequency"] = currentFrequency;
  signalDoc["timestamp"] = millis();
  
  // Test LED multiple times to ensure visibility
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  // Save signal to library
  if (signalCount < MAX_SIGNAL_LIBRARY) {
    Signal newSignal;
    strncpy(newSignal.name, signalDoc["name"].as<const char*>(), MAX_NAME_LENGTH - 1);
    newSignal.code = code;
    newSignal.protocol = mySwitch.getReceivedProtocol();
    newSignal.bitLength = mySwitch.getReceivedBitlength();
    newSignal.pulseLength = mySwitch.getReceivedDelay();
    newSignal.frequency = currentFrequency;
    newSignal.timestamp = millis();
    
    if (mySwitch.getReceivedRawdata()) {
      newSignal.isRaw = true;
      int rawLength = min((int)mySwitch.getReceivedRawdata()[0], MAX_RAW_TIMINGS);
      memcpy(newSignal.rawTimings, mySwitch.getReceivedRawdata() + 1, rawLength * sizeof(uint16_t));
      newSignal.rawTimingsLength = rawLength;
      
      // Debug output for raw timing data
      Serial.println("\nRaw Timing Data Captured:");
      Serial.print("Length: "); Serial.println(rawLength);
      Serial.print("Timings: ");
      for (int i = 0; i < rawLength; i++) {
        Serial.print(newSignal.rawTimings[i]);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      newSignal.isRaw = false;
      Serial.println("No raw timing data available");
    }
    
    signalLibrary[signalCount++] = newSignal;
    saveToEEPROM();
    
    // Print captured signal details
    String jsonOutput;
    serializeJsonPretty(signalDoc, jsonOutput);
    Serial.println("\nCaptured and saved signal:");
    Serial.println(jsonOutput);
  } else {
    Serial.println("Signal library full!");
  }
  
  mySwitch.resetAvailable();
}

void replaySignal(Signal& signal) {
  if (signal.isRaw && signal.rawTimingsLength > 0) {
    // For raw signals, we'll need to reconstruct the signal using the pulse lengths
    digitalWrite(LED_PIN, HIGH);
    // Send the raw timings as a series of on/off pulses
    for (int i = 0; i < signal.rawTimingsLength; i += 2) {
      if (i + 1 < signal.rawTimingsLength) {
        digitalWrite(RF_TX_PIN, HIGH);
        delayMicroseconds(signal.rawTimings[i]);
        digitalWrite(RF_TX_PIN, LOW);
        delayMicroseconds(signal.rawTimings[i + 1]);
      }
    }
    digitalWrite(LED_PIN, LOW);
  } else {
    // Replay decoded signal
    mySwitch.setProtocol(signal.protocol);
    mySwitch.setPulseLength(signal.pulseLength);
    digitalWrite(LED_PIN, HIGH);
    mySwitch.send(signal.code, signal.bitLength);
    digitalWrite(LED_PIN, LOW);
  }
  Serial.printf("Replayed signal: %s\n", signal.name);
}

void analyzeSignals() {
  if (!mySwitch.available()) return;
  
  unsigned long code = mySwitch.getReceivedValue();
  if (code == 0) {
    Serial.println("Unknown encoding");
    return;
  }
  
  StaticJsonDocument<JSON_BUFFER_SIZE> analysisDoc;
  analysisDoc["rawCode"] = code;
  analysisDoc["protocol"] = mySwitch.getReceivedProtocol();
  analysisDoc["bitLength"] = mySwitch.getReceivedBitlength();
  analysisDoc["pulseLength"] = mySwitch.getReceivedDelay();
  analysisDoc["signalStrength"] = measureSignalStrength();
  
  JsonArray matches = analysisDoc.createNestedArray("protocolMatches");
  for (int i = 0; i < NUM_PROTOCOLS; i++) {
    int pulseDiff = (int)PROTOCOLS[i].pulseLength - (int)mySwitch.getReceivedDelay();
    if (abs(pulseDiff) < (int)PROTOCOLS[i].tolerance) {
      JsonObject match = matches.createNestedObject();
      match["name"] = PROTOCOLS[i].name;
      match["pulseLength"] = PROTOCOLS[i].pulseLength;
      match["pulseDiff"] = pulseDiff;
    }
  }
  
  String jsonOutput;
  serializeJsonPretty(analysisDoc, jsonOutput);
  Serial.println("\n=== Signal Analysis ===");
  Serial.println(jsonOutput);
  
  mySwitch.resetAvailable();
}

void printMenu() {
  Serial.println("\nCommands:");
  Serial.println("C: Start Signal Capture");
  Serial.println("S: Stop Signal Capture");
  Serial.println("L: List Saved Signals");
  Serial.println("R<index>: Replay Signal");
  Serial.println("D<index>: Delete Signal");
  Serial.println("E<index>: Export Signal");
  Serial.println("T: Test LED");
  Serial.println("A: Toggle Protocol Analysis");
  Serial.println("X: Clear All Signals");
  Serial.println("F: Change Frequency");
  Serial.println("K: Craft Custom Signal");
  Serial.println("H: Show This Menu");
}

void saveToEEPROM() {
  EEPROM.put(0, signalCount);
  for (int i = 0; i < signalCount; i++) {
    EEPROM.put(sizeof(int) + i * sizeof(Signal), signalLibrary[i]);
  }
  EEPROM.commit();
}

void loadFromEEPROM() {
  EEPROM.get(0, signalCount);
  if (signalCount > MAX_SIGNAL_LIBRARY) signalCount = 0;
  for (int i = 0; i < signalCount; i++) {
    EEPROM.get(sizeof(int) + i * sizeof(Signal), signalLibrary[i]);
  }
}

void printSignalInfo(Signal& signal) {
  Serial.printf("\nSignal: %s\n", signal.name);
  Serial.printf("Code: 0x%lX\n", signal.code);
  Serial.printf("Protocol: %d\n", signal.protocol);
  Serial.printf("Frequency: %.2f MHz\n", signal.frequency);
  if (signal.isRaw) {
    Serial.printf("Raw Timings: %d samples\n", signal.rawTimingsLength);
  }
}

float measureSignalStrength() {
  long total = 0;
  int samples = 64;
  
  for (int i = 0; i < samples; i++) {
    int reading = analogRead(ANALOG_PIN);
    total += reading;
    delayMicroseconds(100);
  }
  
  float average = (float)total / samples;
  float voltage = (average * 3.3) / 4095.0;  // Convert to voltage
  return voltage;
}

void craftCustomSignal() {
  Serial.println("\nEnter signal details:");
  Serial.println("Format: <protocol>,<code>,<bits>");
  Serial.println("Example: 1,FF00FF00,24");
  
  // Wait for input
  while (!Serial.available()) {
    delay(10);
  }
  
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  // Parse the input
  int firstComma = input.indexOf(',');
  int secondComma = input.indexOf(',', firstComma + 1);
  
  if (firstComma == -1 || secondComma == -1) {
    Serial.println("Invalid format. Use: <protocol>,<code>,<bits>");
    return;
  }
  
  int protocol = input.substring(0, firstComma).toInt();
  String codeStr = input.substring(firstComma + 1, secondComma);
  int bits = input.substring(secondComma + 1).toInt();
  
  // Convert hex string to unsigned long
  unsigned long code = strtoul(codeStr.c_str(), NULL, 16);
  
  Serial.printf("\nCrafting signal with:\n");
  Serial.printf("Protocol: %d\n", protocol);
  Serial.printf("Code: 0x%lX\n", code);
  Serial.printf("Bits: %d\n", bits);
  
  if (protocol > 0 && protocol <= NUM_PROTOCOLS) {
    mySwitch.setProtocol(protocol);
    mySwitch.setPulseLength(PROTOCOLS[protocol-1].pulseLength);
    digitalWrite(LED_PIN, HIGH);
    mySwitch.send(code, bits);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Custom signal sent successfully!");
  } else {
    Serial.println("Invalid protocol number. Use 1-8");
  }
}

void listSignals() {
  Serial.println("\nSaved Signals:");
  
  StaticJsonDocument<JSON_BUFFER_SIZE> listDoc;
  JsonArray signals = listDoc.createNestedArray("signals");
  
  for (int i = 0; i < signalCount; i++) {
    JsonObject signal = signals.createNestedObject();
    signal["index"] = i;
    signal["name"] = signalLibrary[i].name;
    signal["code"] = signalLibrary[i].code;
    signal["protocol"] = signalLibrary[i].protocol;
    signal["frequency"] = signalLibrary[i].frequency;
    signal["bitLength"] = signalLibrary[i].bitLength;
    signal["hasRawData"] = signalLibrary[i].isRaw;
  }
  
  String jsonOutput;
  serializeJsonPretty(listDoc, jsonOutput);
  Serial.println(jsonOutput);
}

void deleteSignal(int index) {
  if (index >= 0 && index < signalCount) {
    // Shift signals to remove the signal at the specified index
    for (int i = index; i < signalCount - 1; i++) {
      signalLibrary[i] = signalLibrary[i + 1];
    }
    signalCount--;
    saveToEEPROM();
    Serial.println("Signal deleted");
  }
}

void exportSignal(int index) {
  if (index >= 0 && index < signalCount) {
    StaticJsonDocument<JSON_BUFFER_SIZE> exportDoc;
    Signal& signal = signalLibrary[index];
    
    exportDoc["name"] = signal.name;
    exportDoc["code"] = signal.code;
    exportDoc["protocol"] = signal.protocol;
    exportDoc["bitLength"] = signal.bitLength;
    exportDoc["pulseLength"] = signal.pulseLength;
    exportDoc["frequency"] = signal.frequency;
    exportDoc["timestamp"] = signal.timestamp;
    
    if (signal.isRaw) {
      // Debug output for raw timing data
      Serial.println("\nRaw Timing Data Being Exported:");
      Serial.print("Length: "); Serial.println(signal.rawTimingsLength);
      Serial.print("Timings: ");
      for (int i = 0; i < signal.rawTimingsLength; i++) {
        Serial.print(signal.rawTimings[i]);
        Serial.print(" ");
      }
      Serial.println();
      
      JsonArray rawTimings = exportDoc.createNestedArray("rawTimings");
      for (int i = 0; i < signal.rawTimingsLength; i++) {
        rawTimings.add(signal.rawTimings[i]);
      }
    }
    
    String jsonOutput;
    serializeJson(exportDoc, jsonOutput);
    Serial.println(jsonOutput);
  }
} 