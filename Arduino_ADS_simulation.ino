/*
  TBI Arduino Due Electrode Interrogation and Communication Firmware
  ------------------------------------------------

  Architecture:
  - Arduino generates stepped CV waveform.
  - Arduino outputs corresponding DAC voltage on DAC0.
       V_DAC = V_REF - E_CV
    where V_REF = 1.25 V by default.
  - Arduino reads/simulates ADS124S08 channels:
      AMP_GLU, AMP_LAC, AMP_PYR, POT_K
  - Arduino reads/simulates ADS1299 channels:
      ECOG1-ECOG6, EEG1-EEG2
  - Arduino sends one combined data frame to MATLAB.
  - MATLAB logs data.
  - MATLAB derives calibration information

  CV waveform:
  - Desired electrode potential: -0.4 V to +0.6 V
  - Step size: 5 mV
  - Scan rate: 50 mV/s

  Commands:
  PING
  STATUS
  SET_MODE SIM
  SET_MODE ADS
  SET_RATE <Hz>
  SET_CV <start_V> <vertex_V> <step_V> <scan_rate_V_per_s>
  START_INT
  STOP

  Currently implemented:
  SET_MODE SIM
  SET_RATE 100
  SET_CV -0.4 0.6 0.005 0.05
  START_INT
*/

#include <SPI.h>

// =====================================================
// Serial and board configuration
// =====================================================

#define BAUD_RATE 115200

#define DAC_RESOLUTION_BITS 12
#define DAC_MAX_CODE 4095

// Arduino Due DAC approximate output range.
#define DAC_MIN_VOLTAGE 0.55
#define DAC_MAX_VOLTAGE 2.75

// SPI chip-select pins.
#define CS_ADS124S08 10
#define CS_ADS1299   9

// =====================================================
// Channel configuration
// =====================================================

const int NUM_ADS124_CHANNELS = 4;
const int NUM_ADS1299_CHANNELS = 8;

const char* ads124Names[NUM_ADS124_CHANNELS] = {
  "AMP_GLU",
  "AMP_LAC",
  "AMP_PYR",
  "POT_K"
};

const char* ads1299Names[NUM_ADS1299_CHANNELS] = {
  "ECOG1",
  "ECOG2",
  "ECOG3",
  "ECOG4",
  "ECOG5",
  "ECOG6",
  "EEG1",
  "EEG2"
};

// =====================================================
// Acquisition mode
// =====================================================

enum AcquisitionMode {
  MODE_SIM,
  MODE_ADS
};

AcquisitionMode acquisitionMode = MODE_SIM;

bool interrogationRunning = false;

unsigned long sampleIntervalMicros = 10000; // default 100 Hz
unsigned long lastSampleMicros = 0;
unsigned long sampleCounter = 0;

unsigned long interrogationStartMicros = 0;

// =====================================================
// CV waveform parameters
// =====================================================

float cvStartPotential = -0.4;
float cvVertexPotential = 0.6;

float cvStepSize = 0.005;    // 5 mV
float cvScanRate = 0.05;     // 50 mV/s

unsigned long cvStepIntervalMicros = 100000; // 0.1 s

// Inverting DAC relationship:
// V_DAC = dacReferenceVoltage - E_CV
float dacReferenceVoltage = 1.25;

float currentCVPotential = -0.4;
int cvDirection = 1; // +1 forward, -1 reverse
unsigned long lastCVStepMicros = 0;

// =====================================================
// Analogue conversion parameters
// =====================================================

// TIA conversion:
// Vout = Vref - Iin * Rf
// Iin = (Vref - Vout) / Rf
float tiaReferenceVoltage = 1.25;

float Rf_GLU = 100e6;
float Rf_LAC = 100e6;
float Rf_PYR = 100e6;

// ADS124S08 configuration.
float ads124Vref = 2.5;
float ads124Gain = 1.0;

// ADS1299 configuration.
float ads1299Vref = 2.5;
float ads1299Gain = 1.0;

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);

  analogWriteResolution(DAC_RESOLUTION_BITS);

  pinMode(CS_ADS124S08, OUTPUT);
  pinMode(CS_ADS1299, OUTPUT);

  digitalWrite(CS_ADS124S08, HIGH);
  digitalWrite(CS_ADS1299, HIGH);

  SPI.begin();

  currentCVPotential = cvStartPotential;
  float initialDACVoltage = cvPotentialToDACVoltage(currentCVPotential);
  writeDACVoltage(initialDACVoltage);

  Serial.println("READY,TBI_COMBINED_INTERROGATION_FIRMWARE");
  Serial.println("INFO,Commands: PING, STATUS, SET_MODE SIM/ADS, SET_RATE <Hz>, SET_CV <start_V> <vertex_V> <step_V> <scan_rate_V_per_s>, START_INT, STOP");
}

// =====================================================
// Main loop
// =====================================================

void loop() {
  handleSerialCommands();

  if (interrogationRunning) {
    unsigned long now = micros();

    updateSteppedCV(now);

    if (now - lastSampleMicros >= sampleIntervalMicros) {
      lastSampleMicros = now;

      float dacVoltage = cvPotentialToDACVoltage(currentCVPotential);
      int dacCode = writeDACVoltage(dacVoltage);

      readAndStreamFrame(now, currentCVPotential, dacVoltage, dacCode);

      sampleCounter++;
    }
  }
}

// =====================================================
// Command handling
// =====================================================

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      processCommand(command);
    }
  }
}

void processCommand(String command) {
  if (command == "PING") {
    Serial.println("ACK,PING");
  }

  else if (command == "STATUS") {
    printStatus();
  }

  else if (command.startsWith("SET_MODE")) {
    setMode(command);
  }

  else if (command.startsWith("SET_RATE")) {
    setSamplingRate(command);
  }

  else if (command.startsWith("SET_CV")) {
    setCVParameters(command);
  }

  else if (command == "START_INT") {
    startInterrogation();
  }

  else if (command == "STOP") {
    stopInterrogation();
  }

  else {
    Serial.print("ERR,UNKNOWN_COMMAND,");
    Serial.println(command);
  }
}

void printStatus() {
  Serial.print("STATUS,");
  Serial.print("running=");
  Serial.print(interrogationRunning ? "1" : "0");

  Serial.print(",mode=");
  Serial.print(acquisitionMode == MODE_SIM ? "SIM" : "ADS");

  Serial.print(",fs=");
  Serial.print(1000000.0 / sampleIntervalMicros);

  Serial.print(",cv_start=");
  Serial.print(cvStartPotential, 4);

  Serial.print(",cv_vertex=");
  Serial.print(cvVertexPotential, 4);

  Serial.print(",cv_step=");
  Serial.print(cvStepSize, 4);

  Serial.print(",scan_rate=");
  Serial.print(cvScanRate, 4);

  Serial.print(",step_interval_us=");
  Serial.print(cvStepIntervalMicros);

  Serial.print(",current_cv=");
  Serial.print(currentCVPotential, 4);

  Serial.print(",samples=");
  Serial.println(sampleCounter);
}

void setMode(String command) {
  if (command.endsWith("SIM")) {
    acquisitionMode = MODE_SIM;
    Serial.println("ACK,SET_MODE,SIM");
  }

  else if (command.endsWith("ADS")) {
    acquisitionMode = MODE_ADS;
    Serial.println("ACK,SET_MODE,ADS");
  }

  else {
    Serial.println("ERR,INVALID_MODE");
  }
}

void setSamplingRate(String command) {
  int idx = command.indexOf(' ');

  if (idx < 0) {
    Serial.println("ERR,SET_RATE_REQUIRES_VALUE");
    return;
  }

  float fs = command.substring(idx + 1).toFloat();

  if (fs <= 0 || fs > 1000) {
    Serial.println("ERR,INVALID_SAMPLE_RATE");
    return;
  }

  sampleIntervalMicros = (unsigned long)(1000000.0 / fs);

  Serial.print("ACK,SET_RATE,");
  Serial.println(fs, 2);
}

void setCVParameters(String command) {
  float vals[4];
  int count = parseFourFloats(command, vals);

  if (count != 4) {
    Serial.println("ERR,SET_CV_REQUIRES_4_VALUES");
    Serial.println("INFO,Use: SET_CV <start_V> <vertex_V> <step_V> <scan_rate_V_per_s>");
    return;
  }

  cvStartPotential = vals[0];
  cvVertexPotential = vals[1];
  cvStepSize = abs(vals[2]);
  cvScanRate = abs(vals[3]);

  if (cvStepSize <= 0 || cvScanRate <= 0) {
    Serial.println("ERR,INVALID_CV_STEP_OR_SCAN_RATE");
    return;
  }

  cvStepIntervalMicros = (unsigned long)((cvStepSize / cvScanRate) * 1000000.0);

  currentCVPotential = cvStartPotential;
  cvDirection = 1;

  float dacVoltage = cvPotentialToDACVoltage(currentCVPotential);
  writeDACVoltage(dacVoltage);

  Serial.print("ACK,SET_CV,");
  Serial.print(cvStartPotential, 4);
  Serial.print(",");
  Serial.print(cvVertexPotential, 4);
  Serial.print(",");
  Serial.print(cvStepSize, 4);
  Serial.print(",");
  Serial.print(cvScanRate, 4);
  Serial.print(",");
  Serial.print("step_interval_us=");
  Serial.println(cvStepIntervalMicros);
}

int parseFourFloats(String command, float vals[4]) {
  int firstSpace = command.indexOf(' ');
  if (firstSpace < 0) return 0;

  String args = command.substring(firstSpace + 1);
  args.trim();

  int count = 0;

  while (args.length() > 0 && count < 4) {
    int spaceIndex = args.indexOf(' ');

    String token;

    if (spaceIndex >= 0) {
      token = args.substring(0, spaceIndex);
      args = args.substring(spaceIndex + 1);
      args.trim();
    } else {
      token = args;
      args = "";
    }

    vals[count] = token.toFloat();
    count++;
  }

  return count;
}

void startInterrogation() {
  sampleCounter = 0;

  interrogationStartMicros = micros();
  lastSampleMicros = interrogationStartMicros;
  lastCVStepMicros = interrogationStartMicros;

  currentCVPotential = cvStartPotential;
  cvDirection = 1;

  float dacVoltage = cvPotentialToDACVoltage(currentCVPotential);
  writeDACVoltage(dacVoltage);

  interrogationRunning = true;

  Serial.println("ACK,START_INT");

  Serial.println(
    "HEADER,sample_counter,timestamp_us,time_s,"
    "cv_potential,dac_voltage,dac_code,phase,"
    "AMP_GLU_V,AMP_LAC_V,AMP_PYR_V,POT_K_V,"
    "AMP_GLU_A,AMP_LAC_A,AMP_PYR_A,"
    "ECOG1_V,ECOG2_V,ECOG3_V,ECOG4_V,ECOG5_V,ECOG6_V,EEG1_V,EEG2_V"
  );
}

void stopInterrogation() {
  interrogationRunning = false;

  currentCVPotential = cvStartPotential;
  float dacVoltage = cvPotentialToDACVoltage(currentCVPotential);
  writeDACVoltage(dacVoltage);

  Serial.println("ACK,STOP");
}

// =====================================================
// CV waveform generation
// =====================================================

void updateSteppedCV(unsigned long nowMicros) {
  if (nowMicros - lastCVStepMicros < cvStepIntervalMicros) {
    return;
  }

  lastCVStepMicros = nowMicros;

  currentCVPotential += cvDirection * cvStepSize;

  if (cvDirection > 0 && currentCVPotential >= cvVertexPotential) {
    currentCVPotential = cvVertexPotential;
    cvDirection = -1;
  }

  else if (cvDirection < 0 && currentCVPotential <= cvStartPotential) {
    currentCVPotential = cvStartPotential;
    cvDirection = 1;
  }
}

float cvPotentialToDACVoltage(float cvPotential) {
  return dacReferenceVoltage - cvPotential;
}

float calculateCVPhase() {
  float totalRange = abs(cvVertexPotential - cvStartPotential);

  if (totalRange <= 0) {
    return 0;
  }

  if (cvDirection > 0) {
    return 0.5 * abs(currentCVPotential - cvStartPotential) / totalRange;
  } else {
    return 0.5 + 0.5 * abs(cvVertexPotential - currentCVPotential) / totalRange;
  }
}

int writeDACVoltage(float voltage) {
  float clippedVoltage = voltage;

  if (clippedVoltage < DAC_MIN_VOLTAGE) {
    clippedVoltage = DAC_MIN_VOLTAGE;
  }

  if (clippedVoltage > DAC_MAX_VOLTAGE) {
    clippedVoltage = DAC_MAX_VOLTAGE;
  }

  int dacCode = (int)((clippedVoltage - DAC_MIN_VOLTAGE) /
                      (DAC_MAX_VOLTAGE - DAC_MIN_VOLTAGE) *
                      DAC_MAX_CODE);

  if (dacCode < 0) {
    dacCode = 0;
  }

  if (dacCode > DAC_MAX_CODE) {
    dacCode = DAC_MAX_CODE;
  }

  analogWrite(DAC0, dacCode);

  return dacCode;
}

// =====================================================
// Combined data acquisition and streaming
// =====================================================

void readAndStreamFrame(unsigned long timestampMicros, float cvPotential, float dacVoltage, int dacCode) {
  float ads124Voltages[NUM_ADS124_CHANNELS];
  float ads1299Voltages[NUM_ADS1299_CHANNELS];

  if (acquisitionMode == MODE_SIM) {
    readSimulatedADS124(ads124Voltages, timestampMicros, cvPotential);
    readSimulatedADS1299(ads1299Voltages, timestampMicros);
  } else {
    readADS124S08Channels(ads124Voltages);
    readADS1299Channels(ads1299Voltages);
  }

  float I_GLU = voltageToTIAInputCurrent(ads124Voltages[0], Rf_GLU);
  float I_LAC = voltageToTIAInputCurrent(ads124Voltages[1], Rf_LAC);
  float I_PYR = voltageToTIAInputCurrent(ads124Voltages[2], Rf_PYR);

  float timeSeconds = (timestampMicros - interrogationStartMicros) / 1000000.0;
  float phase = calculateCVPhase();

  Serial.print("DATA,");
  Serial.print(sampleCounter);
  Serial.print(",");
  Serial.print(timestampMicros);
  Serial.print(",");
  Serial.print(timeSeconds, 6);
  Serial.print(",");
  Serial.print(cvPotential, 6);
  Serial.print(",");
  Serial.print(dacVoltage, 6);
  Serial.print(",");
  Serial.print(dacCode);
  Serial.print(",");
  Serial.print(phase, 6);

  for (int i = 0; i < NUM_ADS124_CHANNELS; i++) {
    Serial.print(",");
    Serial.print(ads124Voltages[i], 8);
  }

  Serial.print(",");
  Serial.print(I_GLU, 12);
  Serial.print(",");
  Serial.print(I_LAC, 12);
  Serial.print(",");
  Serial.print(I_PYR, 12);

  for (int i = 0; i < NUM_ADS1299_CHANNELS; i++) {
    Serial.print(",");
    Serial.print(ads1299Voltages[i], 10);
  }

  Serial.println();
}

float voltageToTIAInputCurrent(float tiaOutputVoltage, float feedbackResistance) {
  return (tiaReferenceVoltage - tiaOutputVoltage) / feedbackResistance;
}

// =====================================================
// Simulated ADS124S08 data
// =====================================================

void readSimulatedADS124(float channelVoltages[NUM_ADS124_CHANNELS], unsigned long timestampMicros, float cvPotential) {
  float t = (timestampMicros - interrogationStartMicros) / 1000000.0;

  float I_GLU = 1.0e-9
              + 0.5e-9 * (cvPotential - cvStartPotential)
              + 0.05e-9 * sin(2.0 * PI * 0.2 * t);

  float I_LAC = 1.5e-9
              + 0.3e-9 * (cvPotential - cvStartPotential)
              + 0.05e-9 * sin(2.0 * PI * 0.15 * t + 0.4);

  float I_PYR = 0.8e-9
              + 0.2e-9 * (cvPotential - cvStartPotential)
              + 0.03e-9 * sin(2.0 * PI * 0.1 * t + 0.7);

  channelVoltages[0] = tiaReferenceVoltage - I_GLU * Rf_GLU;
  channelVoltages[1] = tiaReferenceVoltage - I_LAC * Rf_LAC;
  channelVoltages[2] = tiaReferenceVoltage - I_PYR * Rf_PYR;

  // Potentiometric potassium channel simulation.
  channelVoltages[3] = 0.100 + 0.010 * sin(2.0 * PI * 0.05 * t);
}

// =====================================================
// Simulated ADS1299 data
// =====================================================

void readSimulatedADS1299(float neuralVoltages[NUM_ADS1299_CHANNELS], unsigned long timestampMicros) {
  float t = (timestampMicros - interrogationStartMicros) / 1000000.0;

  // Simulated ECoG channels, mV
  neuralVoltages[0] = 1.0e-3 * sin(2.0 * PI * 10.0 * t);
  neuralVoltages[1] = 0.8e-3 * sin(2.0 * PI * 12.0 * t + 0.3);
  neuralVoltages[2] = 0.7e-3 * sin(2.0 * PI * 15.0 * t + 0.6);
  neuralVoltages[3] = 0.6e-3 * sin(2.0 * PI * 18.0 * t + 0.9);
  neuralVoltages[4] = 0.5e-3 * sin(2.0 * PI * 20.0 * t + 1.2);
  neuralVoltages[5] = 0.5e-3 * sin(2.0 * PI * 22.0 * t + 1.5);

  // Simulated EEG channels, lower amplitude.
  neuralVoltages[6] = 100.0e-6 * sin(2.0 * PI * 8.0 * t);
  neuralVoltages[7] = 80.0e-6 * sin(2.0 * PI * 18.0 * t);
}

// =====================================================
// ADS124S08 SPI placeholder layer
// =====================================================

void readADS124S08Channels(float channelVoltages[NUM_ADS124_CHANNELS]) {
  for (int ch = 0; ch < NUM_ADS124_CHANNELS; ch++) {
    int32_t raw = readADS124S08SingleEnded(ch);
    channelVoltages[ch] = ads124RawToVoltage(raw);
  }
}

float ads124RawToVoltage(int32_t rawCode) {
  return ((float)rawCode / 8388607.0) * (ads124Vref / ads124Gain);
}

int32_t readADS124S08SingleEnded(int channel) {
  /*
    Hardware-specific placeholder.
    Final implementation should:
    - configure ADS124S08 input MUX
    - select positive and negative inputs
    - configure PGA/reference/data rate
    - start conversion
    - wait for DRDY or conversion delay
    - read 24-bit signed conversion result
  */

  configureADS124S08InputChannel(channel);

  delayMicroseconds(1000);

  int32_t raw = ads124s08ReadDataCommand();

  return raw;
}

void configureADS124S08InputChannel(int channel) {
  /*
    Placeholder SPI transaction.
    Replace with real ADS124S08 register writes.
  */

  digitalWrite(CS_ADS124S08, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  SPI.transfer(0x00);
  SPI.transfer(channel);

  SPI.endTransaction();
  digitalWrite(CS_ADS124S08, HIGH);
}

int32_t ads124s08ReadDataCommand() {
  /*
    Placeholder 24-bit read.
    Replace command byte with actual ADS124S08 RDATA command.
  */

  digitalWrite(CS_ADS124S08, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  SPI.transfer(0x00);

  byte b1 = SPI.transfer(0x00);
  byte b2 = SPI.transfer(0x00);
  byte b3 = SPI.transfer(0x00);

  SPI.endTransaction();
  digitalWrite(CS_ADS124S08, HIGH);

  int32_t raw = ((int32_t)b1 << 16) | ((int32_t)b2 << 8) | b3;

  if (raw & 0x800000) {
    raw |= 0xFF000000;
  }

  return raw;
}

// =====================================================
// ADS1299 SPI placeholder layer
// =====================================================

void readADS1299Channels(float neuralVoltages[NUM_ADS1299_CHANNELS]) {
  /*
    Placeholder for ADS1299 acquisition.

    Final implementation should read a full ADS1299 data frame:
    - status bytes
    - 8 x 24-bit channel values
  */

  for (int ch = 0; ch < NUM_ADS1299_CHANNELS; ch++) {
    int32_t raw = readADS1299RawChannel(ch);
    neuralVoltages[ch] = ads1299RawToVoltage(raw);
  }
}

float ads1299RawToVoltage(int32_t rawCode) {
  return ((float)rawCode / 8388607.0) * (ads1299Vref / ads1299Gain);
}

int32_t readADS1299RawChannel(int channel) {
  /*
    Placeholder only.

    Real ADS1299 reading should use continuous conversion or full-frame reads,
    not individual channel reads.
  */

  digitalWrite(CS_ADS1299, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  SPI.transfer(0x00);
  SPI.transfer(channel);

  byte b1 = SPI.transfer(0x00);
  byte b2 = SPI.transfer(0x00);
  byte b3 = SPI.transfer(0x00);

  SPI.endTransaction();
  digitalWrite(CS_ADS1299, HIGH);

  int32_t raw = ((int32_t)b1 << 16) | ((int32_t)b2 << 8) | b3;

  if (raw & 0x800000) {
    raw |= 0xFF000000;
  }

  return raw;
}
