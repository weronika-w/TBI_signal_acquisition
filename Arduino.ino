/*
  TBI Arduino Due Electrode Interrogation and Communication Firmware
  ------------------------------------------------------------------
  - Software SPI is used because PCB routes SPI to D13/D12/D11.
  - D13 = SCLK
  - D12 = DOUT from ADCs = MISO into Arduino
  - D11 = DIN to ADCs = MOSI from Arduino

  Control pins from PCB routing:
  - D10 = CS ADS124S08
  - D9  = CS ADS1299
  - D8  = ADS1299 PWDN
  - D4  = ADS1299 START
  - D3  = RESET
  - D2  = DRDY ADS1299
  - D1  = DRDY ADS124S08

  IMPORTANT:
  - Because DRDY12408 is on D1, Arduino Due Native USB port has to be used.
  - This code uses SerialUSB, not Serial.

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
  - MATLAB logs data and derives calibration information.

  Commands:
  PING
  STATUS
  TEST_PINS
  TEST_ADS
  SET_MODE SIM
  SET_MODE ADS
  SET_RATE <Hz>
  SET_CV <start_V> <vertex_V> <step_V> <scan_rate_V_per_s>
  START_INT
  STOP
*/

// =====================================================
// Serial configuration
// =====================================================

#define PC_SERIAL SerialUSB
#define BAUD_RATE 115200

// =====================================================
// DAC configuration
// =====================================================

#define DAC_RESOLUTION_BITS 12
#define DAC_MAX_CODE 4095

#define DAC_MIN_VOLTAGE 0.55
#define DAC_MAX_VOLTAGE 2.75

// =====================================================
// PCB pin definitions
// =====================================================

// Software SPI pins from PCB routing
#define SOFT_SPI_SCLK 13
#define SOFT_SPI_MISO 12   // ADC DOUT
#define SOFT_SPI_MOSI 11   // ADC DIN

// Chip selects
#define CS_ADS124S08 10
#define CS_ADS1299   9

// ADS1299 control
#define PWDN_ADS1299  8
#define START_ADS1299 4

// Shared RESET net on PCB
#define RESET_ADS 3

// Data-ready pins
#define DRDY_ADS1299   2
#define DRDY_ADS124S08 1

// Software SPI timing.
// Increase if the ADCs do not respond reliably.
#define SOFT_SPI_HALF_PERIOD_US 10

// =====================================================
// ADS124S08 command/register definitions
// =====================================================

#define ADS124_CMD_NOP       0x00
#define ADS124_CMD_WAKEUP    0x02
#define ADS124_CMD_POWERDOWN 0x04
#define ADS124_CMD_RESET     0x06
#define ADS124_CMD_START     0x08
#define ADS124_CMD_STOP      0x0A
#define ADS124_CMD_RDATA     0x12
#define ADS124_CMD_RREG      0x20
#define ADS124_CMD_WREG      0x40

#define ADS124_REG_ID        0x00
#define ADS124_REG_STATUS    0x01
#define ADS124_REG_INPMUX    0x02
#define ADS124_REG_PGA       0x03
#define ADS124_REG_DATARATE  0x04
#define ADS124_REG_REF       0x05

// =====================================================
// ADS1299 command/register definitions
// =====================================================

#define ADS1299_CMD_WAKEUP   0x02
#define ADS1299_CMD_STANDBY  0x04
#define ADS1299_CMD_RESET    0x06
#define ADS1299_CMD_START    0x08
#define ADS1299_CMD_STOP     0x0A
#define ADS1299_CMD_RDATAC   0x10
#define ADS1299_CMD_SDATAC   0x11
#define ADS1299_CMD_RDATA    0x12
#define ADS1299_CMD_RREG     0x20
#define ADS1299_CMD_WREG     0x40

#define ADS1299_REG_ID       0x00
#define ADS1299_REG_CONFIG1  0x01
#define ADS1299_REG_CONFIG2  0x02
#define ADS1299_REG_CONFIG3  0x03
#define ADS1299_REG_LOFF     0x04
#define ADS1299_REG_CH1SET   0x05

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
bool adsInitialised = false;

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

float dacReferenceVoltage = 1.25;

float currentCVPotential = -0.4;
int cvDirection = 1;
unsigned long lastCVStepMicros = 0;

// =====================================================
// Analogue conversion parameters
// =====================================================

float tiaReferenceVoltage = 1.25;

float Rf_GLU = 100e6;
float Rf_LAC = 100e6;
float Rf_PYR = 100e6;

// ADS124S08 conversion settings.
float ads124Vref = 2.5;
float ads124Gain = 1.0;

// ADS1299 conversion settings.
float ads1299Vref = 2.5;
float ads1299Gain = 1.0;

// =====================================================
// Setup
// =====================================================

void setup() {
  PC_SERIAL.begin(BAUD_RATE);

  // Wait briefly for Native USB.
  unsigned long usbWaitStart = millis();
  while (!PC_SERIAL && (millis() - usbWaitStart < 3000)) {
    ;
  }

  delay(500);

  analogWriteResolution(DAC_RESOLUTION_BITS);

  setupPins();
  softSPIBegin();

  currentCVPotential = cvStartPotential;
  float initialDACVoltage = cvPotentialToDACVoltage(currentCVPotential);
  writeDACVoltage(initialDACVoltage);

  PC_SERIAL.println("READY,TBI_INTERROGATION_SOFTWARE_SPI");
  PC_SERIAL.println("INFO,Use Arduino Due Native USB port because DRDY12408 is on D1");
  PC_SERIAL.println("INFO,Commands: PING, STATUS, TEST_PINS, TEST_ADS, SET_MODE SIM/ADS, SET_RATE <Hz>, SET_CV <start_V> <vertex_V> <step_V> <scan_rate_V_per_s>, START_INT, STOP");
}

void setupPins() {
  pinMode(CS_ADS124S08, OUTPUT);
  pinMode(CS_ADS1299, OUTPUT);

  pinMode(PWDN_ADS1299, OUTPUT);
  pinMode(START_ADS1299, OUTPUT);
  pinMode(RESET_ADS, OUTPUT);

  pinMode(DRDY_ADS1299, INPUT);
  pinMode(DRDY_ADS124S08, INPUT);

  digitalWrite(CS_ADS124S08, HIGH);
  digitalWrite(CS_ADS1299, HIGH);

  digitalWrite(PWDN_ADS1299, HIGH);
  digitalWrite(START_ADS1299, LOW);
  digitalWrite(RESET_ADS, HIGH);
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
  if (PC_SERIAL.available() > 0) {
    String command = PC_SERIAL.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      processCommand(command);
    }
  }
}

void processCommand(String command) {
  if (command == "PING") {
    PC_SERIAL.println("ACK,PING");
  }

  else if (command == "STATUS") {
    printStatus();
  }

  else if (command == "TEST_PINS") {
    testPins();
  }

  else if (command == "TEST_ADS") {
    testADSCommunication();
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
    PC_SERIAL.print("ERR,UNKNOWN_COMMAND,");
    PC_SERIAL.println(command);
  }
}

void printStatus() {
  PC_SERIAL.print("STATUS,");
  PC_SERIAL.print("running=");
  PC_SERIAL.print(interrogationRunning ? "1" : "0");

  PC_SERIAL.print(",mode=");
  PC_SERIAL.print(acquisitionMode == MODE_SIM ? "SIM" : "ADS");

  PC_SERIAL.print(",adsInitialised=");
  PC_SERIAL.print(adsInitialised ? "1" : "0");

  PC_SERIAL.print(",fs=");
  PC_SERIAL.print(1000000.0 / sampleIntervalMicros);

  PC_SERIAL.print(",cv_start=");
  PC_SERIAL.print(cvStartPotential, 4);

  PC_SERIAL.print(",cv_vertex=");
  PC_SERIAL.print(cvVertexPotential, 4);

  PC_SERIAL.print(",cv_step=");
  PC_SERIAL.print(cvStepSize, 4);

  PC_SERIAL.print(",scan_rate=");
  PC_SERIAL.print(cvScanRate, 4);

  PC_SERIAL.print(",step_interval_us=");
  PC_SERIAL.print(cvStepIntervalMicros);

  PC_SERIAL.print(",current_cv=");
  PC_SERIAL.print(currentCVPotential, 4);

  PC_SERIAL.print(",samples=");
  PC_SERIAL.println(sampleCounter);
}

void setMode(String command) {
  if (command.endsWith("SIM")) {
    acquisitionMode = MODE_SIM;
    PC_SERIAL.println("ACK,SET_MODE,SIM");
  }

  else if (command.endsWith("ADS")) {
    acquisitionMode = MODE_ADS;

    if (!adsInitialised) {
      initialiseADS();
    }

    PC_SERIAL.println("ACK,SET_MODE,ADS");
  }

  else {
    PC_SERIAL.println("ERR,INVALID_MODE");
  }
}

void setSamplingRate(String command) {
  int idx = command.indexOf(' ');

  if (idx < 0) {
    PC_SERIAL.println("ERR,SET_RATE_REQUIRES_VALUE");
    return;
  }

  float fs = command.substring(idx + 1).toFloat();

  if (fs <= 0 || fs > 1000) {
    PC_SERIAL.println("ERR,INVALID_SAMPLE_RATE");
    return;
  }

  sampleIntervalMicros = (unsigned long)(1000000.0 / fs);

  PC_SERIAL.print("ACK,SET_RATE,");
  PC_SERIAL.println(fs, 2);
}

void setCVParameters(String command) {
  float vals[4];
  int count = parseFourFloats(command, vals);

  if (count != 4) {
    PC_SERIAL.println("ERR,SET_CV_REQUIRES_4_VALUES");
    PC_SERIAL.println("INFO,Use: SET_CV <start_V> <vertex_V> <step_V> <scan_rate_V_per_s>");
    return;
  }

  cvStartPotential = vals[0];
  cvVertexPotential = vals[1];
  cvStepSize = abs(vals[2]);
  cvScanRate = abs(vals[3]);

  if (cvStepSize <= 0 || cvScanRate <= 0) {
    PC_SERIAL.println("ERR,INVALID_CV_STEP_OR_SCAN_RATE");
    return;
  }

  cvStepIntervalMicros = (unsigned long)((cvStepSize / cvScanRate) * 1000000.0);

  currentCVPotential = cvStartPotential;
  cvDirection = 1;

  float dacVoltage = cvPotentialToDACVoltage(currentCVPotential);
  writeDACVoltage(dacVoltage);

  PC_SERIAL.print("ACK,SET_CV,");
  PC_SERIAL.print(cvStartPotential, 4);
  PC_SERIAL.print(",");
  PC_SERIAL.print(cvVertexPotential, 4);
  PC_SERIAL.print(",");
  PC_SERIAL.print(cvStepSize, 4);
  PC_SERIAL.print(",");
  PC_SERIAL.print(cvScanRate, 4);
  PC_SERIAL.print(",");
  PC_SERIAL.print("step_interval_us=");
  PC_SERIAL.println(cvStepIntervalMicros);
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
  if (acquisitionMode == MODE_ADS && !adsInitialised) {
    initialiseADS();
  }

  sampleCounter = 0;

  interrogationStartMicros = micros();
  lastSampleMicros = interrogationStartMicros;
  lastCVStepMicros = interrogationStartMicros;

  currentCVPotential = cvStartPotential;
  cvDirection = 1;

  float dacVoltage = cvPotentialToDACVoltage(currentCVPotential);
  writeDACVoltage(dacVoltage);

  interrogationRunning = true;

  PC_SERIAL.println("ACK,START_INT");

  PC_SERIAL.println(
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

  PC_SERIAL.println("ACK,STOP");
}

// =====================================================
// Test commands
// =====================================================

void testPins() {
  PC_SERIAL.println("TEST,PINS");

  PC_SERIAL.print("SOFT_SPI_SCLK=");
  PC_SERIAL.println(SOFT_SPI_SCLK);

  PC_SERIAL.print("SOFT_SPI_MISO_DOUT=");
  PC_SERIAL.println(SOFT_SPI_MISO);

  PC_SERIAL.print("SOFT_SPI_MOSI_DIN=");
  PC_SERIAL.println(SOFT_SPI_MOSI);

  PC_SERIAL.print("CS_ADS124S08=");
  PC_SERIAL.println(CS_ADS124S08);

  PC_SERIAL.print("CS_ADS1299=");
  PC_SERIAL.println(CS_ADS1299);

  PC_SERIAL.print("PWDN_ADS1299=");
  PC_SERIAL.println(PWDN_ADS1299);

  PC_SERIAL.print("START_ADS1299=");
  PC_SERIAL.println(START_ADS1299);

  PC_SERIAL.print("RESET_ADS=");
  PC_SERIAL.println(RESET_ADS);

  PC_SERIAL.print("DRDY_ADS1299 pin=");
  PC_SERIAL.print(DRDY_ADS1299);
  PC_SERIAL.print(", value=");
  PC_SERIAL.println(digitalRead(DRDY_ADS1299));

  PC_SERIAL.print("DRDY_ADS124S08 pin=");
  PC_SERIAL.print(DRDY_ADS124S08);
  PC_SERIAL.print(", value=");
  PC_SERIAL.println(digitalRead(DRDY_ADS124S08));
}

void testADSCommunication() {
  if (!adsInitialised) {
    initialiseADS();
  }

  byte ads1299ID = ads1299ReadRegister(ADS1299_REG_ID);
  byte ads124ID = ads124ReadRegister(ADS124_REG_ID);
  byte ads124PGA = ads124ReadRegister(ADS124_REG_PGA);

  PC_SERIAL.print("TEST,ADS1299_ID,0x");
  PC_SERIAL.println(ads1299ID, HEX);

  PC_SERIAL.print("TEST,ADS124S08_ID,0x");
  PC_SERIAL.println(ads124ID, HEX);

  PC_SERIAL.print("TEST,ADS124S08_PGA,0x");
  PC_SERIAL.println(ads124PGA, HEX);
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

  PC_SERIAL.print("DATA,");
  PC_SERIAL.print(sampleCounter);
  PC_SERIAL.print(",");
  PC_SERIAL.print(timestampMicros);
  PC_SERIAL.print(",");
  PC_SERIAL.print(timeSeconds, 6);
  PC_SERIAL.print(",");
  PC_SERIAL.print(cvPotential, 6);
  PC_SERIAL.print(",");
  PC_SERIAL.print(dacVoltage, 6);
  PC_SERIAL.print(",");
  PC_SERIAL.print(dacCode);
  PC_SERIAL.print(",");
  PC_SERIAL.print(phase, 6);

  for (int i = 0; i < NUM_ADS124_CHANNELS; i++) {
    PC_SERIAL.print(",");
    PC_SERIAL.print(ads124Voltages[i], 8);
  }

  PC_SERIAL.print(",");
  PC_SERIAL.print(I_GLU, 12);
  PC_SERIAL.print(",");
  PC_SERIAL.print(I_LAC, 12);
  PC_SERIAL.print(",");
  PC_SERIAL.print(I_PYR, 12);

  for (int i = 0; i < NUM_ADS1299_CHANNELS; i++) {
    PC_SERIAL.print(",");
    PC_SERIAL.print(ads1299Voltages[i], 10);
  }

  PC_SERIAL.println();
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

  channelVoltages[3] = 0.100 + 0.010 * sin(2.0 * PI * 0.05 * t);
}

// =====================================================
// Simulated ADS1299 data
// =====================================================

void readSimulatedADS1299(float neuralVoltages[NUM_ADS1299_CHANNELS], unsigned long timestampMicros) {
  float t = (timestampMicros - interrogationStartMicros) / 1000000.0;

  neuralVoltages[0] = 1.0e-3 * sin(2.0 * PI * 10.0 * t);
  neuralVoltages[1] = 0.8e-3 * sin(2.0 * PI * 12.0 * t + 0.3);
  neuralVoltages[2] = 0.7e-3 * sin(2.0 * PI * 15.0 * t + 0.6);
  neuralVoltages[3] = 0.6e-3 * sin(2.0 * PI * 18.0 * t + 0.9);
  neuralVoltages[4] = 0.5e-3 * sin(2.0 * PI * 20.0 * t + 1.2);
  neuralVoltages[5] = 0.5e-3 * sin(2.0 * PI * 22.0 * t + 1.5);

  neuralVoltages[6] = 100.0e-6 * sin(2.0 * PI * 8.0 * t);
  neuralVoltages[7] = 80.0e-6 * sin(2.0 * PI * 18.0 * t);
}

// =====================================================
// Software SPI implementation, Mode 1 style
// =====================================================

void softSPIBegin() {
  pinMode(SOFT_SPI_SCLK, OUTPUT);
  pinMode(SOFT_SPI_MOSI, OUTPUT);
  pinMode(SOFT_SPI_MISO, INPUT);

  digitalWrite(SOFT_SPI_SCLK, LOW);
  digitalWrite(SOFT_SPI_MOSI, LOW);
}

byte softSPITransfer(byte outByte) {
  byte inByte = 0;

  for (int bit = 7; bit >= 0; bit--) {

    // For CPHA = 1, prepare MOSI while clock is low
    digitalWrite(SOFT_SPI_MOSI, (outByte & (1 << bit)) ? HIGH : LOW);
    delayMicroseconds(SOFT_SPI_HALF_PERIOD_US);

    // Leading edge: clock goes high
    digitalWrite(SOFT_SPI_SCLK, HIGH);
    delayMicroseconds(SOFT_SPI_HALF_PERIOD_US);

    // Trailing edge: clock goes low
    digitalWrite(SOFT_SPI_SCLK, LOW);
    delayMicroseconds(SOFT_SPI_HALF_PERIOD_US);

    // Sample after falling edge for mode 1
    if (digitalRead(SOFT_SPI_MISO)) {
      inByte |= (1 << bit);
    }

    delayMicroseconds(SOFT_SPI_HALF_PERIOD_US);
  }

  return inByte;
}

void softSPIWriteCommand(int csPin, byte commandByte) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  softSPITransfer(commandByte);
  delayMicroseconds(2);
  digitalWrite(csPin, HIGH);
}

byte softSPIReadRegister(int csPin, byte rregCommand, byte regAddress) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);

  softSPITransfer(rregCommand | regAddress);
  softSPITransfer(0x00); // read one register
  byte value = softSPITransfer(0x00);

  delayMicroseconds(2);
  digitalWrite(csPin, HIGH);

  return value;
}

void softSPIWriteRegister(int csPin, byte wregCommand, byte regAddress, byte value) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);

  softSPITransfer(wregCommand | regAddress);
  softSPITransfer(0x00); // write one register
  softSPITransfer(value);

  delayMicroseconds(2);
  digitalWrite(csPin, HIGH);
}

int32_t signExtend24(byte b1, byte b2, byte b3) {
  int32_t raw = ((int32_t)b1 << 16) | ((int32_t)b2 << 8) | b3;

  if (raw & 0x800000) {
    raw |= 0xFF000000;
  }

  return raw;
}

// =====================================================
// ADS initialisation
// =====================================================

void initialiseADS() {
  PC_SERIAL.println("INFO,INITIALISING_ADS_DEVICES");

  // Reset both ADCs first, assuming RESET_ADS is shared
  digitalWrite(RESET_ADS, LOW);
  delay(5);
  digitalWrite(RESET_ADS, HIGH);
  delay(20);

  initialiseADS124S08_noReset();
  initialiseADS1299_noReset();

  adsInitialised = true;

  PC_SERIAL.println("INFO,ADS_INITIALISATION_COMPLETE");
}

void initialiseADS124S08_noReset() {
  softSPIWriteCommand(CS_ADS124S08, ADS124_CMD_RESET);
  delay(10);

  softSPIWriteRegister(CS_ADS124S08, ADS124_CMD_WREG, ADS124_REG_PGA, 0x00);
  softSPIWriteRegister(CS_ADS124S08, ADS124_CMD_WREG, ADS124_REG_DATARATE, 0x14);
  softSPIWriteRegister(CS_ADS124S08, ADS124_CMD_WREG, ADS124_REG_REF, 0x10);

  PC_SERIAL.println("INFO,ADS124S08_INITIALISED");
}

void initialiseADS1299_noReset() {
  digitalWrite(PWDN_ADS1299, HIGH);
  delay(10);

  softSPIWriteCommand(CS_ADS1299, ADS1299_CMD_RESET);
  delay(20);

  softSPIWriteCommand(CS_ADS1299, ADS1299_CMD_SDATAC);
  delay(5);

  byte id = ads1299ReadRegister(ADS1299_REG_ID);
  PC_SERIAL.print("INFO,ADS1299_ID,0x");
  PC_SERIAL.println(id, HEX);

  ads1299WriteRegister(ADS1299_REG_CONFIG1, 0x96);
  ads1299WriteRegister(ADS1299_REG_CONFIG2, 0xD0);
  ads1299WriteRegister(ADS1299_REG_CONFIG3, 0xEC);

  for (int ch = 0; ch < NUM_ADS1299_CHANNELS; ch++) {
    ads1299WriteRegister(ADS1299_REG_CH1SET + ch, 0x60);
  }

  digitalWrite(START_ADS1299, HIGH);
  delay(2);

  softSPIWriteCommand(CS_ADS1299, ADS1299_CMD_START);
  delay(5);

  PC_SERIAL.println("INFO,ADS1299_INITIALISED");
}

// =====================================================
// ADS124S08 real readout
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
  configureADS124S08InputChannel(channel);

  softSPIWriteCommand(CS_ADS124S08, ADS124_CMD_START);

  unsigned long t0 = millis();

  while (digitalRead(DRDY_ADS124S08) == HIGH) {
    if (millis() - t0 > 100) {
      PC_SERIAL.println("WARN,ADS124S08_DRDY_TIMEOUT");
      break;
    }
  }

  int32_t raw = ads124s08ReadDataCommand();

  softSPIWriteCommand(CS_ADS124S08, ADS124_CMD_STOP);

  return raw;
}

void configureADS124S08InputChannel(int channel) {
  /*
    Assumed single-ended mapping:
    positive input = AIN channel number
    negative input = AINCOM code
  */

  byte positiveInput = channel;
  byte negativeInput = 0x0C;

  byte muxValue = (positiveInput << 4) | negativeInput;

  softSPIWriteRegister(CS_ADS124S08, ADS124_CMD_WREG, ADS124_REG_INPMUX, muxValue);

  delayMicroseconds(100);
}

int32_t ads124s08ReadDataCommand() {
  digitalWrite(CS_ADS124S08, LOW);
  delayMicroseconds(2);

  softSPITransfer(ADS124_CMD_RDATA);

  byte b1 = softSPITransfer(0x00);
  byte b2 = softSPITransfer(0x00);
  byte b3 = softSPITransfer(0x00);

  delayMicroseconds(2);
  digitalWrite(CS_ADS124S08, HIGH);

  return signExtend24(b1, b2, b3);
}

byte ads124ReadRegister(byte regAddress) {
  return softSPIReadRegister(CS_ADS124S08, ADS124_CMD_RREG, regAddress);
}

void ads124WriteRegister(byte regAddress, byte value) {
  softSPIWriteRegister(CS_ADS124S08, ADS124_CMD_WREG, regAddress, value);
}

// =====================================================
// ADS1299 real readout
// =====================================================

void readADS1299Channels(float neuralVoltages[NUM_ADS1299_CHANNELS]) {
  int32_t rawChannels[NUM_ADS1299_CHANNELS];

  bool ok = readADS1299Frame(rawChannels);

  if (!ok) {
    PC_SERIAL.println("WARN,ADS1299_FRAME_READ_FAILED");

    for (int i = 0; i < NUM_ADS1299_CHANNELS; i++) {
      neuralVoltages[i] = 0.0;
    }

    return;
  }

  for (int i = 0; i < NUM_ADS1299_CHANNELS; i++) {
    neuralVoltages[i] = ads1299RawToVoltage(rawChannels[i]);
  }
}

float ads1299RawToVoltage(int32_t rawCode) {
  return ((float)rawCode / 8388607.0) * (ads1299Vref / ads1299Gain);
}

bool readADS1299Frame(int32_t rawChannels[NUM_ADS1299_CHANNELS]) {
  unsigned long t0 = millis();

  while (digitalRead(DRDY_ADS1299) == HIGH) {
    if (millis() - t0 > 100) {
      return false;
    }
  }

  digitalWrite(CS_ADS1299, LOW);
  delayMicroseconds(2);

  // On-demand RDATA read.
  softSPITransfer(ADS1299_CMD_RDATA);

  // 3 status bytes.
  byte status1 = softSPITransfer(0x00);
  byte status2 = softSPITransfer(0x00);
  byte status3 = softSPITransfer(0x00);

  for (int ch = 0; ch < NUM_ADS1299_CHANNELS; ch++) {
    byte b1 = softSPITransfer(0x00);
    byte b2 = softSPITransfer(0x00);
    byte b3 = softSPITransfer(0x00);

    rawChannels[ch] = signExtend24(b1, b2, b3);
  }

  delayMicroseconds(2);
  digitalWrite(CS_ADS1299, HIGH);

  return true;
}

byte ads1299ReadRegister(byte regAddress) {
  return softSPIReadRegister(CS_ADS1299, ADS1299_CMD_RREG, regAddress);
}

void ads1299WriteRegister(byte regAddress, byte value) {
  softSPIWriteRegister(CS_ADS1299, ADS1299_CMD_WREG, regAddress, value);
}