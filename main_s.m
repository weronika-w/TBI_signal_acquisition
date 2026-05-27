%% ================================================================
%  Simultaneous CV interrogation for amperometric AFE
% ================================================================
%
% Vdd2 = 1.25;  The non-inverting input of U1 is biased at Vdd2 = 1.25 V.
%
% V_WE = Vdd2;  The working electrode is held at approx. Vdd2.
%
% V_RE = V_DAC; U1.1 is a unity gain buffer.
%
% E_WE_RE = V_WE - V_RE;    Electrochemical potential is the voltage 
% difference between working and reference electrodes.
%
% V_DAC = Vdd2 - E_desired;   Voltage that has to be applied by DAC to 
% apply a desired CV potential E_desired.
%
% Iin = (Vdd2 - Vout_amp) / Rf  TIA output currnet conversion

clear; clc; close all;

%% ================================================================
%  USER CONFIGURATION
% ================================================================

cfg = struct();

% ------------------------------------------------
% AFE voltage references
% ------------------------------------------------

cfg.Vdd1 = 2.5;        % Positive analogue supply [V]
cfg.Vdd2 = 1.25;       % TIA reference [V]
cfg.V_TIA_ref = cfg.Vdd2;

% DAC limits.
cfg.DAC_min = 0.0;
cfg.DAC_max = 2.5;

% ------------------------------------------------
% CV waveform in electrochemical potential
% ------------------------------------------------
%
% E_WE_RE = V_WE - V_RE  The desired working-electrode vs reference-electrode potential.
%
% Since V_WE ≈ Vdd2 and V_RE ≈ V_DAC:
% V_DAC = Vdd2 - E_WE_RE

cfg.E_start = -0.4;        % Start potential WE vs RE [V]
cfg.E_upper =  0.6;        % Upper vertex potential WE vs RE [V]
cfg.E_lower = -0.4;        % Lower vertex potential WE vs RE [V]

cfg.dE = 0.005;            % Potential step [V]
cfg.scanRate = 0.05;       % Scan rate [V/s]

cfg.numCycles = 1;

% Time per CV step.
cfg.stepTime = cfg.dE / cfg.scanRate;

% Settling after DAC update, must be shorter than stepTime.
cfg.settlingTime = 0.005;

% ------------------------------------------------
% Amperometric channels
% ------------------------------------------------

cfg.numChannels = 4;

% Feedback resistor selected in the TIA.
%   R1 = 100 MOhm with C1 = 10 pF
%   R2 = 10 MOhm  with C2 = 100 pF
%
% Use one feedback path at a time.
%
% 100 MOhm gives higher sensitivity but lower current range.
% 10 MOhm gives lower sensitivity but higher current range.

cfg.feedbackMode = "100M";     % Choose "100M" or "10M"

switch cfg.feedbackMode
    case "100M"
        cfg.Rf = 100e6 * ones(1, cfg.numChannels);
        cfg.Cf = 10e-12 * ones(1, cfg.numChannels);

    case "10M"
        cfg.Rf = 10e6 * ones(1, cfg.numChannels);
        cfg.Cf = 100e-12 * ones(1, cfg.numChannels);

    otherwise
        error("Invalid feedback mode.");
end

% Passive output LPF from schematic:
% R3 = 10 kOhm, C3 = 600 nF
cfg.R_LPF = 10e3;
cfg.C_LPF = 600e-9;
cfg.f_LPF = 1 / (2*pi*cfg.R_LPF*cfg.C_LPF);

% Feedback network cut-off:
cfg.f_TIA_feedback = 1 ./ (2*pi*cfg.Rf.*cfg.Cf);

% ------------------------------------------------
% ADC settings
% ------------------------------------------------

cfg.adcRef = 2.5;          % ADC full-scale reference [V]
cfg.adcBits = 24;          % ADC resolution
cfg.samplesPerPoint = 5;

cfg.saturationLow = 0.05 * cfg.adcRef;
cfg.saturationHigh = 0.95 * cfg.adcRef;

% ------------------------------------------------
% Calibration / degradation tracking
% ------------------------------------------------
%
% Initial electrochemical response feature for each channel, from pre-implantation calibration.
%

cfg.expectedDeltaI = [10e-9, 10e-9, 10e-9, 10e-9];     % A; placeholder
cfg.featureE1 = 0.00;       % First potential for response feature [V]; pleceholder
cfg.featureE2 = 0.20;       % Second potential for response feature [V]; pleceholder

% Initial analyte sensitivity from pre-implantation calibration.
% Example units: A/mM.
cfg.initialAnalyteSensitivity = [20e-9, 22e-9, 19e-9, 21e-9];

% ------------------------------------------------
% Runtime mode
% ------------------------------------------------

cfg.simulationMode = true;      % true = simulated hardware
                                % false = replace hardware functions

cfg.saveResults = true;
cfg.outputFilename = "AFE_simultaneous_CV_results.mat";

%% ================================================================
%  SAFETY CHECKS
% ================================================================

fprintf("TIA feedback mode: %s\n", cfg.feedbackMode);
fprintf("Rf = %.3g Ohm\n", cfg.Rf(1));
fprintf("TIA feedback cut-off ≈ %.2f Hz\n", cfg.f_TIA_feedback(1));
fprintf("Output LPF cut-off ≈ %.2f Hz\n", cfg.f_LPF);
fprintf("CV step time = %.4f s\n", cfg.stepTime);

if cfg.settlingTime >= cfg.stepTime
    error("settlingTime must be shorter than stepTime.");
end

%% ================================================================
%  INITIALISE HARDWARE
% ================================================================

initialiseHardware(cfg);

configureFeedbackNetwork(cfg);

%% ================================================================
%  CREATE DESIRED ELECTROCHEMICAL CV WAVEFORM
% ================================================================

E_waveform = createCVWaveform( ...
    cfg.E_start, ...
    cfg.E_upper, ...
    cfg.E_lower, ...
    cfg.dE, ...
    cfg.numCycles);

numSteps = length(E_waveform);

% Convert desired WE-RE potential to DAC voltage.
DAC_waveform = electrodePotentialToDAC(E_waveform, cfg);

% Check DAC limits.
if any(DAC_waveform < cfg.DAC_min) || any(DAC_waveform > cfg.DAC_max)
    error("Requested CV waveform exceeds DAC range. Adjust E_start/E_upper/E_lower.");
end

fprintf("CV waveform contains %d points.\n", numSteps);
fprintf("Estimated duration = %.2f s\n", numSteps * cfg.stepTime);

%% ================================================================
%  PREALLOCATE ARRAYS
% ================================================================

time_s = zeros(numSteps, 1);

V_DAC = zeros(numSteps, 1);
E_applied = zeros(numSteps, 1);

Vout_amp = zeros(numSteps, cfg.numChannels);
I_amp = zeros(numSteps, cfg.numChannels);

status = strings(numSteps, cfg.numChannels);

%% ================================================================
%  MAIN SIMULTANEOUS CV LOOP
% ================================================================

disp("Starting simultaneous common-reference CV...");

tic;

for k = 1:numSteps

    stepTimer = tic;

    % Desired electrochemical potential.
    E_cmd = E_waveform(k);

    % Corresponding DAC voltage.
    Vdac_cmd = DAC_waveform(k);

    % ------------------------------------------------------------
    % 1. Apply DAC voltage to reference electrode buffer
    % ------------------------------------------------------------

    setDACVoltage(Vdac_cmd, cfg);

    % ------------------------------------------------------------
    % 2. Let DAC, reference buffer, TIA and output LPF settle
    % ------------------------------------------------------------

    pause(cfg.settlingTime);

    % ------------------------------------------------------------
    % 3. Read all amperometric TIA outputs
    % ------------------------------------------------------------

    Vout_channels = readAllAmpChannels(cfg, E_cmd);

    % ------------------------------------------------------------
    % 4. Convert TIA output voltage to input current
    % ------------------------------------------------------------

    time_s(k) = toc;

    V_DAC(k) = Vdac_cmd;
    E_applied(k) = E_cmd;

    for ch = 1:cfg.numChannels

        Vout_amp(k, ch) = Vout_channels(ch);

        I_amp(k, ch) = tiaVoltageToCurrent( ...
            Vout_amp(k, ch), ...
            cfg.V_TIA_ref, ...
            cfg.Rf(ch));

        status(k, ch) = checkSaturation( ...
            Vout_amp(k, ch), ...
            cfg.saturationLow, ...
            cfg.saturationHigh);
    end

    % ------------------------------------------------------------
    % 5. Enforce scan-rate timing
    % ------------------------------------------------------------

    elapsedStep = toc(stepTimer);
    remainingTime = cfg.stepTime - elapsedStep;

    if remainingTime > 0
        pause(remainingTime);
    else
        warning("Step %d overran by %.4f s. Reduce samplesPerPoint or scan rate.", ...
            k, abs(remainingTime));
    end
end

disp("CV acquisition complete.");

%% ================================================================
%  RETURN DAC TO RESTING VALUE
% ================================================================

E_rest = cfg.E_start;
Vdac_rest = electrodePotentialToDAC(E_rest, cfg);
setDACVoltage(Vdac_rest, cfg);

fprintf("DAC returned to %.3f V, corresponding to E = %.3f V WE vs RE.\n", ...
    Vdac_rest, E_rest);

%% ================================================================
%  PACKAGE RESULTS
% ================================================================

CV = struct();

CV.cfg = cfg;
CV.time_s = time_s;
CV.E_applied = E_applied;
CV.V_DAC = V_DAC;
CV.Vout_amp = Vout_amp;
CV.I_amp = I_amp;
CV.status = status;

CV.features = extractCalibrationFeatures(CV);
CV.updatedCalibration = estimateSensitivityDrift(CV);

%% ================================================================
%  SAVE RESULTS
% ================================================================

if cfg.saveResults
    save(cfg.outputFilename, "CV");
    fprintf("Saved results to %s\n", cfg.outputFilename);
end

%% ================================================================
%  PLOT RESULTS
% ================================================================

plotResults(CV);

disp("Done.");

%% ================================================================
%  LOCAL FUNCTIONS
% ================================================================

function initialiseHardware(cfg)

    if cfg.simulationMode
        disp("Simulation mode enabled.");
        disp("No real DAC/ADC hardware initialised.");
    else
        % Replace with hardware setup once available
        %
        %   a = arduino(...);
        %   configure SPI ADC;
        %   configure DAC;
        %   configure GPIO pins for feedback switches;
        %
        disp("Initialising real hardware...");
    end
end

% ----------------------------------------------------------------

function configureFeedbackNetwork(cfg)

    if cfg.simulationMode
        fprintf("Simulated feedback network set to %s.\n", cfg.feedbackMode);
    else
        % Replace with GPIO control for analogue switch.
        %
        % if cfg.feedbackMode == "100M"
        %     close switch SW-SPST1;
        %     open  switch SW-SPST2;
        % elseif cfg.feedbackMode == "10M"
        %     open  switch SW-SPST1;
        %     close switch SW-SPST2;
        % end
    end
end

% ----------------------------------------------------------------

function E = createCVWaveform(E_start, E_upper, E_lower, dE, numCycles)

    forwardSweep = E_start:dE:E_upper;
    reverseSweep = E_upper:-dE:E_lower;

    singleCycle = [forwardSweep, reverseSweep(2:end)];

    E = repmat(singleCycle, 1, numCycles);
    E = E(:);
end

% ----------------------------------------------------------------

function Vdac = electrodePotentialToDAC(E_WE_RE, cfg)

    Vdac = cfg.Vdd2 - E_WE_RE;
end

% ----------------------------------------------------------------

function setDACVoltage(Vdac, cfg)

    if cfg.simulationMode
        % Simulated DAC write.
        % Do nothing.
    else
        % Replace with DAC write function.
        %
        %   writeVoltage(a, "DAC0", Vdac);
        %
    end
end

% ----------------------------------------------------------------

function Vout_channels = readAllAmpChannels(cfg, E_cmd)

    Vout_channels = zeros(1, cfg.numChannels);

    for ch = 1:cfg.numChannels

        samples = zeros(cfg.samplesPerPoint, 1);

        for n = 1:cfg.samplesPerPoint
            samples(n) = readADCVoltage(ch, cfg, E_cmd);
        end

        Vout_channels(ch) = mean(samples);
    end
end

% ----------------------------------------------------------------

function Vout = readADCVoltage(channelID, cfg, E_cmd)

    if cfg.simulationMode

        % --------------------------------------------------------
        % Simulated electrode current
        % --------------------------------------------------------
        %
        % This creates a fake CV-like response so that the software can
        % be tested before the real hardware is connected.

        channelScale = [1.00, 0.85, 1.15, 0.70];

        if channelID > length(channelScale)
            scale = 1.0;
        else
            scale = channelScale(channelID);
        end

        % Pseudo-faradaic response.
        I_faradaic = scale * 30e-9 * tanh(8 * (E_cmd - 0.15));

        % Background/capacitive-like component.
        I_background = scale * 3e-9 * E_cmd;

        % Offset current.
        I_offset = (channelID - 1) * 1e-9;

        % Noise.
        I_noise = 0.5e-9 * randn();

        Iin = I_faradaic + I_background + I_offset + I_noise;

        % TIA equation for inverting input:
        %
        % Vout = Vref - Iin * Rf

        Vout = cfg.V_TIA_ref - Iin * cfg.Rf(channelID);

        % Clip to ADC range.
        Vout = min(max(Vout, 0), cfg.adcRef);

    else
        % Replace with real ADC read.
        %
        % If using an external ADC with MUX:
        %
        %   selectADCChannel(channelID);
        %   raw = readADC_SPI();
        %   Vout = adcCodeToVoltage(raw, cfg.adcRef, cfg.adcBits);
        %
        % If using multi-channel ADC:
        %
        %   raw = readADCChannel(channelID);
        %   Vout = adcCodeToVoltage(raw, cfg.adcRef, cfg.adcBits);
        %
        error("Real ADC reading not implemented.");
    end
end

% ----------------------------------------------------------------

function Iin = tiaVoltageToCurrent(Vout, Vref, Rf)

    Iin = (Vref - Vout) / Rf;
end

% ----------------------------------------------------------------

function status = checkSaturation(Vout, Vlow, Vhigh)

    if Vout <= Vlow
        status = "SATURATED_LOW";
    elseif Vout >= Vhigh
        status = "SATURATED_HIGH";
    else
        status = "OK";
    end
end

% ----------------------------------------------------------------

function features = extractCalibrationFeatures(CV)

    cfg = CV.cfg;

    E = CV.E_applied;
    I = CV.I_amp;

    features = struct();

    for ch = 1:cfg.numChannels

        I_ch = I(:, ch);

        % Current at feature potentials E1 and E2.
        [~, idx1] = min(abs(E - cfg.featureE1));
        [~, idx2] = min(abs(E - cfg.featureE2));

        I_E1 = I_ch(idx1);
        I_E2 = I_ch(idx2);

        deltaI = I_E2 - I_E1;
        deltaE = cfg.featureE2 - cfg.featureE1;

        slope_dIdE = deltaI / deltaE;

        % Peak features.
        [Imax, idxMax] = max(I_ch);
        [Imin, idxMin] = min(I_ch);

        features(ch).channel = ch;

        features(ch).E1 = cfg.featureE1;
        features(ch).E2 = cfg.featureE2;

        features(ch).I_E1 = I_E1;
        features(ch).I_E2 = I_E2;

        features(ch).deltaI = deltaI;
        features(ch).deltaE = deltaE;
        features(ch).slope_dIdE = slope_dIdE;

        features(ch).Imax = Imax;
        features(ch).E_at_Imax = E(idxMax);

        features(ch).Imin = Imin;
        features(ch).E_at_Imin = E(idxMin);
    end
end

% ----------------------------------------------------------------

function updatedCalibration = estimateSensitivityDrift(CV)

    cfg = CV.cfg;
    features = CV.features;

    updatedCalibration = struct();

    for ch = 1:cfg.numChannels

        measuredDeltaI = features(ch).deltaI;
        expectedDeltaI = cfg.expectedDeltaI(ch);

        responseFactor = measuredDeltaI / expectedDeltaI;

        updatedSensitivity = cfg.initialAnalyteSensitivity(ch) * responseFactor;

        updatedCalibration(ch).channel = ch;
        updatedCalibration(ch).measuredDeltaI = measuredDeltaI;
        updatedCalibration(ch).expectedDeltaI = expectedDeltaI;
        updatedCalibration(ch).responseFactor = responseFactor;
        updatedCalibration(ch).initialAnalyteSensitivity = cfg.initialAnalyteSensitivity(ch);
        updatedCalibration(ch).updatedAnalyteSensitivity = updatedSensitivity;
    end
end

% ----------------------------------------------------------------

function plotResults(CV)

    cfg = CV.cfg;

    E = CV.E_applied;
    Vdac = CV.V_DAC;
    I = CV.I_amp;
    Vout = CV.Vout_amp;

    numChannels = cfg.numChannels;

    % ------------------------------------------------------------
    % CV current plots
    % ------------------------------------------------------------

    figure;
    hold on;

    for ch = 1:numChannels
        plot(E, I(:, ch) * 1e9, ...
            'LineWidth', 1.5, ...
            'DisplayName', sprintf('Channel %d', ch));
    end

    xlabel('Applied potential E_{WE-RE} / V');
    ylabel('Input current / nA');
    title('Simultaneous common-reference cyclic voltammetry');
    legend('Location', 'best');
    grid on;

    % ------------------------------------------------------------
    % TIA output voltage plots
    % ------------------------------------------------------------

    figure;
    hold on;

    for ch = 1:numChannels
        plot(E, Vout(:, ch), ...
            'LineWidth', 1.5, ...
            'DisplayName', sprintf('Channel %d', ch));
    end

    xlabel('Applied potential E_{WE-RE} / V');
    ylabel('TIA output voltage V_{out,amp} / V');
    title('TIA output during CV');
    legend('Location', 'best');
    grid on;

    % ------------------------------------------------------------
    % DAC command waveform
    % ------------------------------------------------------------

    figure;
    plot(CV.time_s, Vdac, 'LineWidth', 1.5);
    xlabel('Time / s');
    ylabel('DAC voltage applied to reference electrode / V');
    title('DAC waveform driving reference electrode buffer');
    grid on;

    % ------------------------------------------------------------
    % Applied electrochemical waveform
    % ------------------------------------------------------------

    figure;
    plot(CV.time_s, E, 'LineWidth', 1.5);
    xlabel('Time / s');
    ylabel('E_{WE-RE} / V');
    title('Applied electrochemical CV waveform');
    grid on;

    % ------------------------------------------------------------
    % Print calibration feature summary
    % ------------------------------------------------------------

    fprintf("\nCalibration / degradation feature summary\n");
    fprintf("------------------------------------------------------------\n");

    for ch = 1:numChannels

        f = CV.features(ch);
        c = CV.updatedCalibration(ch);

        fprintf("Channel %d\n", ch);
        fprintf("  I(E1 = %.3f V): %.3f nA\n", f.E1, f.I_E1 * 1e9);
        fprintf("  I(E2 = %.3f V): %.3f nA\n", f.E2, f.I_E2 * 1e9);
        fprintf("  Measured delta I: %.3f nA\n", f.deltaI * 1e9);
        fprintf("  Expected delta I: %.3f nA\n", c.expectedDeltaI * 1e9);
        fprintf("  Response factor: %.3f\n", c.responseFactor);
        fprintf("  Initial analyte sensitivity: %.3f nA/unit\n", ...
            c.initialAnalyteSensitivity * 1e9);
        fprintf("  Updated analyte sensitivity: %.3f nA/unit\n", ...
            c.updatedAnalyteSensitivity * 1e9);
        fprintf("\n");
    end
end