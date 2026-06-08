%% ================================================================
%  Calibration summary from Arduino-generated CSV
% ================================================================
%
% Input:
%   interrogation_combined_data.csv
%
% Expected columns:
%   sample_counter, timestamp_us, time_s,
%   cv_potential, dac_voltage, dac_code, phase,
%   AMP_GLU_V, AMP_LAC_V, AMP_PYR_V, POT_K_V,
%   AMP_GLU_A, AMP_LAC_A, AMP_PYR_A,
%   ECOG1_V, ECOG2_V, ECOG3_V, ECOG4_V, ECOG5_V, ECOG6_V, EEG1_V, EEG2_V
%
% Output:
%   CV_results_from_arduino.mat
%   calibration_feature_summary.csv / .xlsx
%   updated_calibration_summary.csv / .xlsx
%   ads1299_neural_summary.csv / .xlsx

clear; clc; close all;

%% ================================================================
%  USER CONFIGURATION
% ================================================================

cfg = struct();

% ------------------------------------------------
% Input/output files
% ------------------------------------------------

cfg.inputCsv = "interrogation_combined_data.csv";

cfg.saveResults = true;
cfg.outputMatFilename = "CV_results_from_arduino.mat";

cfg.featureSummaryCsv = "calibration_feature_summary.csv";
cfg.featureSummaryExcel = "calibration_feature_summary.xlsx";

cfg.updatedCalibrationCsv = "updated_calibration_summary.csv";
cfg.updatedCalibrationExcel = "updated_calibration_summary.xlsx";

cfg.neuralSummaryCsv = "ads1299_neural_summary.csv";
cfg.neuralSummaryExcel = "ads1299_neural_summary.xlsx";

% ------------------------------------------------
% AFE voltage references
% ------------------------------------------------

cfg.Vdd1 = 2.5;          % Positive analogue supply [V]
cfg.Vdd2 = 1.25;         % TIA reference [V]
cfg.V_TIA_ref = cfg.Vdd2;

% DAC limits used during Arduino waveform generation
cfg.DAC_min = 0.0;
cfg.DAC_max = 2.5;

% ------------------------------------------------
% CV waveform settings
% ------------------------------------------------

cfg.E_start = -0.4;      % Start potential WE vs RE [V]
cfg.E_upper =  0.6;      % Upper vertex potential WE vs RE [V]
cfg.E_lower = -0.4;      % Lower vertex potential WE vs RE [V]

cfg.dE = 0.005;          % Potential step [V]
cfg.scanRate = 0.05;     % Scan rate [V/s]
cfg.stepTime = cfg.dE / cfg.scanRate;

% ------------------------------------------------
% Amperometric channels
% ------------------------------------------------

cfg.ampChannelNames = ["GLU", "LAC", "PYR"];
cfg.numChannels = numel(cfg.ampChannelNames);

cfg.feedbackMode = "100M";     % "100M" or "10M"

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

cfg.R_LPF = 10e3;
cfg.C_LPF = 600e-9;
cfg.f_LPF = 1 / (2*pi*cfg.R_LPF*cfg.C_LPF);

cfg.f_TIA_feedback = 1 ./ (2*pi*cfg.Rf.*cfg.Cf);

% ------------------------------------------------
% ADC / saturation settings
% ------------------------------------------------

cfg.adcRef = 2.5;
cfg.adcBits = 24;

cfg.saturationLow = 0.05 * cfg.adcRef;
cfg.saturationHigh = 0.95 * cfg.adcRef;

% ------------------------------------------------
% Calibration / degradation tracking
% ------------------------------------------------
%
% Placeholders, to be changed when testing with a known concentration solution 

cfg.expectedDeltaI = [10e-9, 10e-9, 10e-9];     % A

cfg.featureE1 = 0.00;       % First feature potential [V]
cfg.featureE2 = 0.20;       % Second feature potential [V]

cfg.initialAnalyteSensitivity = [20e-9, 22e-9, 19e-9];   % A/unit placeholder

% ------------------------------------------------
% Plot options
% ------------------------------------------------

cfg.makeCVCurrentPlot = true;
cfg.makeTIAVoltagePlot = true;
cfg.makeDACPlot = false;
cfg.makeAppliedPotentialPlot = false;
cfg.makeNeuralPlot = true;

%% ================================================================
%  LOAD CSV DATA
% ================================================================

T = readtable(cfg.inputCsv);

disp("Loaded data:");
disp(cfg.inputCsv);

requiredColumns = ["time_s", "cv_potential", "dac_voltage", ...
                   "AMP_GLU_V", "AMP_LAC_V", "AMP_PYR_V", ...
                   "AMP_GLU_A", "AMP_LAC_A", "AMP_PYR_A", ...
                   "ECOG1_V", "ECOG2_V", "ECOG3_V", "ECOG4_V", ...
                   "ECOG5_V", "ECOG6_V", "EEG1_V", "EEG2_V"];

for i = 1:numel(requiredColumns)
    if ~ismember(requiredColumns(i), string(T.Properties.VariableNames))
        error("Missing required column: %s", requiredColumns(i));
    end
end

%% ================================================================
%  SAFETY / CONFIGURATION SUMMARY
% ================================================================

fprintf("TIA feedback mode: %s\n", cfg.feedbackMode);
fprintf("Rf = %.3g Ohm\n", cfg.Rf(1));
fprintf("TIA feedback cut-off ≈ %.2f Hz\n", cfg.f_TIA_feedback(1));
fprintf("Output LPF cut-off ≈ %.2f Hz\n", cfg.f_LPF);
fprintf("Expected CV step time = %.4f s\n", cfg.stepTime);

%% ================================================================
%  RECONSTRUCT CV RESULT STRUCTURE
% ================================================================

CV = struct();

CV.cfg = cfg;

CV.time_s = T.time_s;
CV.E_applied = T.cv_potential;
CV.V_DAC = T.dac_voltage;

CV.Vout_amp = [T.AMP_GLU_V, T.AMP_LAC_V, T.AMP_PYR_V];
CV.I_amp = [T.AMP_GLU_A, T.AMP_LAC_A, T.AMP_PYR_A];

CV.neural = struct();
CV.neural.channelNames = ["ECOG1", "ECOG2", "ECOG3", "ECOG4", ...
                          "ECOG5", "ECOG6", "EEG1", "EEG2"];

CV.neural.voltage = [T.ECOG1_V, T.ECOG2_V, T.ECOG3_V, T.ECOG4_V, ...
                     T.ECOG5_V, T.ECOG6_V, T.EEG1_V, T.EEG2_V];

CV.status = strings(height(T), cfg.numChannels);

for k = 1:height(T)
    for ch = 1:cfg.numChannels
        CV.status(k, ch) = checkSaturation( ...
            CV.Vout_amp(k, ch), ...
            cfg.saturationLow, ...
            cfg.saturationHigh);
    end
end

%% ================================================================
%  EXTRACT FEATURES AND UPDATE CALIBRATION
% ================================================================

CV.features = extractCalibrationFeatures(CV);
CV.updatedCalibration = estimateSensitivityDrift(CV);

featureTable = featuresToTable(CV);
updatedCalibrationTable = updatedCalibrationToTable(CV);
neuralSummaryTable = neuralSummaryToTable(CV);

%% ================================================================
%  SAVE RESULTS
% ================================================================

if cfg.saveResults
    save(cfg.outputMatFilename, "CV");
    fprintf("Saved CV structure to %s\n", cfg.outputMatFilename);

    writetable(featureTable, cfg.featureSummaryCsv);
    writetable(featureTable, cfg.featureSummaryExcel);

    writetable(updatedCalibrationTable, cfg.updatedCalibrationCsv);
    writetable(updatedCalibrationTable, cfg.updatedCalibrationExcel);

    writetable(neuralSummaryTable, cfg.neuralSummaryCsv);
    writetable(neuralSummaryTable, cfg.neuralSummaryExcel);

    fprintf("Saved summary tables.\n");
end

%% ================================================================
%  PRINT SUMMARY
% ================================================================

fprintf("\nCalibration / degradation feature summary\n");
fprintf("------------------------------------------------------------\n");

for ch = 1:cfg.numChannels

    f = CV.features(ch);
    c = CV.updatedCalibration(ch);

    fprintf("Channel %d: %s\n", ch, cfg.ampChannelNames(ch));
    fprintf("  I(E1 = %.3f V): %.3f nA\n", f.E1, f.I_E1 * 1e9);
    fprintf("  I(E2 = %.3f V): %.3f nA\n", f.E2, f.I_E2 * 1e9);
    fprintf("  Measured delta I: %.3f nA\n", f.deltaI * 1e9);
    fprintf("  Expected delta I: %.3f nA\n", c.expectedDeltaI * 1e9);
    fprintf("  Response factor: %.3f\n", c.responseFactor);
    fprintf("  Initial analyte sensitivity: %.3f nA/unit\n", ...
        c.initialAnalyteSensitivity * 1e9);
    fprintf("  Updated analyte sensitivity: %.3f nA/unit\n", ...
        c.updatedAnalyteSensitivity * 1e9);
    fprintf("  Imax: %.3f nA at E = %.3f V\n", ...
        f.Imax * 1e9, f.E_at_Imax);
    fprintf("  Imin: %.3f nA at E = %.3f V\n", ...
        f.Imin * 1e9, f.E_at_Imin);
    fprintf("\n");
end

disp("Feature table:");
disp(featureTable);

disp("Updated calibration table:");
disp(updatedCalibrationTable);

disp("ADS1299 neural summary table:");
disp(neuralSummaryTable);

%% ================================================================
%  PLOT RESULTS
% ================================================================

plotResults(CV);

disp("Done.");

%% ================================================================
%  LOCAL FUNCTIONS
% ================================================================

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

        [~, idx1] = min(abs(E - cfg.featureE1));
        [~, idx2] = min(abs(E - cfg.featureE2));

        I_E1 = I_ch(idx1);
        I_E2 = I_ch(idx2);

        deltaI = I_E2 - I_E1;
        deltaE = cfg.featureE2 - cfg.featureE1;

        slope_dIdE = deltaI / deltaE;

        [Imax, idxMax] = max(I_ch);
        [Imin, idxMin] = min(I_ch);

        features(ch).channel = ch;
        features(ch).channelName = cfg.ampChannelNames(ch);

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
        updatedCalibration(ch).channelName = cfg.ampChannelNames(ch);
        updatedCalibration(ch).measuredDeltaI = measuredDeltaI;
        updatedCalibration(ch).expectedDeltaI = expectedDeltaI;
        updatedCalibration(ch).responseFactor = responseFactor;
        updatedCalibration(ch).initialAnalyteSensitivity = cfg.initialAnalyteSensitivity(ch);
        updatedCalibration(ch).updatedAnalyteSensitivity = updatedSensitivity;
    end
end

% ----------------------------------------------------------------

function featureTable = featuresToTable(CV)

    f = CV.features;
    cfg = CV.cfg;

    channel = zeros(cfg.numChannels, 1);
    channelName = strings(cfg.numChannels, 1);

    E1 = zeros(cfg.numChannels, 1);
    E2 = zeros(cfg.numChannels, 1);

    I_E1_A = zeros(cfg.numChannels, 1);
    I_E2_A = zeros(cfg.numChannels, 1);
    deltaI_A = zeros(cfg.numChannels, 1);
    deltaE_V = zeros(cfg.numChannels, 1);
    slope_dIdE_A_per_V = zeros(cfg.numChannels, 1);

    Imax_A = zeros(cfg.numChannels, 1);
    E_at_Imax_V = zeros(cfg.numChannels, 1);

    Imin_A = zeros(cfg.numChannels, 1);
    E_at_Imin_V = zeros(cfg.numChannels, 1);

    for ch = 1:cfg.numChannels
        channel(ch) = f(ch).channel;
        channelName(ch) = f(ch).channelName;

        E1(ch) = f(ch).E1;
        E2(ch) = f(ch).E2;

        I_E1_A(ch) = f(ch).I_E1;
        I_E2_A(ch) = f(ch).I_E2;
        deltaI_A(ch) = f(ch).deltaI;
        deltaE_V(ch) = f(ch).deltaE;
        slope_dIdE_A_per_V(ch) = f(ch).slope_dIdE;

        Imax_A(ch) = f(ch).Imax;
        E_at_Imax_V(ch) = f(ch).E_at_Imax;

        Imin_A(ch) = f(ch).Imin;
        E_at_Imin_V(ch) = f(ch).E_at_Imin;
    end

    featureTable = table(channel, channelName, E1, E2, ...
        I_E1_A, I_E2_A, deltaI_A, deltaE_V, slope_dIdE_A_per_V, ...
        Imax_A, E_at_Imax_V, Imin_A, E_at_Imin_V);
end

% ----------------------------------------------------------------

function updatedCalibrationTable = updatedCalibrationToTable(CV)

    c = CV.updatedCalibration;
    cfg = CV.cfg;

    channel = zeros(cfg.numChannels, 1);
    channelName = strings(cfg.numChannels, 1);

    measuredDeltaI_A = zeros(cfg.numChannels, 1);
    expectedDeltaI_A = zeros(cfg.numChannels, 1);
    responseFactor = zeros(cfg.numChannels, 1);
    initialAnalyteSensitivity_A_per_unit = zeros(cfg.numChannels, 1);
    updatedAnalyteSensitivity_A_per_unit = zeros(cfg.numChannels, 1);

    for ch = 1:cfg.numChannels
        channel(ch) = c(ch).channel;
        channelName(ch) = c(ch).channelName;

        measuredDeltaI_A(ch) = c(ch).measuredDeltaI;
        expectedDeltaI_A(ch) = c(ch).expectedDeltaI;
        responseFactor(ch) = c(ch).responseFactor;
        initialAnalyteSensitivity_A_per_unit(ch) = c(ch).initialAnalyteSensitivity;
        updatedAnalyteSensitivity_A_per_unit(ch) = c(ch).updatedAnalyteSensitivity;
    end

    updatedCalibrationTable = table(channel, channelName, ...
        measuredDeltaI_A, expectedDeltaI_A, responseFactor, ...
        initialAnalyteSensitivity_A_per_unit, ...
        updatedAnalyteSensitivity_A_per_unit);
end

% ----------------------------------------------------------------

function neuralSummaryTable = neuralSummaryToTable(CV)

    names = CV.neural.channelNames;
    V = CV.neural.voltage;

    n = numel(names);

    channelName = strings(n, 1);
    mean_V = zeros(n, 1);
    std_V = zeros(n, 1);
    min_V = zeros(n, 1);
    max_V = zeros(n, 1);
    pp_V = zeros(n, 1);
    rms_V = zeros(n, 1);

    for i = 1:n
        x = V(:, i);

        channelName(i) = names(i);
        mean_V(i) = mean(x);
        std_V(i) = std(x);
        min_V(i) = min(x);
        max_V(i) = max(x);
        pp_V(i) = max(x) - min(x);
        rms_V(i) = sqrt(mean(x.^2));
    end

    neuralSummaryTable = table(channelName, mean_V, std_V, min_V, max_V, pp_V, rms_V);
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

    if cfg.makeCVCurrentPlot
        figure;
        hold on;

        for ch = 1:numChannels
            plot(E, I(:, ch) * 1e9, ...
                'LineWidth', 1.5, ...
                'DisplayName', sprintf('%s', cfg.ampChannelNames(ch)));
        end

        xlabel('Applied potential E_{WE-RE} / V');
        ylabel('Input current / nA');
        title('Simultaneous common-reference cyclic voltammetry');
        legend('Location', 'best');
        grid on;
    end

    % ------------------------------------------------------------
    % TIA output voltage plots
    % ------------------------------------------------------------

    if cfg.makeTIAVoltagePlot
        figure;
        hold on;

        for ch = 1:numChannels
            plot(E, Vout(:, ch), ...
                'LineWidth', 1.5, ...
                'DisplayName', sprintf('%s', cfg.ampChannelNames(ch)));
        end

        xlabel('Applied potential E_{WE-RE} / V');
        ylabel('TIA output voltage V_{out,amp} / V');
        title('TIA output during CV');
        legend('Location', 'best');
        grid on;
    end

    % ------------------------------------------------------------
    % DAC command waveform
    % ------------------------------------------------------------

    if cfg.makeDACPlot
        figure;
        plot(CV.time_s, Vdac, 'LineWidth', 1.5);
        xlabel('Time / s');
        ylabel('DAC voltage applied to reference electrode / V');
        title('DAC waveform driving reference electrode buffer');
        grid on;
    end

    % ------------------------------------------------------------
    % Applied electrochemical waveform
    % ------------------------------------------------------------

    if cfg.makeAppliedPotentialPlot
        figure;
        plot(CV.time_s, E, 'LineWidth', 1.5);
        xlabel('Time / s');
        ylabel('E_{WE-RE} / V');
        title('Applied electrochemical CV waveform');
        grid on;
    end

    % ------------------------------------------------------------
    % ADS1299 neural signals
    % ------------------------------------------------------------

    if cfg.makeNeuralPlot
        figure;
        hold on;

        names = CV.neural.channelNames;
        V = CV.neural.voltage;

        for ch = 1:numel(names)
            plot(CV.time_s, V(:, ch), ...
                'LineWidth', 1.0, ...
                'DisplayName', names(ch));
        end

        xlabel('Time / s');
        ylabel('ADS1299 voltage / V');
        title('ADS1299 ECoG/EEG channels');
        legend('Location', 'best');
        grid on;
    end
end