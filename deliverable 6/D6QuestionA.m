% Parameters
d_w = 100;
target_fc_hz = 0.025;
wc_target = target_fc_hz * 2 * pi; % Target crossover (~0.1571 rad/s)

% 1. Define Plant G(s)
num_G = [5.1e-3, (0.2 + 0.049 * d_w), 250];
den_G = [5e5, (5e4 + 100 * d_w), 6.2e5, 4.9e4, 0];
G = tf(num_G, den_G);

% 2. Design Controller C1(s)
% Motivation: alpha scales the "lead" effect. For this system, a higher alpha 
% provides more phase lead at wc, pushing the Nyquist plot away from -1.
alpha = 1.5; % Increased alpha to improve stability and Modulus Margin
w_c_param = wc_target; 

% Base shape of the controller (Equation 3 without K)
C_shape = tf([alpha/w_c_param, 1], [1/(alpha*w_c_param), 1]);

% 3. Calculate K analytically to force crossover at wc_target
% Magnitude condition: |K * C_shape(j*wc) * G(j*wc)| = 1
mag_at_wc = abs(evalfr(C_shape * G, 1i * wc_target));
K = 44.6632;

% Final Controller and Open-Loop/Closed-Loop Transfer Functions
C1 = K * C_shape;
L1 = C1 * G;
T = feedback(L1, 1);
S = feedback(1, L1); % Sensitivity function

% 4. Verify Requirements
[gm, pm, wcg, wcp] = margin(L1);
f_c_actual = wcp / (2*pi);

% Calculate Modulus Margin (MM)
% MM is the minimum distance from L(jw) to the -1 point.
% This is equivalent to 1/max(abs(S))
[magS, ~] = bode(S, logspace(-3, 2, 1000));
Ms_linear = max(squeeze(magS));
Ms_db = 20*log10(Ms_linear); % Sensitivity peak in dB
MM_linear = 1 / Ms_linear;   % Distance from -1

% 5. Print Results
fprintf('--- Controller Parameters ---\n');
fprintf('K:     %.4f\n', K);
fprintf('alpha: %.2f\n', alpha);
fprintf('wc:    %.4f rad/s\n', w_c_param);

fprintf('\n--- Performance Metrics ---\n');
fprintf('Crossover Freq: %.4f Hz (Target: 0.025)\n', f_c_actual);
fprintf('Sensitivity Peak (Ms): %.2f dB (Requirement: <= 6dB)\n', Ms_db);
fprintf('Phase Margin:   %.2f degrees\n', pm);

% Final Stability Check
if isstable(T) && abs(f_c_actual - target_fc_hz) < 0.001 && Ms_db <= 6
    fprintf('\nSTATUS: ALL REQUIREMENTS MET\n');
else
    fprintf('\nSTATUS: REQUIREMENTS NOT MET\n');
    if ~isstable(T), fprintf('  [!] System is still Unstable. Try increasing alpha.\n'); end
    if Ms_db > 6,    fprintf('  [!] Modulus Margin requirement failed (Ms > 6dB).\n'); end
end

% Visual Verification
figure(1); margin(L1); grid on;
figure(2); nyquist(L1); grid on; axis([-2 0.5 -1.5 1.5]);

% 1. Setup
target_wc = 0.025 * 2 * pi;
alphas = 1:0.01:20; % Range of alpha to test
best_alpha = NaN;

for a = alphas
    % Define shape with current alpha
    C_s = tf([a/target_wc, 1], [1/(a*target_wc), 1]);
    
    % Calculate K for this specific alpha to hit target_wc
    K_val = 1 / abs(evalfr(C_s * G, 1i * target_wc));
    
    % Check Stability and Modulus Margin
    L_test = K_val * C_s * G;
    S_test = feedback(1, L_test);
    [magS, ~] = bode(S_test, logspace(-2, 1, 500));
    Ms_db = 20*log10(max(squeeze(magS)));
    
    if isstable(feedback(L_test, 1)) && Ms_db <= 6
        best_alpha = a;
        best_K = K_val;
        fprintf('Found it! Alpha = %.1f, K = %.4f gives Ms = %.2f dB\n', a, K_val, Ms_db);
        break; % Stop at the first alpha that works
    end
end

% question b)

% 1. Calculate Margins using Matlab's built-in tool
[gm_linear, pm, wcg, wcp] = margin(L1);

% Convert Gain Margin to dB
gm_db = 20*log10(gm_linear);

% 2. Calculate Modulus Margin (as Peak Sensitivity)
S = feedback(1, L1);
[magS, ~] = bode(S, logspace(-3, 2, 1000));
Ms_linear = max(squeeze(magS));
ms_db = 20*log10(Ms_linear); % This is the Sensitivity Peak (Ms)
mm_linear = 1/Ms_linear;      % This is the actual Modulus Margin (distance)

% 3. Print Results Nicely
fprintf('\n======================================\n');
fprintf('     STABILITY MARGIN ANALYSIS       \n');
fprintf('======================================\n');
fprintf('Gain Margin (GM):      %8.2f dB\n', gm_db);
fprintf('Phase Margin (PM):     %8.2f degrees\n', pm);
fprintf('Modulus Margin (Ms):   %8.2f dB\n', ms_db);
fprintf('--------------------------------------\n');
fprintf('Crossover Freq (fc):   %8.4f Hz\n', wcp/(2*pi));
fprintf('Stability Status:      %s\n', char(string(isstable(T))));
fprintf('======================================\n');

% question c)

% 1. Define Time Vector
t = 0:0.1:100; % Time from 0 to 100s

% 2. Perform Step Response
% T is the closed-loop transfer function: Load Position / Reference
[xl, t_out] = step(T, t); 

% 3. Calculate Tracking Error
% For a unit step input, Error = 1 - Output
e = 1 - xl;

% 4. Find Settling Time
% Requirement: |e(t)| < 0.025
threshold = 0.025;
% We look for the last time the absolute error was ABOVE the threshold
indices_outside = find(abs(e) >= threshold);

if isempty(indices_outside)
    settling_time = 0;
else
    last_index = indices_outside(end);
    % If the last index is the very end of our simulation, it hasn't settled yet
    if last_index == length(t)
        settling_time = NaN; 
        fprintf('Warning: System has not settled within 100s.\n');
    else
        settling_time = t(last_index + 1);
    end
end

% 5. Plotting
figure(4);
subplot(2,1,1);
plot(t_out, xl, 'LineWidth', 1.5);
ylabel('Position x_l (m)');
title('Closed-Loop Step Response');
grid on;

subplot(2,1,2);
plot(t_out, e, 'r', 'LineWidth', 1.5);
hold on;
yline(threshold, '--k', 'Threshold +0.025');
yline(-threshold, '--k', 'Threshold -0.025');
ylabel('Tracking Error e (m)');
xlabel('Time (s)');
title(['Error Signal (Settling Time: ', num2str(settling_time), 's)']);
grid on;

fprintf('The settling time for |e(t)| < 0.025m is: %.2f seconds\n', settling_time);


% question d)


% 1. Setup Fixed Design Parameters
time_vector = 0:0.1:100;

% 2. Run Analysis for Both Cases
% Original Case (dW = 100)
[data100] = analyze_crane(100, alpha, K, target_fc_hz, time_vector);

% Perturbed Case (dW = 10)
[data10] = analyze_crane(10, alpha, K, target_fc_hz, time_vector);

%% 3. Print Results Comparison (Questions 6b & 6c)
fprintf('\n======================================================\n');
fprintf('        PERFORMANCE COMPARISON: dW=100 vs dW=10       \n');
fprintf('======================================================\n');
fprintf('Metric              | dW = 100         | dW = 10       \n');
fprintf('--------------------|------------------|---------------\n');
fprintf('Gain Margin (GM)    | %8.2f dB     | %8.2f dB  \n', data100.gm_db, data10.gm_db);
fprintf('Phase Margin (PM)   | %8.2f deg    | %8.2f deg \n', data100.pm, data10.pm);
fprintf('Modulus Margin (Ms) | %8.2f dB     | %8.2f dB  \n', data100.ms_db, data10.ms_db);
fprintf('Settling Time (ts)  | %8.2f s      | %8.2f s   \n', data100.set_time, data10.set_time);
fprintf('Stability           | %10s       | %10s    \n', data100.stable, data10.stable);
fprintf('======================================================\n');

%% 4. Nyquist Plot Comparison (Question 6d)
figure(5);
hold on;

% 1. Improved Unit Circle (Solid/Grey for better visibility)
th = linspace(0, 2*pi, 200);
plot(cos(th), sin(th), 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'HandleVisibility', 'off'); 

% 2. Plot critical point -1 (Larger marker)
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');
text(-1.1, 0.1, '(-1,0)', 'Color', 'r', 'FontSize', 10); % Label the point

% Plot both Open-Loop Transfer Functions
[re100, im100] = nyquist(data100.L);
[re10, im10]   = nyquist(data10.L);

% Squeeze handles the 3D array output of nyquist()
plot(squeeze(re100), squeeze(im100), 'b', 'LineWidth', 1.5, 'DisplayName', 'dW = 100 (Original)');
plot(squeeze(re10), squeeze(im10), 'r', 'LineWidth', 1.5, 'DisplayName', 'dW = 10 (Perturbed)');

% 3. Formatting for Clarity
grid on;
axis equal;             % CRITICAL: Ensures the unit circle actually looks like a circle
axis([-2.5 0.5 -2 2]); % Adjusted to center the unit circle and critical point
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Nyquist Plot Comparison: Effect of Damping Decrease');
legend('show', 'Location', 'best');
%% --- HELPER FUNCTIONS ---

function [stats] = analyze_crane(dw, alpha, K, target_fc, t)
    % A. Define Plant G(s)
    wc_target = target_fc * 2 * pi;
    num_G = [5.1e-3, (0.2 + 0.049 * dw), 250];
    den_G = [5e5, (5e4 + 100 * dw), 6.2e5, 4.9e4, 0];
    G = tf(num_G, den_G);
    
    % B. Define Controller C(s) and Loops
    C = K * tf([alpha/wc_target, 1], [1/(alpha*wc_target), 1]);
    L = C * G;
    T = feedback(L, 1);
    S = feedback(1, L);
    
    % C. Calculate Margins
    [gm_lin, pm] = margin(L);
    [magS, ~] = bode(S, logspace(-3, 2, 1000));
    ms_db = 20*log10(max(squeeze(magS)));
    
    % D. Calculate Settling Time (|e| < 0.025)
    [xl, ~] = step(T, t);
    e = abs(1 - xl);
    idx = find(e >= 0.025, 1, 'last');
    if isempty(idx)
        st = 0; 
    elseif idx == length(t)
        st = NaN; % Did not settle
    else
        st = t(idx+1); 
    end
    
    % Store outputs in structure
    stats.L = L;
    stats.pm = pm;
    stats.gm_db = 20*log10(gm_lin);
    stats.ms_db = ms_db;
    stats.set_time = st;
    if isstable(T), stats.stable = 'Stable'; else stats.stable = 'Unstable'; end
end

%% Part (d): Step Response for Perturbed Case (dw = 10)
% 1. Setup Simulation Parameters
t_sim = 0:0.1:100;
dw_perturbed = 10; % Ensure we are using the lower damping value
target_wc = 0.025 * 2 * pi;

% 2. Reconstruct the Perturbed Plant G_perturbed(s)
num_G_p = [5.1e-3, (0.2 + 0.049 * dw_perturbed), 250];
den_G_p = [5e5, (5e4 + 100 * dw_perturbed), 6.2e5, 4.9e4, 0];
G_p = tf(num_G_p, den_G_p);

% 3. Use the existing Controller (C1) to form the Perturbed Closed-Loop (T_p)
% Note: We use the C1 designed for dw=100 to see how it handles the change.
L_p = C1 * G_p;
T_p = feedback(L_p, 1);

% 4. Perform Step Response for the Perturbed system
[xlp, t_out] = step(T_p, t_sim); 

% 5. Calculate Tracking Error e(t)
e_p = 1 - xlp;

% 6. Calculate Custom Settling Time (|e| < 0.025)
threshold = 0.025;
indices_outside = find(abs(e_p) >= threshold);

if isempty(indices_outside)
    settling_time = 0;
else
    last_index = indices_outside(end);
    if last_index == length(t_sim)
        settling_time = NaN; 
        fprintf('Warning: System with dw=10 did not settle within 100s.\n');
    else
        settling_time = t_out(last_index + 1);
    end
end

% 7. Plotting
figure(6);
clf;

% Subplot 1: Load Position xl (Perturbed)
subplot(2,1,1);
plot(t_out, xlp, 'b', 'LineWidth', 1.5);
ylabel('Position x_l (m)');
title(['Closed-Loop Step Response (Perturbed Case: d_w = ', num2str(dw_perturbed), ')']);
grid on;

% Subplot 2: Tracking Error e (Perturbed)
subplot(2,1,2);
plot(t_out, e_p, 'r', 'LineWidth', 1.5);
hold on;
yline(threshold, '--k', 'Threshold +0.025');
yline(-threshold, '--k', 'Threshold -0.025');
ylabel('Tracking Error e (m)');
xlabel('Time (s)');
title(['Error Signal (Settling Time: ', num2str(settling_time, '%.2f'), 's)']);
grid on;

fprintf('--- Results for d_w = %d ---\n', dw_perturbed);
fprintf('The settling time for |e(t)| < 0.025m is: %.2f seconds\n', settling_time);


%question e)
