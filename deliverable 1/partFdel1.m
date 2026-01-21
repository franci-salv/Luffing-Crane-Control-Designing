%% System Parameters and Initial Conditions

% Generalized coordinates vector: q = [x; theta]
% x: position of the hoist block (m)
% theta: absolute angle of the chain (rad)

% Initial generalized position q(0)
x0 = 11.5; % Initial hoist position (m)
theta0 = pi/3; % Initial chain angle (rad)
q0 = [x0; theta0]; 

% Initial generalized velocity q_dot(0)
x_dot0 = 0; % Initial hoist velocity (m/s)
theta_dot0 = 0; % Initial chain angular velocity (rad/s)
q_dot0 = [x_dot0; theta_dot0]; 

% Input signals (as specified)
FA = 0; % Force applied to the hoist (N) - constant
spd = 1 / (12 * pi); % Speed parameter (for any internal speed dependencies, if applicable)

% Simulation settings
T_sim = 110; % Simulation time in seconds

% Store initial conditions for Simulink (often used as separate variables)
% Note: In Simulink, the initial conditions are typically set in the 
% Integrator blocks directly, referencing these workspace variables.
initial_position = q0;
initial_velocity = q_dot0;


%% Run Simulink Model

% IMPORTANT: Replace 'your_simulink_model_name' with the actual name of your .slx file.
% Your Simulink model must:
% 1. Implement the EoMs: solve for q_ddot (q_double_dot) from M*q_double_dot = F.
% 2. Use two Integrator blocks in series to go from q_double_dot -> q_dot -> q.
% 3. Have 'To Workspace' blocks named 'simout_x' and 'simout_theta' 
%    (or similar) for the x(t) and theta(t) signals.
% 4. Use the variables defined above (e.g., FA, spd) as inputs or parameters.

% Extract state trajectories from q
simOut = sim("TimeTraj.slx");
q_data = simOut.q;    % This is an array: [time x theta] or [x theta] depending on your block

% If q is saved as an array with columns [x, theta]:
x_data = q_data(:,1);
theta_data = q_data(:,2);

% Time vector
time = simOut.tout;

% Extract time and position data from the simulation output
% Assuming your 'To Workspace' blocks save data as 'timeseries' objects
% If you used a different format, adjust the extraction accordingly.

%% Load measured lab data
load('MeasuredSignals.mat');   % loads t_meas, x_meas, theta_meas



%% 📈 Plot 1: Hoist Position x(t)

figure(1);
plot(time, x_data, 'LineWidth', 1.5);

% Required axis limits
xlim([0 110]);          % Time interval [0, 110] s
ylim([11.15 12.26]);    % Position interval [11.15, 12.26] m

% Required labels and title
title('Position of Hoist Block x(t) vs Time');
xlabel('Time (s)');
ylabel('Hoist Position x (m)');

% Required legend
legend('x(t)', 'Location', 'best');

grid on;


%% 📉 Plot 2: Chain Angle θ(t)

figure(2);
plot(time, theta_data, 'LineWidth', 1.5);

% Required axis limits
xlim([0 110]);         % Time interval [0, 110] s
ylim([0.95 2.05]);     % Angle interval [0.95, 2.05] rad

% Required labels and title
title('Absolute Angle of Chain \theta(t) vs Time');
xlabel('Time (s)');
ylabel('Chain Angle \theta (rad)');

% Required legend
legend('\theta(t)', 'Location', 'best');

grid on;

%% Load measured lab data
S = load('MeasuredSignals.mat');   % loads struct with field MeasuredSignals

t_meas     = S.MeasuredSignals.t;
x_meas     = S.MeasuredSignals.x;
theta_meas = S.MeasuredSignals.theta;



%% 📈 Plot 3: Hoist Position x(t) – Simulation vs Measured
figure(3);
plot(time, x_data, 'LineWidth', 1.5); hold on;
plot(t_meas, x_meas, '--', 'LineWidth', 1.5);

xlim([0 110]);
ylim([11.15 12.26]);

title('Hoist Position x(t): Simulation vs Measured');
xlabel('Time (s)');
ylabel('Hoist Position x (m)');

legend('Simulation x(t)', 'Measured x(t)', 'Location', 'best');

grid on;


%% 📉 Plot 4: Chain Angle θ(t) – Simulation vs Measured
figure(4);
plot(time, theta_data, 'LineWidth', 1.5); hold on;
plot(t_meas, theta_meas, '--', 'LineWidth', 1.5);

xlim([0 110]);
ylim([0.95 2.05]);

title('Chain Angle \theta(t): Simulation vs Measured');
xlabel('Time (s)');
ylabel('Chain Angle \theta (rad)');

legend('Simulation \theta(t)', 'Measured \theta(t)', 'Location', 'best');

grid on;
